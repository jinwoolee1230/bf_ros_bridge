"""``bf_ros_bridge`` node: MSP serial <-> ROS topics.

One node owns the serial port (only one process can). Every stream is gated by
params and can be enabled/disabled and rate-set independently.

Publishers (private ns), created only when ``publish/<x>/enabled`` and rate > 0:
  ~imu          sensor_msgs/Imu               <- MSP_RAW_IMU   (REP-103 FLU, SI)
  ~rc           mavros_msgs/RCIn              <- MSP_RC
  ~status       diagnostic_msgs/DiagnosticArray <- MSP_STATUS (+ MSP_BOXIDS)
  ~motor/rpm    std_msgs/Int32MultiArray     <- MSP_MOTOR_TELEMETRY
  ~motor/esc    std_msgs/Float32MultiArray   <- MSP_MOTOR_TELEMETRY (optional)

Subscribers, created only when ``control/<x>_enabled``:
  ~setpoint_raw/attitude    mavros_msgs/AttitudeTarget  -> MSP_CTBR
  ~setpoint/motor_throttle  std_msgs/Float32MultiArray  -> MSP_THROTTLE
"""

import threading

import rospy

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Vector3  # noqa: F401 (kept for downstream clarity)
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray, Int32MultiArray, MultiArrayDimension

try:
    from mavros_msgs.msg import AttitudeTarget, RCIn
    HAVE_MAVROS = True
except Exception:  # noqa: BLE001
    HAVE_MAVROS = False

from . import msp
from .serial_link import SerialLink

# mavros_msgs/AttitudeTarget.type_mask bits
IGNORE_ROLL_RATE = 1
IGNORE_PITCH_RATE = 2
IGNORE_YAW_RATE = 4
IGNORE_THRUST = 64


def _clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


class _Stream(object):
    __slots__ = ("enabled", "rate_hz")

    def __init__(self, enabled, rate_hz):
        self.enabled = enabled
        self.rate_hz = rate_hz


class BfBridge(object):
    def __init__(self):
        gp = rospy.get_param

        self.port = gp("~port", "/dev/ttyACM0")
        self.baud = int(gp("~baud", 115200))
        self.motor_count = max(1, min(8, int(gp("~motor_count", 4))))
        self.frame_id = gp("~frame_id", "bf_imu")

        self.pub_cfg = {
            "rc": self._stream("rc", 20.0),
            "status": self._stream("status", 10.0),
            "motor": self._stream("motor", 50.0),
            "imu": self._stream("imu", 200.0),
        }
        self.motor_esc = bool(gp("~publish/motor/esc", False))

        self.ctbr_enabled = bool(gp("~control/ctbr_enabled", True))
        self.throttle_enabled = bool(gp("~control/throttle_enabled", False))
        self.cmd_rate_hz = float(gp("~control/cmd_rate_hz", 100.0))
        self.mode = str(gp("~control/mode", "auto")).lower()
        self.idle_mode = str(gp("~control/idle_mode", "ctbr")).lower()
        self.command_timeout = float(gp("~control/command_timeout", 0.15))
        self.max_thrust = float(gp("~control/max_thrust", 0.35))
        self.max_rate = float(gp("~control/max_rate_radps", 12.0))
        self.max_motor_throttle = float(gp("~control/max_motor_throttle", 0.35))
        sign = list(gp("~control/body_rate_sign", [1.0, 1.0, 1.0]))
        sign = [float(x) for x in sign] + [1.0, 1.0, 1.0]
        self.body_rate_sign = sign[:3]
        self.body_rate_scale = float(gp("~control/body_rate_scale", 1.0))

        self.acc_1g = float(gp("~imu/acc_1g", 2048.0))
        self.gyro_scale = float(gp("~imu/gyro_scale_dps_per_lsb", 2000.0 / 32768.0))
        self.ori_cov = float(gp("~imu/orientation_covariance", -1.0))
        self.av_var = float(gp("~imu/angular_velocity_stddev", 0.02)) ** 2
        self.la_var = float(gp("~imu/linear_acceleration_stddev", 0.2)) ** 2

        # shared state
        self._lock = threading.Lock()
        self._sp_ctbr = None        # (thrust, [r, p, y], rospy.Time)
        self._sp_throttle = None    # ([m0..], rospy.Time)
        self._last_mode_sent = self.idle_mode
        self._boxids = []           # active-box-index -> permanentId
        self._have_boxids = False
        self._link_up = False
        self._link_err = "startup"
        self._last_status = None

        if not HAVE_MAVROS and (self.pub_cfg["rc"].enabled or self.ctbr_enabled):
            rospy.logwarn("mavros_msgs not found: RC publish and CTBR subscribe are disabled")

        # publishers
        self.pubs = {}
        if self.pub_cfg["imu"].enabled:
            self.pubs["imu"] = rospy.Publisher("~imu", Imu, queue_size=10)
        if self.pub_cfg["rc"].enabled and HAVE_MAVROS:
            self.pubs["rc"] = rospy.Publisher("~rc", RCIn, queue_size=10)
        if self.pub_cfg["status"].enabled:
            self.pubs["status"] = rospy.Publisher("~status", DiagnosticArray, queue_size=5)
        if self.pub_cfg["motor"].enabled:
            self.pubs["motor_rpm"] = rospy.Publisher("~motor/rpm", Int32MultiArray, queue_size=10)
            if self.motor_esc:
                self.pubs["motor_esc"] = rospy.Publisher(
                    "~motor/esc", Float32MultiArray, queue_size=10
                )

        # serial link
        self.link = SerialLink(self.port, self.baud, self._on_frame, self._on_link_change)

        # subscribers
        if self.ctbr_enabled and HAVE_MAVROS:
            rospy.Subscriber("~setpoint_raw/attitude", AttitudeTarget,
                             self._on_attitude, queue_size=20)
        if self.throttle_enabled:
            rospy.Subscriber("~setpoint/motor_throttle", Float32MultiArray,
                             self._on_throttle, queue_size=20)

        # timers
        self._timers = []
        self.link.start()
        self._start_timers()
        rospy.on_shutdown(self._shutdown)

        rospy.loginfo(
            "bf_ros_bridge up: port=%s baud=%d motors=%d | pub[%s] | ctbr=%s throttle=%s "
            "cmd=%.0fHz mode=%s",
            self.port, self.baud, self.motor_count,
            ",".join(k for k, c in self.pub_cfg.items() if c.enabled),
            self.ctbr_enabled, self.throttle_enabled, self.cmd_rate_hz, self.mode,
        )

    # -- param helpers --------------------------------------------------

    def _stream(self, name, default_hz):
        enabled = bool(rospy.get_param("~publish/%s/enabled" % name, True))
        rate = float(rospy.get_param("~publish/%s/rate_hz" % name, default_hz))
        return _Stream(enabled and rate > 0.0, rate)

    def _start_timers(self):
        polls = (
            ("rc", msp.MSP_RC, "rc"),
            ("motor", msp.MSP_MOTOR_TELEMETRY, "motor_rpm"),
            ("imu", msp.MSP_RAW_IMU, "imu"),
        )
        for name, cmd, pub_key in polls:
            cfg = self.pub_cfg[name]
            if not cfg.enabled or pub_key not in self.pubs:
                continue
            self._timers.append(rospy.Timer(
                rospy.Duration(1.0 / cfg.rate_hz), self._make_poll_cb(cmd)
            ))

        if self.pub_cfg["status"].enabled:
            self._timers.append(rospy.Timer(
                rospy.Duration(1.0 / self.pub_cfg["status"].rate_hz), self._status_cb
            ))

        if (self.ctbr_enabled or self.throttle_enabled):
            hz = self.cmd_rate_hz if self.cmd_rate_hz > 0.0 else 100.0
            self._timers.append(rospy.Timer(rospy.Duration(1.0 / hz), self._cmd_cb))

    def _make_poll_cb(self, cmd):
        def cb(_evt):
            self.link.write(msp.make_msp1_request(cmd))
        return cb

    # -- TX: poll status + publish heartbeat --------------------------

    def _status_cb(self, _evt):
        if self.link.connected:
            self.link.write(msp.make_msp1_request(msp.MSP_STATUS))
            if not self._have_boxids:
                self.link.write(msp.make_msp1_request(msp.MSP_BOXIDS))
        self._publish_status()

    # -- TX: control frame -------------------------------------------

    def _cmd_cb(self, _evt):
        now = rospy.Time.now()
        with self._lock:
            sp_c = self._sp_ctbr
            sp_t = self._sp_throttle

        def fresh(sp):
            return sp is not None and (now - sp[-1]).to_sec() <= self.command_timeout

        fc, ft = fresh(sp_c), fresh(sp_t)

        if self.mode == "ctbr":
            mode = "ctbr"
        elif self.mode == "throttle":
            mode = "throttle"
        else:  # auto: throttle wins (matches firmware), else the fresh one, else idle
            if ft:
                mode = "throttle"
            elif fc:
                mode = "ctbr"
            else:
                mode = self._last_mode_sent or self.idle_mode

        # never stream a mode we are not subscribed for
        if mode == "ctbr" and not self.ctbr_enabled:
            mode = "throttle" if self.throttle_enabled else "ctbr"
        if mode == "throttle" and not self.throttle_enabled:
            mode = "ctbr" if self.ctbr_enabled else "throttle"
        self._last_mode_sent = mode

        if mode == "throttle":
            if ft:
                vals = [_clamp(float(v), 0.0, self.max_motor_throttle) for v in sp_t[0]]
                vals = (vals + [0.0] * self.motor_count)[:self.motor_count]
                frame = msp.make_throttle(vals)
            else:
                frame = msp.make_throttle([0.0] * self.motor_count)
        else:  # ctbr
            if fc:
                thr = _clamp(float(sp_c[0]), 0.0, self.max_thrust)
                r = sp_c[1]
                rr = _clamp(r[0] * self.body_rate_sign[0] * self.body_rate_scale,
                            -self.max_rate, self.max_rate)
                pr = _clamp(r[1] * self.body_rate_sign[1] * self.body_rate_scale,
                            -self.max_rate, self.max_rate)
                yr = _clamp(r[2] * self.body_rate_sign[2] * self.body_rate_scale,
                            -self.max_rate, self.max_rate)
                frame = msp.make_ctbr(thr, rr, pr, yr)
            else:
                frame = msp.make_ctbr(0.0, 0.0, 0.0, 0.0)

        self.link.write(frame)

    # -- subscribers -------------------------------------------------

    def _on_attitude(self, m):
        tm = m.type_mask
        thr = 0.0 if (tm & IGNORE_THRUST) else float(m.thrust)
        rx = 0.0 if (tm & IGNORE_ROLL_RATE) else float(m.body_rate.x)
        ry = 0.0 if (tm & IGNORE_PITCH_RATE) else float(m.body_rate.y)
        rz = 0.0 if (tm & IGNORE_YAW_RATE) else float(m.body_rate.z)
        with self._lock:
            self._sp_ctbr = (thr, [rx, ry, rz], rospy.Time.now())

    def _on_throttle(self, m):
        vals = list(m.data)
        if len(vals) != self.motor_count:
            rospy.logwarn_throttle(
                2.0, "motor_throttle length %d != motor_count %d; ignoring"
                % (len(vals), self.motor_count)
            )
            return
        with self._lock:
            self._sp_throttle = ([float(v) for v in vals], rospy.Time.now())

    # -- RX frame dispatch (serial thread) --------------------------

    def _on_frame(self, cmd, payload, ok):
        if not ok:
            return
        if cmd == msp.MSP_STATUS:
            st = msp.decode_status(payload)
            if st is not None:
                with self._lock:
                    self._last_status = st
        elif cmd == msp.MSP_RAW_IMU:
            self._publish_imu(payload)
        elif cmd == msp.MSP_RC:
            self._publish_rc(payload)
        elif cmd == msp.MSP_MOTOR_TELEMETRY:
            self._publish_motor(payload)
        elif cmd == msp.MSP_BOXIDS:
            with self._lock:
                self._boxids = list(payload)
                self._have_boxids = True

    def _on_link_change(self, up, err):
        with self._lock:
            self._link_up = up
            self._link_err = err
            if up:
                self._have_boxids = False  # refetch box map on (re)connect
        if up:
            rospy.loginfo("bf_ros_bridge: serial link up (%s)", self.port)
        else:
            rospy.logwarn("bf_ros_bridge: serial link down: %s", err)

    # -- publishers ------------------------------------------------

    def _publish_imu(self, payload):
        p = self.pubs.get("imu")
        if p is None:
            return
        d = msp.decode_raw_imu(payload, self.acc_1g, self.gyro_scale)
        if d is None:
            return
        # This fork's MSP_RAW_IMU is already in REP-103 FLU (X-fwd, Y-left, Z-up):
        # bench-verified props-off (right-side-down -> +lin.acc.y, nose-up -> +ang.vel.y,
        # yaw-right -> +ang.vel.z, roll-right -> +ang.vel.x). Only unit scaling is
        # applied here (deg/s -> rad/s, g -> m/s^2). Do NOT re-add axis/sign flips
        # without re-checking on the bench.
        gx, gy, gz = (v * msp.DEG2RAD for v in d["gyro"])
        ax, ay, az = (v * msp.G_TO_MS2 for v in d["acc"])

        m = Imu()
        m.header.stamp = rospy.Time.now()
        m.header.frame_id = self.frame_id
        m.orientation_covariance[0] = self.ori_cov
        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = gx, gy, gz
        m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = ax, ay, az
        for k in (0, 4, 8):
            m.angular_velocity_covariance[k] = self.av_var
            m.linear_acceleration_covariance[k] = self.la_var
        p.publish(m)

    def _publish_rc(self, payload):
        p = self.pubs.get("rc")
        if p is None:
            return
        ch = msp.decode_rc(payload)
        if ch is None:
            return
        m = RCIn()
        m.header.stamp = rospy.Time.now()
        m.rssi = 0
        m.channels = ch
        p.publish(m)

    def _publish_motor(self, payload):
        motors = msp.decode_motor_telemetry(payload)
        if not motors:
            return
        n = len(motors)
        rp = self.pubs.get("motor_rpm")
        if rp is not None:
            a = Int32MultiArray()
            a.layout.dim = [MultiArrayDimension(label="motor", size=n, stride=n)]
            a.data = [mm["rpm"] for mm in motors]
            rp.publish(a)
        ep = self.pubs.get("motor_esc")
        if ep is not None:
            a = Float32MultiArray()
            a.layout.dim = [
                MultiArrayDimension(label="motor", size=n, stride=n * 4),
                MultiArrayDimension(label="field", size=4, stride=4),
            ]
            data = []
            for mm in motors:
                data += [mm["voltage_v"], mm["current_a"],
                         float(mm["temp_c"]), float(mm["consumption_mah"])]
            a.data = data
            ep.publish(a)

    def _box_active(self, perm, st, boxids, have):
        if not have or st is None:
            return None
        try:
            idx = boxids.index(perm)
        except ValueError:
            return None
        return bool(st["flight_mode_bits"] & (1 << idx))

    def _publish_status(self):
        p = self.pubs.get("status")
        if p is None:
            return
        with self._lock:
            st = self._last_status
            up = self._link_up
            err = self._link_err
            boxids = list(self._boxids)
            have = self._have_boxids

        ds = DiagnosticStatus()
        ds.name = "bf_ros_bridge: FC"
        ds.hardware_id = self.port
        kv = ds.values

        def add(key, val):
            kv.append(KeyValue(key=key, value=str(val)))

        def tri(v):
            return "unknown" if v is None else ("true" if v else "false")

        add("link", "up" if up else "down")
        add("link_error", err or "")

        if st is None:
            ds.level = DiagnosticStatus.ERROR if not up else DiagnosticStatus.WARN
            ds.message = ("link down: %s" % err) if not up else "no MSP_STATUS yet"
        else:
            armed = self._box_active(msp.PERM_ARM, st, boxids, have)
            adf = st["arming_flags"]
            add("armed", tri(armed))
            add("arming_disable_flags", "0x%08x" % adf)
            add("arming_disable_names", ",".join(st["arming_flag_names"]))
            add("cpu_load_pct", st["system_load"])
            add("cycle_time_us", st["cycle_time"])
            add("loop_hz", st["loop_hz"])
            add("i2c_errors", st["i2c_errors"])
            add("pid_profile", st["pid_profile"])
            add("sensors", ",".join(st["sensors"]))
            add("msp_ctbr_box", tri(self._box_active(msp.PERM_MSP_CTBR, st, boxids, have)))
            add("msp_throttle_box", tri(self._box_active(msp.PERM_MSP_THROTTLE, st, boxids, have)))
            if not up:
                ds.level = DiagnosticStatus.ERROR
                ds.message = "link down"
            elif adf:
                ds.level = DiagnosticStatus.WARN
                ds.message = "arming disabled: " + ",".join(st["arming_flag_names"])
            else:
                ds.level = DiagnosticStatus.OK
                ds.message = "armed" if armed else "ready"

        arr = DiagnosticArray()
        arr.header.stamp = rospy.Time.now()
        arr.status = [ds]
        p.publish(arr)

    # -- shutdown -----------------------------------------------------

    def _shutdown(self):
        try:
            for _ in range(10):
                if self._last_mode_sent == "throttle":
                    self.link.write(msp.make_throttle([0.0] * self.motor_count))
                else:
                    self.link.write(msp.make_ctbr(0.0, 0.0, 0.0, 0.0))
                rospy.sleep(0.01)
        except Exception:  # noqa: BLE001
            pass
        self.link.stop()


def main():
    rospy.init_node("bf_ros_bridge")
    BfBridge()
    rospy.spin()

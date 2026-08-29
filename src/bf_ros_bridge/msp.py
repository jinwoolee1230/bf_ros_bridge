"""MultiWii Serial Protocol (MSP) codec for the custom Betaflight fork.

Self-contained; adapted from ``BF_MSP/monitor.py`` so the dashboard stays
untouched. Encoders build frames, decoders parse payloads, ``parse_frames``
walks an RX buffer of MSPv1 (``$M>``) responses.

Firmware external-control commands (MSPv2, NO_REPLY, 100 ms freshness window):

* ``MSP_CTBR`` 0x30F0 - 16-byte payload = 4x float32 LE
  ``[collective_thrust 0..1, roll_rate, pitch_rate, yaw_rate]`` rad/s.
  Firmware converts rad/s -> deg/s and clamps to +/- 1998 deg/s.
* ``MSP_THROTTLE`` 0x30F1 - N*float32 LE, one normalized throttle 0..1 per
  motor, in motor order. Length must equal the craft's motor count exactly.
"""

import struct

# --- command IDs ----------------------------------------------------------
MSP_CTBR = 0x30F0
MSP_THROTTLE = 0x30F1

MSP_STATUS = 101
MSP_RAW_IMU = 102
MSP_RC = 105
MSP_MOTOR_TELEMETRY = 139
MSP_BOXIDS = 119

# --- box permanent IDs (firmware src/main/msp/msp_box.c) -----------------
PERM_ARM = 0
PERM_MSP_CTBR = 55
PERM_MSP_THROTTLE = 56

# firmware EXTERNAL_CONTROL_TIMEOUT_US
EXTERNAL_CONTROL_TIMEOUT_S = 0.1

# firmware rad/s -> deg/s clamp in setExternalRateThrust()
MAX_BODY_RATE_RADPS = 1998.0 * 0.017453292519943295  # ~= 34.87

DEG2RAD = 0.017453292519943295
G_TO_MS2 = 9.80665

ARMING_FLAGS = {
    0: "NO_GYRO", 1: "FAILSAFE", 2: "RX_FAILSAFE", 3: "NOT_DISARMED",
    4: "BOXFAILSAFE", 5: "RUNAWAY", 6: "CRASH", 7: "THROTTLE", 8: "ANGLE",
    9: "BOOT_GRACE", 10: "NOPREARM", 11: "LOAD", 12: "CALIBRATING", 13: "CLI",
    14: "CMS", 15: "BST", 16: "MSP", 17: "PARALYZE", 18: "GPS", 19: "RESC",
    20: "DSHOT_TELEM", 21: "REBOOT_REQUIRED", 22: "DSHOT_BITBANG",
    23: "ACC_CALIBRATION", 24: "MOTOR_PROTOCOL", 25: "ARM_SWITCH",
}

SENSOR_BITS = [
    (0, "ACC"), (1, "BARO"), (2, "MAG"), (3, "GPS"),
    (4, "RANGEFINDER"), (5, "GYRO"),
]


# --- framing ------------------------------------------------------------------

def crc8_dvb_s2(crc, value):
    crc ^= value
    for _ in range(8):
        if crc & 0x80:
            crc = ((crc << 1) ^ 0xD5) & 0xFF
        else:
            crc = (crc << 1) & 0xFF
    return crc


def make_msp2(cmd, payload=b""):
    header = struct.pack("<BHH", 0, cmd, len(payload))
    crc = 0
    for b in header + payload:
        crc = crc8_dvb_s2(crc, b)
    return b"$X<" + header + payload + bytes([crc])


def make_msp1_request(cmd):
    size = 0
    checksum = size ^ cmd
    return b"$M<" + bytes([size, cmd, checksum])


# --- encoders (ROS -> FC) ---------------------------------------------------

def make_ctbr(thrust, roll, pitch, yaw):
    """MSP_CTBR: collective thrust (0..1) + body rates in rad/s, 4x float32."""
    payload = struct.pack(
        "<ffff",
        max(0.0, min(1.0, float(thrust))),
        float(roll),
        float(pitch),
        float(yaw),
    )
    return make_msp2(MSP_CTBR, payload)


def make_throttle(values):
    """MSP_THROTTLE: one float32 normalized throttle (0..1) per motor.

    Payload length must equal the craft's motor count; the firmware rejects any
    other length and any non-finite value.
    """
    vals = [max(0.0, min(1.0, float(v))) for v in values]
    payload = struct.pack("<%df" % len(vals), *vals)
    return make_msp2(MSP_THROTTLE, payload)


# --- decoders (FC -> ROS) -------------------------------------------------

def decode_status(payload):
    """Decode an MSP_STATUS payload into a plain dict (None if too short)."""
    if len(payload) < 17:
        return None

    cycle_time = struct.unpack_from("<H", payload, 0)[0]
    i2c_errors = struct.unpack_from("<H", payload, 2)[0]
    sensor_status = struct.unpack_from("<H", payload, 4)[0]
    flight_mode_flags = struct.unpack_from("<I", payload, 6)[0]
    pid_profile = payload[10]
    system_load = struct.unpack_from("<H", payload, 11)[0]

    extra_mode_bytes = payload[15]
    p = 16 + extra_mode_bytes

    # Full flight-mode bitmask: first 4 bytes (offset 6) + `extra_mode_bytes`
    # more after the count byte. Bit i == the i-th active box in MSP_BOXIDS.
    fm_bytes = bytes(payload[6:10]) + bytes(payload[16:16 + extra_mode_bytes])
    flight_mode_bits = int.from_bytes(fm_bytes, "little")

    arming_flags = 0
    if len(payload) >= p + 5:
        arming_flags = struct.unpack_from("<I", payload, p + 1)[0]

    names = [name for bit, name in ARMING_FLAGS.items() if arming_flags & (1 << bit)]
    sensors = [name for bit, name in SENSOR_BITS if sensor_status & (1 << bit)]
    loop_hz = (1_000_000.0 / cycle_time) if cycle_time else 0.0

    return {
        "cycle_time": cycle_time,
        "loop_hz": round(loop_hz, 1),
        "i2c_errors": i2c_errors,
        "sensor_status": sensor_status,
        "sensors": sensors,
        "flight_mode_flags": flight_mode_flags,
        "flight_mode_bits": flight_mode_bits,
        "pid_profile": pid_profile,
        "system_load": system_load,
        "arming_flags": arming_flags,
        "arming_flag_names": names,
    }


def decode_raw_imu(payload, acc_1g, gyro_dps_per_lsb):
    """MSP_RAW_IMU (cmd 102): 9x int16 LE = acc[3], gyro[3], mag[3].

    acc is raw ``acc.accADC`` counts (``acc_1g`` counts == 1 g); gyro is raw
    counts (``* gyroDev.scale`` -> deg/s). Returns g and deg/s in the sensor
    frame; frame/unit conversion is the caller's job. mag is ignored.
    """
    if len(payload) < 18:
        return None
    ax, ay, az, gx, gy, gz = struct.unpack_from("<6h", payload, 0)
    inv_g = (1.0 / acc_1g) if acc_1g else 0.0
    return {
        "acc": [ax * inv_g, ay * inv_g, az * inv_g],
        "gyro": [gx * gyro_dps_per_lsb, gy * gyro_dps_per_lsb, gz * gyro_dps_per_lsb],
    }


def decode_rc(payload):
    """MSP_RC (cmd 105): N x uint16 channel values (us), N = dataSize / 2."""
    n = len(payload) // 2
    if n == 0:
        return None
    return list(struct.unpack_from("<%dH" % n, payload, 0))


def decode_motor_telemetry(payload):
    """MSP_MOTOR_TELEMETRY (cmd 139): u8 count, then per motor 13 bytes:
    u32 rpm, u16 invalidPct (0.01%), u8 tempC, u16 voltage (0.01V),
    u16 current (0.01A), u16 consumption (mAh).

    Returns a list of dicts, one per motor.
    """
    if not payload:
        return None
    count = payload[0]
    motors = []
    for i in range(count):
        off = 1 + i * 13
        if off + 13 > len(payload):
            break
        rpm, invalid_pct, temp_c, volt, curr, mah = struct.unpack_from(
            "<IHBHHH", payload, off
        )
        motors.append({
            "rpm": int(rpm),
            "invalid_pct": invalid_pct / 100.0,
            "temp_c": int(temp_c),
            "voltage_v": volt / 100.0,
            "current_a": curr / 100.0,
            "consumption_mah": int(mah),
        })
    return motors


# --- RX buffer walker -------------------------------------------------------

def parse_frames(buffer, on_frame):
    """Consume complete MSPv1 ($M>) response frames from ``buffer`` (bytearray).

    Calls ``on_frame(cmd: int, payload: bytes, ok: bool)`` for every frame.
    ``ok`` is the XOR-checksum result; on a bad checksum the cmd is still
    reported (with an empty payload) so callers can count errors.
    """
    while True:
        pos = buffer.find(b"$M>")
        if pos < 0:
            if len(buffer) > 1024:
                del buffer[:-16]
            return

        if pos:
            del buffer[:pos]

        if len(buffer) < 6:
            return

        size = buffer[3]
        frame_len = 6 + size
        if len(buffer) < frame_len:
            return

        cmd = buffer[4]
        payload = bytes(buffer[5:5 + size])
        checksum = buffer[5 + size]

        calc = size ^ cmd
        for b in payload:
            calc ^= b

        del buffer[:frame_len]

        on_frame(cmd, payload, True) if calc == checksum else on_frame(cmd, b"", False)

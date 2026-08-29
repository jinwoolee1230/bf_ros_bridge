# bf_ros_bridge

ROS 1 (Noetic) bridge between the custom Betaflight fork's MSP link and ROS.

One node owns the serial port. It **publishes** RC / flight status / motor RPM /
IMU decoded from MSP replies, and **subscribes** to two control setpoint streams,
encodes them to MSP and streams them to the flight controller:

| direction | ROS topic (private) | type | MSP |
|---|---|---|---|
| pub | `~imu` | `sensor_msgs/Imu` | `MSP_RAW_IMU` (102) |
| pub | `~rc` | `mavros_msgs/RCIn` | `MSP_RC` (105) |
| pub | `~status` | `diagnostic_msgs/DiagnosticArray` | `MSP_STATUS` (101) + `MSP_BOXIDS` (119) |
| pub | `~motor/rpm` | `std_msgs/Int32MultiArray` | `MSP_MOTOR_TELEMETRY` (139) |
| pub | `~motor/esc` | `std_msgs/Float32MultiArray` | same frame, `[V, A, °C, mAh]` per motor (opt-in) |
| sub | `~setpoint_raw/attitude` | `mavros_msgs/AttitudeTarget` | `MSP_CTBR` (0x30F0) — thrust + body rates |
| sub | `~setpoint/motor_throttle` | `std_msgs/Float32MultiArray` | `MSP_THROTTLE` (0x30F1) — per-motor 0..1 |

Only standard messages — no custom `.msg`, nothing to generate.

## Control modes

The firmware acts on whichever command matches the **AUX box** the pilot has
active (`MSP CTBR` permId 55 / `MSP THROTTLE` permId 56; THROTTLE wins if both).
This bridge only chooses which frame it *streams*; you still set the AUX switch.

* **CTBR** (`~setpoint_raw/attitude`): `thrust` -> collective thrust `0..1`;
  `body_rate.{x,y,z}` -> `MSP_CTBR` `{roll, pitch, yaw}` **rad/s, pass-through**
  (mavros base_link FLU is taken as the firmware's convention). `type_mask`
  IGNORE bits zero the corresponding field. The firmware converts rad/s -> deg/s
  and clamps to ±1998 deg/s.
* **THROTTLE** (`~setpoint/motor_throttle`): `data` must be exactly
  `motor_count` floats `0..1`, in motor order. Direct to the motor output — no
  attitude PID, no mixer, no RPM loop. Wrong length is warned and ignored.

`control/mode`: `ctbr` | `throttle` | `auto`. In `auto` the bridge streams
whichever setpoint is *fresh* (received within `control/command_timeout`);
if both are fresh, `throttle` wins (matches the firmware); if neither is fresh
it sends zero keep-alive frames of `control/idle_mode`.

Every fresh setpoint is sent immediately; there is no separate output-enable
service. When a setpoint goes stale the bridge streams a zero-thrust /
zero-throttle keep-alive frame so the FC's 100 ms freshness window never trips
on a transient gap.

## Build

The package folder lives at `~/Desktop/BF_MSP/bf_ros_bridge`. Symlink it into any
catkin workspace and build:

```bash
ln -s ~/Desktop/BF_MSP/bf_ros_bridge ~/Projects/catkin_ws/src/bf_ros_bridge
cd ~/Projects/catkin_ws
catkin build bf_ros_bridge        # or: catkin_make
source devel/setup.bash
```

Pure Python (`rospy`); the build only installs the node/launch/config and
validates `package.xml`. Runtime needs `pyserial` importable by the interpreter
that runs the node.

If your `cmake` is 4.x it rejects Noetic's catkin toplevel
(`Compatibility with CMake < 3.5 has been removed`). Build with the policy
shim (needed for any Noetic package on this box, not just this one):

```bash
catkin_make --cmake-args -DCMAKE_POLICY_VERSION_MINIMUM=3.5
# or:  catkin config --cmake-args -DCMAKE_POLICY_VERSION_MINIMUM=3.5 && catkin build
```

## Run

```bash
roslaunch bf_ros_bridge bf_ros_bridge.launch port:=/dev/ttyACM0
```

Every knob is a launch arg (see the top of `launch/bf_ros_bridge.launch`), e.g.:

```bash
roslaunch bf_ros_bridge bf_ros_bridge.launch \
    port:=/dev/ttyACM0 motor_count:=4 \
    pub_imu:=true pub_imu_hz:=250  pub_motor:=true pub_motor_hz:=100 \
    pub_rc:=false  pub_status_hz:=5 \
    ctbr_enabled:=true throttle_enabled:=false cmd_hz:=150 \
    output_mode:=auto max_thrust:=0.30 max_rate_radps:=10.0 \
    attitude_topic:=/mavros/setpoint_raw/attitude
```

Disable a stream by setting its `pub_*:=false` (publisher + its MSP poll are not
created) or its Hz to 0.

## Verify

**1. Unit (no hardware)**

```bash
python3 ~/Desktop/BF_MSP/bf_ros_bridge/test/test_msp.py
```

**2. No-FC smoke**

```bash
roslaunch bf_ros_bridge bf_ros_bridge.launch port:=/dev/ttyNOPE
rostopic list                                  # all ~ topics advertised
rostopic echo -n1 /bf_ros_bridge/status        # level: 2 (ERROR), link: down
```

**3. With FC — PROPS OFF**

```bash
rostopic hz  /bf_ros_bridge/imu                 # ~ pub_imu_hz
rostopic echo -n1 /bf_ros_bridge/imu            # at rest: gyro ~0, lin.acc.z ~ +9.8 (FLU up)
rostopic echo -n1 /bf_ros_bridge/status        # arming flags plausible, armed: false
rostopic echo /bf_ros_bridge/motor/rpm

# CTBR: set craft AUX to "MSP CTBR", ARM, then
rostopic pub -r 30 /bf_ros_bridge/setpoint_raw/attitude mavros_msgs/AttitudeTarget \
  '{type_mask: 128, body_rate: {x: 0.5, y: 0.0, z: 0.0}, thrust: 0.0}'
#   -> only the ROLL-axis motors respond, in the expected direction.
#      Repeat for y (pitch) and z (yaw); flip control/body_rate_sign[i] for any inverted axis.

# THROTTLE: set craft AUX to "MSP THROTTLE", relaunch with throttle_enabled:=true output_mode:=throttle
rostopic pub -r 30 /bf_ros_bridge/setpoint/motor_throttle std_msgs/Float32MultiArray \
  '{data: [0.05, 0.0, 0.0, 0.0]}'
#   -> only motor 1 spins. Stop publishing -> motors stop within ~150 ms.
```

Make sure the FC's `serial_update_rate_hz` (CLI) is **≥** the sum of all enabled
poll rates + `cmd_hz`, or MSP replies start dropping.

## Gotchas

* **Interpreter**: run from a shell with `/opt/ros/noetic/setup.bash` sourced and
  conda **deactivated**, so `bf_bridge_node`'s `#!/usr/bin/env python3` is the
  system `python3` (it has `rospy`, and `pyserial` via `~/.local`).
* **One port, one owner**: stop `../monitor.py` before launching the bridge.
* **CTBR axis pass-through is an assumption.** The firmware stores the rates
  straight into Betaflight's native setpoint array. Bench-verify every axis
  props-off and correct with `control/body_rate_sign`. Only if an axis needs
  *reordering* (not just a sign) does `bridge.py:_cmd_cb` need editing.
* **IMU scaling** (`imu/acc_1g`, `imu/gyro_scale_dps_per_lsb`) is board-specific;
  defaults are for the MICOAIR743V2 (BMI270). Cross-check against a known
  rotation / a level surface.
* `~status` is a fixed-rate heartbeat (so "link down" is observable); the other
  streams publish the instant an MSP reply is decoded.

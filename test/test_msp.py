#!/usr/bin/env python3
"""Plain-python unit checks for bf_ros_bridge.msp (no ROS needed).

    python3 test/test_msp.py
"""

import os
import struct
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from bf_ros_bridge import msp  # noqa: E402


def _crc8(data):
    crc = 0
    for b in data:
        crc = msp.crc8_dvb_s2(crc, b)
    return crc


def _msp1_response(cmd, payload):
    size = len(payload)
    body = bytes([size, cmd]) + payload
    chk = 0
    for b in body:
        chk ^= b
    return b"$M>" + body + bytes([chk])


def test_crc_vector():
    # DVB-S2 CRC8 of {0x00} == 0x00; of {0x01} == 0xD5
    assert _crc8(b"\x00") == 0x00
    assert _crc8(b"\x01") == 0xD5


def test_make_ctbr_layout():
    f = msp.make_ctbr(0.5, 1.0, -2.0, 3.0)
    assert f[:3] == b"$X<"
    assert struct.unpack_from("<H", f, 4)[0] == msp.MSP_CTBR
    assert struct.unpack_from("<H", f, 6)[0] == 16
    vals = struct.unpack_from("<ffff", f, 8)
    assert abs(vals[0] - 0.5) < 1e-6
    assert abs(vals[2] + 2.0) < 1e-6
    assert _crc8(f[3:3 + 5 + 16]) == f[-1]
    # thrust clamps to 0..1
    assert struct.unpack_from("<f", msp.make_ctbr(9.0, 0, 0, 0), 8)[0] == 1.0


def test_make_throttle_layout():
    f = msp.make_throttle([0.0, 0.25, 0.5, 1.0])
    assert struct.unpack_from("<H", f, 6)[0] == 16
    assert struct.unpack_from("<4f", f, 8) == (0.0, 0.25, 0.5, 1.0)
    assert _crc8(f[3:3 + 5 + 16]) == f[-1]
    # each value clamps to 0..1
    f2 = msp.make_throttle([2.0, -1.0])
    assert struct.unpack_from("<2f", f2, 8) == (1.0, 0.0)


def test_parse_frames_and_rc():
    rc = struct.pack("<8H", 1500, 1500, 1000, 1500, 2000, 1000, 1500, 1800)
    got = []
    buf = bytearray(b"\x00garbage" + _msp1_response(msp.MSP_RC, rc))
    msp.parse_frames(buf, lambda c, p, ok: got.append((c, p, ok)))
    assert len(got) == 1
    cmd, payload, ok = got[0]
    assert cmd == msp.MSP_RC and ok is True
    assert msp.decode_rc(payload)[4] == 2000


def test_parse_frames_bad_checksum():
    frame = bytearray(_msp1_response(msp.MSP_RC, struct.pack("<4H", 1, 2, 3, 4)))
    frame[-1] ^= 0xFF
    got = []
    msp.parse_frames(frame, lambda c, p, ok: got.append(ok))
    assert got == [False]


def test_parse_frames_partial_then_rest():
    full = _msp1_response(msp.MSP_STATUS, b"\x00" * 22)
    buf = bytearray(full[:7])
    got = []
    msp.parse_frames(buf, lambda c, p, ok: got.append(ok))
    assert got == []            # incomplete: nothing emitted, buffer kept
    buf.extend(full[7:])
    msp.parse_frames(buf, lambda c, p, ok: got.append(ok))
    assert got == [True]


def test_decode_motor_telemetry():
    payload = bytes([4])
    for i in range(4):
        payload += struct.pack("<IHBHHH", 12000 + i, 0, 25, 1580, 120, 300)
    motors = msp.decode_motor_telemetry(payload)
    assert len(motors) == 4
    assert motors[0]["rpm"] == 12000
    assert motors[3]["rpm"] == 12003
    assert abs(motors[0]["voltage_v"] - 15.80) < 1e-6
    assert abs(motors[0]["current_a"] - 1.20) < 1e-6
    assert motors[0]["temp_c"] == 25


def test_decode_raw_imu():
    # az = +1 g worth of counts, everything else zero
    payload = struct.pack("<9h", 0, 0, 2048, 10, -20, 30, 0, 0, 0)
    d = msp.decode_raw_imu(payload, 2048.0, 2000.0 / 32768.0)
    assert abs(d["acc"][2] - 1.0) < 1e-6
    assert abs(d["gyro"][0] - 10 * (2000.0 / 32768.0)) < 1e-6
    assert abs(d["gyro"][1] + 20 * (2000.0 / 32768.0)) < 1e-6


def test_decode_status():
    # minimal MSP_STATUS: cycle=250, load=17, 0 extra mode bytes, arming flags bit3
    p = bytearray(21)
    struct.pack_into("<H", p, 0, 250)     # cycle time
    struct.pack_into("<H", p, 11, 17)     # system load
    p[15] = 0                             # extra mode bytes
    p[16] = 4                             # arming disable flag count
    struct.pack_into("<I", p, 17, 1 << 3)  # NOT_DISARMED
    st = msp.decode_status(bytes(p))
    assert st["cycle_time"] == 250
    assert st["system_load"] == 17
    assert st["arming_flags"] == (1 << 3)
    assert "NOT_DISARMED" in st["arming_flag_names"]


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items())
             if k.startswith("test_") and callable(v)]
    for fn in tests:
        fn()
        print("ok  ", fn.__name__)
    print("\n%d passed" % len(tests))

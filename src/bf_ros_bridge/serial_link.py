"""Threaded serial owner: one RX thread, locked writes, auto-reconnect.

Mirrors ``BF_MSP/monitor.py``'s ``Worker`` I/O loop without the web/integrity
machinery. The RX thread reads bytes and hands complete frames to ``on_frame``;
any thread may call :meth:`write`.
"""

import threading

import serial

from . import msp


class SerialLink(object):
    def __init__(self, port, baud, on_frame, on_link_change=None, reconnect_s=1.5):
        self.port = port
        self.baud = baud
        self._on_frame = on_frame
        self._on_link_change = on_link_change or (lambda up, err: None)
        self._reconnect_s = reconnect_s

        self._ser = None
        self._wlock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None

        self.connected = False
        self.last_error = "not started"

    # -- lifecycle ---------------------------------------------------------

    def start(self):
        self._thread = threading.Thread(
            target=self._run, name="bf_serial_rx", daemon=True
        )
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        try:
            if self._ser is not None:
                self._ser.close()
        except Exception:  # noqa: BLE001
            pass

    # -- write -----------------------------------------------------------

    def write(self, frame):
        """Send raw bytes. Returns False if the link is down or the write fails."""
        ser = self._ser
        if ser is None or not self.connected:
            return False
        try:
            with self._wlock:
                ser.write(frame)
            return True
        except (serial.SerialException, OSError) as e:
            self._set_link(False, str(e))
            return False

    # -- internals -----------------------------------------------------

    def _set_link(self, up, err=""):
        if up == self.connected and err == self.last_error:
            return
        self.connected = up
        self.last_error = err
        try:
            self._on_link_change(up, err)
        except Exception:  # noqa: BLE001
            pass

    def _run(self):
        buf = bytearray()
        while not self._stop.is_set():
            try:
                self._ser = serial.Serial(self.port, self.baud, timeout=0)
            except Exception as e:  # noqa: BLE001 - surface any open failure
                self._set_link(False, str(e))
                self._stop.wait(self._reconnect_s)
                continue

            self._set_link(True, "")
            buf = bytearray()
            try:
                while not self._stop.is_set():
                    data = self._ser.read(4096)
                    if data:
                        buf.extend(data)
                        msp.parse_frames(buf, self._on_frame)
                    else:
                        self._stop.wait(0.001)
            except (serial.SerialException, OSError) as e:
                self._set_link(False, str(e))
            finally:
                try:
                    self._ser.close()
                except Exception:  # noqa: BLE001
                    pass
                self._ser = None

            if not self._stop.is_set():
                self._set_link(False, "reconnecting")
                self._stop.wait(self._reconnect_s)

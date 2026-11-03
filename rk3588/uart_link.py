import serial
import struct
import threading
import time


class FpgaLink:
    def __init__(self, port, baud=115200, timeout=0.1):
        self._ser = serial.Serial(port, baud, timeout=timeout)
        self._lock = threading.Lock()
        self._alive = True
        self.telemetry = None
        self._t = threading.Thread(target=self._reader, daemon=True)
        self._t.start()

    def close(self):
        self._alive = False
        try:
            self._ser.close()
        except Exception:
            pass

    def _write(self, frame):
        with self._lock:
            self._ser.write(frame)

    def write_reg(self, addr, val):
        val &= 0xFFFF
        hi = (val >> 8) & 0xFF
        lo = val & 0xFF
        cmd = 0x01
        chk = (cmd + addr + hi + lo) & 0xFF
        self._write(bytes([0x55, cmd, addr, hi, lo, chk]))

    def set_mode(self, mode):
        self.write_reg(0x00, mode & 0x7)

    def set_velocity(self, v, w):
        self.write_reg(0x01, v & 0xFFFF)
        self.write_reg(0x02, w & 0xFFFF)

    def set_rotate(self, deg):
        self.write_reg(0x03, deg & 0xFFFF)

    def pulse_start(self):
        self.write_reg(0x04, 0x0002)

    def _reader(self):
        buf = bytearray()
        while self._alive:
            try:
                data = self._ser.read(256)
            except Exception:
                break
            if not data:
                continue
            buf.extend(data)
            buf = self._parse(buf)

    def _parse(self, buf):
        while len(buf) >= 19:
            if buf[0] == 0xA5 and buf[1] == 0x5A:
                payload = buf[2:18]
                if (sum(payload) & 0xFF) == buf[18]:
                    roll, pitch, yaw, dl, dc, dr, spd_l, spd_r = struct.unpack('>8h', bytes(payload))
                    self.telemetry = {
                        'roll': roll / 128.0,
                        'pitch': pitch / 128.0,
                        'yaw': yaw / 128.0,
                        'dl': dl,
                        'dc': dc,
                        'dr': dr,
                        'spd_l': spd_l,
                        'spd_r': spd_r,
                    }
                    buf = buf[19:]
                else:
                    buf = buf[1:]
            else:
                buf = buf[1:]
        return buf

#!/usr/bin/env python3
"""
FPGA 机器人串口调试工具（pip install pyserial）
用法示例（Windows 串口名如 COM5，Linux 如 /dev/ttyUSB0）：
  python regtool.py COM5 mon                      # 实时看遥测：姿态/三路距离/左右轮速/状态
  python regtool.py COM5 rd 0x48                  # 读寄存器
  python regtool.py COM5 wr 0x05 300              # 写寄存器（十进制或 0x 十六进制，负数自动补码）
  python regtool.py COM5 drive 40 0               # 手动：v=40 w=0（计数/10ms），模式1，看门狗 2 秒
  python regtool.py COM5 stop
  python regtool.py COM5 rot 90                   # 原地左转 90°（模式3）
  python regtool.py COM5 straight 40              # 直行+航向保持+避障（模式2，任务3）
  python regtool.py COM5 mission "90,2000;-90,1500;0,1000"   # 航点：每段 转角(度),距离(计数)；写入后启动（模式4）
  python regtool.py COM5 zero                     # 偏航角清零
"""
import sys, struct, time
try:
    import serial
except ImportError:
    sys.exit("请先 pip install pyserial")

R = dict(MODE=0x00, V=0x01, W=0x02, ROT=0x03, CTRL=0x04, KPS=0x05, KIS=0x06, KFS=0x07,
         STOP=0x08, SLOW=0x09, NLEG=0x0A, WAV=0x0B, KPH=0x0C, WMAX=0x0D, WMIN=0x0E, TOL=0x0F)

def s16(v): v &= 0xFFFF; return v - 65536 if v & 0x8000 else v

class Bot:
    def __init__(self, port):
        self.s = serial.Serial(port, 115200, timeout=0.3)

    def wr(self, addr, val):
        v = val & 0xFFFF
        b = [0x01, addr, v >> 8, v & 0xFF]
        self.s.write(bytes([0x55] + b + [sum(b) & 0xFF]))

    def rd(self, addr):
        b = [0x02, addr, 0, 0]
        self.s.reset_input_buffer()
        self.s.write(bytes([0x55] + b + [sum(b) & 0xFF]))
        t0 = time.time(); buf = b""
        while time.time() - t0 < 1:
            buf += self.s.read(64)
            i = buf.find(b"\x55\x82")
            if i >= 0 and len(buf) >= i + 6:
                f = buf[i:i + 6]
                if (sum(f[1:5]) & 0xFF) == f[5] and f[2] == addr:
                    return s16((f[3] << 8) | f[4])
                buf = buf[i + 1:]
        raise TimeoutError("读寄存器无回应：检查串口号/波特率/TX-RX 是否接反/共地")

    def tele(self):
        """读一帧遥测，返回 dict"""
        buf = b""; t0 = time.time()
        while time.time() - t0 < 2:
            buf += self.s.read(64)
            i = buf.find(b"\xa5\x5a")
            while i >= 0 and len(buf) >= i + 19:
                f = buf[i:i + 19]
                if (sum(f[2:18]) & 0xFF) == f[18]:
                    w = struct.unpack(">8h", f[2:18])
                    return dict(roll=w[0] / 128, pitch=w[1] / 128, yaw=w[2] / 128, dl=w[3] & 0xFFFF, dc=w[4] & 0xFFFF,
                                dr=w[5] & 0xFFFF, vl=w[6], vr=w[7])
                i = buf.find(b"\xa5\x5a", i + 1)
        return None

def main():
    if len(sys.argv) < 3: print(__doc__); return
    b = Bot(sys.argv[1]); cmd = sys.argv[2]; a = sys.argv[3:]
    num = lambda x: int(x, 0)
    if cmd == "mon":
        yaw0 = None
        while True:
            t = b.tele()
            if not t: print("没有遥测数据（FPGA 是否下载了程序/串口是否正确）"); continue
            print("roll %7.2f  pitch %7.2f  yaw(回绕) %7.2f | 距离 L%5d C%5d R%5d mm | 轮速 L%4d R%4d" %
                  (t["roll"], t["pitch"], t["yaw"], t["dl"], t["dc"], t["dr"], t["vl"], t["vr"]))
    elif cmd == "rd":  print(hex(num(a[0])), "=", b.rd(num(a[0])))
    elif cmd == "wr":  b.wr(num(a[0]), num(a[1]))
    elif cmd == "zero": b.wr(R["CTRL"], 1)
    elif cmd == "stop": b.wr(R["MODE"], 0)
    elif cmd == "drive":
        b.wr(R["V"], num(a[0])); b.wr(R["W"], num(a[1])); b.wr(R["MODE"], 1)
        print("已发送。持续发送以喂看门狗（Ctrl-C 停车）")
        try:
            while True: time.sleep(0.5); b.wr(R["V"], num(a[0]))
        except KeyboardInterrupt: b.wr(R["MODE"], 0)
    elif cmd == "rot":
        b.wr(R["ROT"], num(a[0])); b.wr(R["MODE"], 3); time.sleep(0.05); b.wr(R["CTRL"], 2)
    elif cmd == "straight":
        b.wr(R["V"], num(a[0])); b.wr(R["MODE"], 2); time.sleep(0.05); b.wr(R["CTRL"], 2)
        try:
            while True: time.sleep(0.5); b.wr(R["V"], num(a[0]))
        except KeyboardInterrupt: b.wr(R["MODE"], 0)
    elif cmd == "mission":
        legs = [tuple(map(int, x.split(","))) for x in a[0].split(";")]
        assert 1 <= len(legs) <= 8
        for i, (turn, dist) in enumerate(legs):
            base = 0x20 + 4 * i
            b.wr(base, turn); b.wr(base + 1, (dist >> 16) & 0xFFFF); b.wr(base + 2, dist & 0xFFFF)
        b.wr(R["NLEG"], len(legs))
        if len(a) > 1: b.wr(R["V"], num(a[1]))
        b.wr(R["MODE"], 4); time.sleep(0.05); b.wr(R["CTRL"], 1); time.sleep(0.05); b.wr(R["CTRL"], 2)
        print("任务已启动。用 mon 观察；状态寄存器 0x48 的 bit2=完成 bit3=超时")
    else: print(__doc__)

if __name__ == "__main__":
    main()

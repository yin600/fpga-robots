import argparse
import time

import cv2

from uart_link import FpgaLink
from detector import Detector, INPUT_SIZE, CLS_PERSON

KP_TURN = 0.5
KP_DIST = 0.4
TARGET_BOX_H = 220.0
MAX_V = 40.0
MAX_W = 40.0
STOP_MM = 250
LOST_TIMEOUT = 2.0


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', required=True)
    ap.add_argument('--model', required=True)
    ap.add_argument('--cam', type=int, default=0)
    args = ap.parse_args()

    link = FpgaLink(args.port)
    det = Detector(args.model)
    cap = cv2.VideoCapture(args.cam)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    last_seen = time.time()
    try:
        while True:
            if link.telemetry and link.telemetry['dc'] < STOP_MM:
                link.set_mode(0)
                time.sleep(0.02)
                continue
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.01)
                continue
            dets = det.detect(frame)
            persons = [d for d in dets if d['cls'] == CLS_PERSON]
            if persons:
                p = max(persons, key=lambda d: d['conf'])
                last_seen = time.time()
                azimuth = p['cx'] - INPUT_SIZE / 2.0
                w = -KP_TURN * azimuth
                v = KP_DIST * (TARGET_BOX_H - p['h'])
                v = clamp(v, -MAX_V, MAX_V)
                w = clamp(w, -MAX_W, MAX_W)
                link.set_velocity(int(v), int(w))
                link.set_mode(1)
            else:
                if time.time() - last_seen > LOST_TIMEOUT:
                    link.set_mode(0)
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        link.set_mode(0)
        det.release()
        cap.release()
        link.close()


if __name__ == '__main__':
    main()

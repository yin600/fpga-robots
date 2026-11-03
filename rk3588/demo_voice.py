import argparse
import time

from uart_link import FpgaLink
from voice import VoiceRecognizer

FORWARD_SPEED = 30
BACKWARD_SPEED = -30
ROTATE_DEG = 90


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', required=True)
    ap.add_argument('--vosk-model', required=True)
    args = ap.parse_args()

    link = FpgaLink(args.port)
    voice = VoiceRecognizer(args.vosk_model)
    voice.start()

    try:
        while True:
            cmd = voice.take()
            if cmd == 'forward':
                link.set_velocity(FORWARD_SPEED, 0)
                link.set_mode(2)
            elif cmd == 'backward':
                link.set_velocity(BACKWARD_SPEED, 0)
                link.set_mode(1)
            elif cmd == 'left':
                link.set_mode(0)
                time.sleep(0.05)
                link.set_rotate(ROTATE_DEG)
                link.set_mode(3)
            elif cmd == 'right':
                link.set_mode(0)
                time.sleep(0.05)
                link.set_rotate(-ROTATE_DEG)
                link.set_mode(3)
            elif cmd == 'stop':
                link.set_mode(0)
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        link.set_mode(0)
        voice.stop()
        link.close()


if __name__ == '__main__':
    main()

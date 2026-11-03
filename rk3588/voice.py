import json
import threading

import sounddevice as sd

try:
    from vosk import Model, KaldiRecognizer
except ImportError:
    Model = None
    KaldiRecognizer = None

_SAMPLE_RATE = 16000

_COMMANDS = {
    'forward': ['往前走', '前进', '向前', '走'],
    'backward': ['后退', '倒车', '向后', '退'],
    'left': ['左转', '向左', '左边', '左'],
    'right': ['右转', '向右', '右边', '右'],
    'stop': ['停', '停下', '停止'],
}


def match_command(text):
    for cmd, words in _COMMANDS.items():
        for w in words:
            if w in text:
                return cmd
    return None


class VoiceRecognizer:
    def __init__(self, model_path, device=None):
        if Model is None:
            raise RuntimeError('vosk not installed')
        self.model = Model(model_path)
        self.rec = KaldiRecognizer(self.model, _SAMPLE_RATE)
        self.command = None
        self._alive = True
        self._stream = sd.RawInputStream(
            samplerate=_SAMPLE_RATE,
            blocksize=8000,
            device=device,
            dtype='int16',
            channels=1,
            callback=self._cb,
        )

    def _cb(self, indata, frames, time_info, status):
        if self.rec.AcceptWaveform(bytes(indata)):
            result = json.loads(self.rec.Result())
            text = result.get('text', '')
            cmd = match_command(text)
            if cmd:
                self.command = cmd

    def start(self):
        self._stream.start()

    def stop(self):
        self._alive = False
        try:
            self._stream.stop()
            self._stream.close()
        except Exception:
            pass

    def take(self):
        c = self.command
        self.command = None
        return c

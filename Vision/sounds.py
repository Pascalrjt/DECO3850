"""
Sound effects for the leaderboard runs, played on the computer running the tracker.

The sounds are generated here from simple tones (no audio files to ship) and
written once to a temporary folder. Playback uses the system's own player, so
it never blocks the tracker.

  start    three bouncy notes one second apart, then a chord on "go"
  goal     a three-note fanfare into a held chord
  last10   a bouncing groove for the final ten seconds, speeding up at the end
  timesup  two falling notes and a gong
"""

import shutil
import subprocess
import sys
import tempfile
import wave
from pathlib import Path

import numpy as np

RATE = 44100
SOUND_DIR = Path(tempfile.gettempdir()) / "sensory_drift_sounds_v2"
LAST_SECONDS = 10                 # the suspense groove starts this long before time runs out


def _note(semitones: float) -> float:
    """Frequency of a note, counted in semitones from A4 (440 Hz)."""
    return 440.0 * 2 ** (semitones / 12)


def _fade_in(y, seconds: float = 0.003):
    n = int(RATE * seconds)
    y[:n] *= np.linspace(0, 1, n)
    return y


def _pluck(freq, dur=0.35, vol=0.5, decay=9.0):
    """Bouncy mallet-style note: a fundamental plus a bright overtone, fading fast."""
    t = np.arange(int(RATE * dur)) / RATE
    y = (np.sin(2 * np.pi * freq * t)
         + 0.45 * np.sin(2 * np.pi * 4 * freq * t) * np.exp(-t * 22)
         + 0.2 * np.sin(2 * np.pi * 2 * freq * t)) * np.exp(-t * decay)
    return _fade_in(y) * vol


def _bass(freq, dur=0.22, vol=0.55):
    t = np.arange(int(RATE * dur)) / RATE
    y = (np.sin(2 * np.pi * freq * t) + 0.3 * np.sin(2 * np.pi * 2 * freq * t)
         + 0.12 * np.sin(2 * np.pi * 3 * freq * t)) * np.exp(-t * 7)
    release = int(RATE * 0.03)
    y[-release:] *= np.linspace(1, 0, release)
    return _fade_in(y, 0.004) * vol


def _stab(freqs, dur=0.8, vol=0.5, decay=3.0):
    t = np.arange(int(RATE * dur)) / RATE
    y = sum(np.sin(2 * np.pi * f * t) + 0.4 * np.sin(2 * np.pi * 2 * f * t)
            + 0.2 * np.sin(2 * np.pi * 3 * f * t) for f in freqs) / len(freqs) * np.exp(-t * decay)
    return _fade_in(y, 0.004) * vol


def _gong(dur=2.2, vol=0.7, base=110.0):
    """Struck-metal sound: partials that are not whole multiples, each fading at its own rate."""
    t = np.arange(int(RATE * dur)) / RATE
    partials = [(1.0, 1.0, 1.6), (2.76, 0.6, 2.4), (5.4, 0.35, 3.6), (8.93, 0.2, 5.0), (1.51, 0.5, 2.0)]
    y = sum(a * np.sin(2 * np.pi * base * m * t) * np.exp(-t * d) for m, a, d in partials)
    return _fade_in(y, 0.004) / 2.0 * vol


def _tick(vol=0.25):
    t = np.arange(int(RATE * 0.03)) / RATE
    return np.sin(2 * np.pi * 2200 * t) * np.exp(-t * 160) * vol


def _mix(total_seconds, layers):
    y = np.zeros(int(RATE * total_seconds))
    for start, layer in layers:
        i = int(RATE * start)
        n = min(len(layer), len(y) - i)
        if n > 0:
            y[i:i + n] += layer[:n]
    return y


def _start():
    layers = []
    for i in range(3):
        layers += [(i * 1.0, _pluck(_note(3), 0.45, 0.55)), (i * 1.0, _bass(_note(-21), 0.25)),
                   (i * 1.0 + 0.5, _bass(_note(-9), 0.14, 0.3))]
    layers += [(3.0, _stab((_note(3), _note(7), _note(10), _note(15)), 1.1, 0.6, 2.2)),
               (3.0, _bass(_note(-21), 0.5, 0.7))]
    layers += [(3.0 + k * 0.07, _pluck(_note(15 + s), 0.3, 0.3, 12)) for k, s in enumerate((0, 4, 7, 12))]
    return _mix(4.4, layers)


def _goal():
    layers, t = [], 0.0
    for freq in (_note(3), _note(7), _note(10)):
        layers += [(t, _stab((freq, freq * 2), 0.3, 0.55, 6.0)), (t, _bass(freq / 4, 0.2, 0.5))]
        t += 0.17
    layers += [(t, _stab((_note(15), _note(19), _note(22), _note(27)), 1.9, 0.7, 1.5)),
               (t, _bass(_note(-21), 0.9, 0.75)), (t, _bass(_note(-9), 0.9, 0.4))]
    layers += [(t + 0.12 + k * 0.08, _pluck(_note(27 + s), 0.5, 0.26, 7)) for k, s in enumerate((0, 4, 7, 12, 16))]
    return _mix(t + 2.1, layers)


def _timesup():
    return _mix(2.8, [(0.0, _pluck(_note(3), 0.3, 0.5, 8)), (0.18, _pluck(_note(-2), 0.35, 0.5, 8)),
                      (0.4, _gong())])


def _last10():
    layers = []
    for i in range(LAST_SECONDS):                 # i = 0 is "10 seconds left"
        left, t0 = LAST_SECONDS - i, float(i)
        key = i // 2                              # climbs a semitone every two seconds...
        if left <= 3:
            key = 4 + (4 - left)                  # ...then every second at the end
        root, top = _note(-21 + key), _note(3 + key)
        layers += [(t0, _bass(root, 0.22, 0.6)), (t0 + 0.5, _bass(root * 1.5, 0.16, 0.4))]
        layers += [(t0 + 0.25, _pluck(top, 0.2, 0.32, 14)), (t0 + 0.75, _pluck(top * 1.5, 0.2, 0.26, 14))]
        layers += [(t0, _tick(0.3)), (t0 + 0.5, _tick(0.16))]
        if left <= 5:
            layers += [(t0 + 0.125 + k * 0.25, _tick(0.14)) for k in range(4)]
        if left <= 3:                             # double time
            layers += [(t0 + k * 0.125, _pluck(top * (2 if k % 2 else 1), 0.12, 0.22, 20)) for k in range(8)]
            layers += [(t0 + 0.25, _bass(root, 0.12, 0.45)), (t0 + 0.75, _bass(root * 1.5, 0.12, 0.4))]
    return _mix(float(LAST_SECONDS), layers)


SOUNDS = {"start": _start, "goal": _goal, "timesup": _timesup, "last10": _last10}


def _write(path: Path, samples) -> None:
    samples = samples / max(1.0, float(np.max(np.abs(samples))) / 0.95)
    with wave.open(str(path), "wb") as out:
        out.setnchannels(1)
        out.setsampwidth(2)
        out.setframerate(RATE)
        out.writeframes((samples * 32767).astype("<i2").tobytes())


def _player_command() -> list[str] | None:
    """The system's command-line audio player, if there is one."""
    if sys.platform == "darwin" and shutil.which("afplay"):
        return ["afplay"]
    for name in ("paplay", "aplay"):
        if shutil.which(name):
            return [name, "-q"] if name == "aplay" else [name]
    return None


class SoundPlayer:
    """Plays the run sounds without blocking. Silent if there is no player or sound is off."""

    def __init__(self, enabled: bool = True):
        self.enabled = enabled
        self.command = _player_command()
        self.files: dict[str, Path] = {}
        self.process: subprocess.Popen | None = None
        self.current: str | None = None
        if enabled and self.command is None:
            print("No audio player found on this computer. Run sounds are off.")
        if enabled and self.command is not None:
            self._prepare()

    def _prepare(self) -> None:
        try:
            SOUND_DIR.mkdir(parents=True, exist_ok=True)
            for name, build in SOUNDS.items():
                path = SOUND_DIR / f"{name}.wav"
                if not path.exists():
                    _write(path, build())
                self.files[name] = path
        except OSError as exc:
            print(f"Could not prepare the run sounds: {exc}")
            self.files = {}

    @property
    def available(self) -> bool:
        return self.command is not None and bool(self.files)

    def play(self, name: str) -> None:
        """Start a sound, replacing whatever is playing."""
        self.stop()
        if not (self.enabled and self.available) or name not in self.files:
            return
        try:
            self.process = subprocess.Popen(self.command + [str(self.files[name])],
                                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.current = name
        except OSError:
            self.process = None

    def stop(self) -> None:
        if self.process is not None and self.process.poll() is None:
            self.process.terminate()
        self.process = None
        self.current = None

    def toggle(self) -> bool:
        self.enabled = not self.enabled
        if not self.enabled:
            self.stop()
        elif self.command is not None and not self.files:
            self._prepare()
        return self.enabled

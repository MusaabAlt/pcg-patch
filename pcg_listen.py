#!/usr/bin/env python3
"""
pcg_listen.py — listen to the mic LIVE through your headphones.

    pip install pyserial numpy scipy sounddevice

    ESP8266:  python pcg_listen.py --port COM5 --wire16
    ESP32:    python pcg_listen.py --port COM5

USE HEADPHONES. Heart sounds live at 20-150 Hz. Laptop speakers physically
cannot reproduce that band — you will hear nothing and think the rig is broken.
Any over-ear headphones or half-decent earbuds will do.

Also: never point the mic at a speaker that is playing this. Feedback.

Controls: Ctrl+C to stop.
"""

import argparse, queue, sys, threading, time
import numpy as np
import serial
import sounddevice as sd
from scipy import signal

MAGIC = b"\xA5\x5A\xA5\x5A"
FULL  = 2 ** 23


# ------------------------------------------------------------------ reader
def reader(ser, q, width, dtype, scale, stop, stats):
    window = b""
    exp_seq = None
    scanned, warned = 0, False
    while not stop.is_set():
        while MAGIC not in window:
            chunk = ser.read(512)
            if not chunk:
                print("\n  no data at all — wrong port, or the board is not running",
                      file=sys.stderr)
                stop.set(); return
            scanned += len(chunk)
            # Bytes ARE arriving but none of them are the sync word. Silently
            # spinning here is what makes the meter sit at -120 dBFS forever.
            if not warned and scanned > 40000:
                warned = True
                preview = chunk.decode("ascii", "replace")[:200]
                print("\n  *** receiving data, but no sync word found ***",
                      file=sys.stderr)
                print("  The board is sending something that is not the binary stream.",
                      file=sys.stderr)
                if sum(32 <= b < 127 or b in (10, 13) for b in chunk) > 0.8 * len(chunk):
                    print("  It looks like TEXT, so STREAM is still 0 in the sketch.",
                          file=sys.stderr)
                    print("  Set STREAM to 1, re-upload, close the Serial Monitor.",
                          file=sys.stderr)
                else:
                    print("  It is not text either — check the baud rate is 921600.",
                          file=sys.stderr)
                print(f"  first bytes received: {preview!r}\n", file=sys.stderr)
                stop.set(); return
            window = (window + chunk)[-8192:]
        scanned = 0
        window = window[window.index(MAGIC) + 4:]

        while len(window) < 4:
            window += ser.read(4 - len(window))
        seq = int.from_bytes(window[0:2], "little")
        cnt = int.from_bytes(window[2:4], "little")
        window = window[4:]
        if exp_seq is not None and seq != exp_seq:
            stats["dropped"] += (seq - exp_seq) % 65536
        exp_seq = (seq + 1) % 65536
        n = cnt * width
        while len(window) < n:
            window += ser.read(n - len(window))
        blk = np.frombuffer(window[:n], dtype=dtype).astype(np.float64) * scale
        window = window[n:]
        q.put(blk / FULL)


# ------------------------------------------------------------------ beats
class BeatCounter:
    """Heart rate from envelope autocorrelation over a rolling window.

    Peak-picking does not work here: S1 and S2 are two peaks per beat, so a
    naive picker reports roughly double the true rate. Autocorrelation finds
    the beat-to-beat period directly and ignores the internal S1-S2 spacing."""

    ENV_FS  = 100      # envelope rate after decimation, Hz
    WINDOW  = 8        # seconds of history
    LAG_LO  = 0.40     # 150 bpm
    LAG_HI  = 1.60     # 37 bpm

    def __init__(self, fs):
        self.fs = fs
        self.step = max(1, fs // self.ENV_FS)
        self.env_sos = signal.butter(2, 15, "low", fs=fs, output="sos")
        self.zi = signal.sosfilt_zi(self.env_sos)
        self.hist = np.zeros(0)
        self.since = 0
        self.n = 0          # global input-sample counter, keeps decimation phase

    def push(self, x):
        env, self.zi = signal.sosfilt(self.env_sos, np.abs(x), zi=self.zi)
        # Decimate with a phase that carries across blocks. Blocks are shorter
        # than the stride, so a plain env[::step] would emit one sample per
        # block and silently set the wrong envelope rate.
        first = (-self.n) % self.step
        self.hist = np.concatenate([self.hist, env[first::self.step]])
        self.hist = self.hist[-self.ENV_FS * self.WINDOW:]
        self.n += len(env)
        self.since += len(x)

        # recompute about twice a second, not every block
        if self.since < self.fs // 2 or len(self.hist) < self.ENV_FS * 4:
            return None
        self.since = 0

        e = self.hist - self.hist.mean()
        ac = signal.correlate(e, e, mode="full")[len(e) - 1:]
        lo, hi = int(self.LAG_LO * self.ENV_FS), int(self.LAG_HI * self.ENV_FS)
        if hi >= len(ac):
            return None
        lag = lo + int(np.argmax(ac[lo:hi]))
        # reject when there is no real periodicity: peak must stand above the mean
        if ac[lag] < 0.25 * ac[0]:
            return None
        return 60.0 * self.ENV_FS / lag


# ------------------------------------------------------------------ main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--fs-in", dest="fs", type=int, default=16000)
    ap.add_argument("--wire16", action="store_true", help="ESP8266: int16 samples")
    ap.add_argument("--gain", type=float, default=300.0, help="playback gain")
    ap.add_argument("--band", nargs=2, type=float, default=[20, 400])
    ap.add_argument("--device", default=None,
                    help="output device INDEX (a number) or part of its name")
    ap.add_argument("--list-devices", action="store_true")
    a = ap.parse_args()

    if a.list_devices:
        print(sd.query_devices()); return

    # "--device 3" arrives as the string "3". Passed on as a string it becomes a
    # NAME search, which matches every device with a 3 in it and then errors.
    dev = a.device
    if isinstance(dev, str) and dev.strip().lstrip("-").isdigit():
        dev = int(dev)

    try:
        info = sd.query_devices(dev, "output") if dev is not None \
               else sd.query_devices(sd.default.device[1], "output")
        name = info["name"]
        print(f"output device: {name}")
        low = name.lower()
        if "hands-free" in low or "headset" in low or "hfp" in low:
            print("  *** WARNING: this is the Bluetooth HANDS-FREE profile.")
            print("      It is 8 kHz telephony, 300-3400 Hz. Heart sounds live at")
            print("      20-150 Hz and will be removed entirely before you hear")
            print("      anything. USE WIRED HEADPHONES.")
    except Exception as e:
        print(f"could not query device {dev!r}: {e}")
        print("run with --list-devices and pass the number in square brackets")
        return

    width, dtype, scale = (2, "<i2", 256) if a.wire16 else (4, "<i4", 1)
    fs = a.fs

    ser = serial.Serial(a.port, a.baud, timeout=2)
    time.sleep(2.0)
    ser.reset_input_buffer()

    q, stop = queue.Queue(maxsize=200), threading.Event()
    stats = {"dropped": 0}
    threading.Thread(target=reader,
                     args=(ser, q, width, dtype, scale, stop, stats),
                     daemon=True).start()

    sos = signal.butter(4, a.band, "bandpass", fs=fs, output="sos")
    zi = signal.sosfilt_zi(sos)
    beats = BeatCounter(fs)
    tail = np.zeros(0)
    meter = {"rms": 0.0, "clip": False, "bpm": None}

    def callback(outdata, frames, t, status):
        nonlocal zi, tail
        while len(tail) < frames:
            try:
                blk = q.get(timeout=0.5)
            except queue.Empty:
                outdata[:] = 0; return
            y, zi = signal.sosfilt(sos, blk, zi=zi)
            r = beats.push(y)
            if r:
                meter["bpm"] = r
            meter["rms"] = float(np.sqrt(np.mean(y * y)) + 1e-12)
            tail = np.concatenate([tail, y])
        chunk, tail = tail[:frames], tail[frames:]
        out = chunk * a.gain
        meter["clip"] = bool(np.abs(out).max() > 1.0)
        outdata[:, 0] = np.clip(out, -1.0, 1.0)

    print(f"listening on {a.port} at {fs} Hz, band {a.band[0]:.0f}-{a.band[1]:.0f} Hz")
    print("HEADPHONES ON. Press the mic to your chest, 4th rib space, left of the sternum.")
    print("Ctrl+C to stop.\n")

    try:
        with sd.OutputStream(samplerate=fs, channels=1, dtype="float32",
                             blocksize=512, device=dev, callback=callback):
            silent = 0
            while not stop.is_set():
                time.sleep(0.25)
                if meter["rms"] <= 0:
                    silent += 1
                    if silent == 8:      # 2 s with nothing decoded
                        print("\r  no blocks decoded yet — see the message above"
                              " if one appeared", end="", flush=True)
                    continue
                silent = 0
                rms = meter["rms"]
                db = 20 * np.log10(rms) if rms > 1e-12 else -120.0
                n = max(0, min(30, int((db + 90) / 3)))
                bpm = f"{meter['bpm']:5.0f} bpm" if meter["bpm"] else "   -- bpm"
                flag = "  CLIPPING - lower --gain" if meter["clip"] else ""
                drop = f"  dropped {stats['dropped']}" if stats["dropped"] else ""
                print(f"\r{db:7.1f} dBFS |{'#'*n:<30}| {bpm}{drop}{flag}   ",
                      end="", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set(); ser.close(); print("\nstopped")


if __name__ == "__main__":
    main()
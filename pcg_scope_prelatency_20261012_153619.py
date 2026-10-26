#!/usr/bin/env python3
"""
pcg_scope.py — live scope for the PCG rig. Same serial input, same audio path
and the same flags as pcg_listen.py, plus a window with three stacked panels
and a status line. pcg_listen.py is untouched and remains the fallback.

    python pcg_scope.py --port COM15 --wire16 --fs-in 32000 --gain 100
    python pcg_scope.py --port COM15 --wire16 --fs-in 32000 --gain 100 \
                        --mode lung --hp 80

  Panel 1  RAW, before the band-pass. Scrolling 5 s window.
  Panel 2  FILTERED, after the band-pass. Same 5 s window, same sample index,
           so what the filter removed is visible by comparing the two.
  Panel 3  Live spectrum of the RAW signal: Welch over the last 0.5 s, plus a
           slow-decaying peak-hold trace, with the bands of interest shaded in
           the same colours as analog_mic_record.py.

MODES. The same microphone is used for heart and lung; only the placement on
the body changes. HEART and LUNG buttons at the bottom of the window (or the H
and L keys) switch the band-pass, the spectrum x-axis, the shaded bands and the
rate readout together, in one click. Every number either mode uses lives in the
PRESETS dict below and nowhere else. The startup mode is --mode heart|lung.

The mode is NOT cosmetic: a recording made in the wrong mode is a recording of
the wrong band, so the active mode is always the first field of the status line
and is printed to the console on every switch.

The LUNG band follows the CORSA / European Respiratory Society recording
standard: high-pass at 60 Hz with a slope steeper than 18 dB/octave (the
4th-order Butterworth here is 24 dB/octave), low-pass at 2000 Hz, which covers
crackles and wheezes. The high-pass corner is the one number that is genuinely
contested, so it is a flag rather than a constant: --hp 60 is the CORSA value,
--hp 100 the value ICBHI pipelines use because heart sound reaches to 150 Hz,
and --hp 80 (the default) is what most published pipelines use. Raising it
removes more heart bleed and loses more low breath sound. Tune it by ear and by
measurement; do not average the published values.

THE Y AXIS DOES NOT AUTO-SCALE. A per-frame autoscaling axis is a visual AGC:
it rewrites apparent loudness frame by frame and hides the S1:S2 amplitude
ratio, which is the diagnostic information in this project. The waveform
range is chosen ONCE from the first half second of data, printed, and then
frozen. UP arrow halves the range (zoom in), DOWN doubles it; every change is
printed. Both waveform panels always share one range. There is no autoscale
option.

DISPLAY DECIMATION IS FOR PIXELS ONLY. 5 s at 32 kHz is 160k samples; each
panel is rendered from a min/max envelope of about 2000 bins so that narrow
transients such as S1 cannot vanish between kept samples. This happens in the
UI thread on a copy of the data. It never touches the audio path, the filter
state, or anything that could be recorded.

THREADS. Serial reader thread -> audio queue -> sounddevice callback thread
(band-pass + headphones, exactly as pcg_listen.py). The callback hands each
raw/filtered block pair to the UI with a non-blocking put; if the UI stalls,
the UI queue fills and display blocks are dropped (counted), but the audio
never waits on the UI. matplotlib runs on the main thread with blitting.

Controls: H = heart mode, L = lung mode, UP / DOWN = zoom y,
          q or window close = quit, Ctrl+C in console.
"""

import argparse, ctypes, math, queue, sys, threading, time
from collections import deque
import numpy as np
import serial
import sounddevice as sd
from scipy import signal
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.transforms import Bbox
from matplotlib.widgets import Button

# matplotlib binds 'h' to home-view and 'l' to log-y by default, so without
# this the mode keys would also reset the pan/zoom or turn the waveform axis
# logarithmic. Take those two letters back; 'r'/'home' and 'k' still work.
for _km in ("keymap.home", "keymap.yscale", "keymap.xscale"):
    matplotlib.rcParams[_km] = [k for k in matplotlib.rcParams[_km]
                                if k.lower() not in ("h", "l")]

MAGIC    = b"\xA5\x5A\xA5\x5A"
FULL     = 2 ** 23
WINDOW_S = 5.0        # waveform panels, seconds
SPEC_S   = 0.5        # Welch window, seconds
PX       = 2000       # envelope bins per waveform panel
SPEC_YLIM = (-120.0, -10.0)   # dB re FS^2/Hz, fixed
PEAK_DECAY_DB_S = 3.0         # peak-hold decay
TARGET_FPS = 30.0
COL_S1S2   = "#004AAD"        # same shading colours as analog_mic_record.py
COL_MURMUR = "#5A6480"
COL_BREATH = "#0F7B6C"
COL_CRACKLE = "#C08A2E"

# --------------------------------------------------------------------- PRESETS
# Every number that differs between the two modes lives here. Nothing below
# hard-codes a band, an axis limit or a shading edge; to retune LUNG after a
# session with a real probe, edit this dict and nothing else.
#
#   band   band-pass passed to butter(4, ..., "bandpass"), Hz
#   fmax   spectrum x-axis upper limit, Hz
#   shade  shaded spectrum regions: (lo, hi, colour, legend label)
#   rate   which rate engine drives the readout: "bpm" or "brpm"
#   hold   seconds a displayed rate survives with no fresh confirmation before
#          it reverts to "--". The engines recompute periodically and return
#          nothing in between, so a displayed value has to outlive the gaps —
#          but only just. Without this the last good number stays on screen for
#          ever: on the rig a stale 23.4 breaths/min sat there for 80 s, right
#          through a 10.8 % railing episode, long after it meant anything.
#   yr     fixed waveform y half-range in FS, or None to keep the current one.
#          A NUMBER here is still a FIXED range: it is set once on the switch
#          and printed, and nothing adapts it afterwards. It is None in both
#          presets so that a mode switch never moves the y-axis at all.
#
# The shaded regions are drawn on the RAW spectrum, not the filtered one, so a
# shaded band that lies outside the pass-band is intentional: it shows what the
# band-pass is rejecting. HEART shades the murmur region at 200-600 Hz while
# passing only 20-150 Hz, so murmur energy stays visible on the spectrum even
# though it is not in what you hear.
PRESETS = {
    "heart": {
        "label": "HEART",
        "band":  (20.0, 150.0),
        "fmax":  1000.0,
        "shade": [(20.0, 150.0, COL_S1S2, "S1/S2 20-150 Hz"),
                  (200.0, 600.0, COL_MURMUR, "murmur 200-600 Hz")],
        "rate":  "bpm",
        "hold":  4.0,     # BeatCounter recomputes about twice a second
        "yr":    None,
    },
    "lung": {
        # band[0] is the CORSA/ERS high-pass corner and is the DEFAULT only —
        # --hp 60|80|100 replaces it in main(). band[1] is the CORSA low-pass
        # and is not adjustable: 2000 Hz is where crackles and wheezes end.
        "label": "LUNG",
        "band":  (80.0, 2000.0),
        "fmax":  2500.0,
        "shade": [(100.0, 1000.0, COL_BREATH, "breath sounds 100-1000 Hz"),
                  (1000.0, 2000.0, COL_CRACKLE, "crackles/wheeze 1000-2000 Hz")],
        "rate":  "brpm",
        "hold":  8.0,     # BreathCounter recomputes every 2 s
        "yr":    None,
    },
}
BTN_ON, BTN_OFF = "#004AAD", "#E4E7EF"


# ------------------------------------------------------------------ reader
def reader(ser, q, width, dtype, scale, stop, stats):
    """Copied from pcg_listen.py, plus wire-rate bookkeeping in stats."""
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
        stats["samples"] += len(blk)
        stats["hist"].append((time.perf_counter(), stats["samples"]))   # rolling rate
        q.put(blk / FULL)


def wire_rate(stats, span=5.0):
    """Samples per second over roughly the last `span` seconds, from reader timestamps."""
    h = stats["hist"]
    if len(h) < 2:
        return None
    t1, n1 = h[-1]
    t0, n0 = h[0]
    for t, n in h:
        if t1 - t <= span:
            t0, n0 = t, n
            break
    return (n1 - n0) / (t1 - t0) if t1 > t0 else None


# ------------------------------------------------------------------ beats
class BeatCounter:
    """Heart rate from envelope autocorrelation.

    Was copied from pcg_listen.py; it now differs from it in one way, and
    pcg_listen.py still has the original. argmax over a FIXED lag range always
    returns something, and with no periodicity what it returns is a boundary:
    lag == lo reads out as exactly 60/0.40 = 150.0 bpm. Measured on this rig
    with no chest on the mic, it printed 150 bpm twice in 70 s. A confident
    wrong number is worse than "--".

    So the same guard BreathCounter uses: SEARCH wider than we REPORT, and
    discard a peak that lands on the search's own limit. A true 150 or 37.5 can
    then still stand as an interior maximum, while a peak that is really just
    the edge of the window falls in the margin and is thrown away."""

    ENV_FS  = 100
    WINDOW  = 8
    LAG_LO  = 0.40     # 150 bpm, the fastest rate REPORTED
    LAG_HI  = 1.60     # 37.5 bpm, the slowest rate REPORTED
    MARGIN  = 1.25     # search this much wider than the reported range
    PEAK    = 0.25     # ac[lag] must reach this fraction of ac[0]

    def __init__(self, fs):
        self.fs = fs
        self.step = max(1, fs // self.ENV_FS)
        self.env_sos = signal.butter(2, 15, "low", fs=fs, output="sos")
        self.zi = signal.sosfilt_zi(self.env_sos)
        self.hist = np.zeros(0)
        self.since = 0
        self.n = 0

    def push(self, x):
        env, self.zi = signal.sosfilt(self.env_sos, np.abs(x), zi=self.zi)
        first = (-self.n) % self.step
        self.hist = np.concatenate([self.hist, env[first::self.step]])
        self.hist = self.hist[-self.ENV_FS * self.WINDOW:]
        self.n += len(env)
        self.since += len(x)
        if self.since < self.fs // 2 or len(self.hist) < self.ENV_FS * 4:
            return None
        self.since = 0
        e = self.hist - self.hist.mean()
        ac = signal.correlate(e, e, mode="full")[len(e) - 1:]
        lo, hi = int(self.LAG_LO * self.ENV_FS), int(self.LAG_HI * self.ENV_FS)
        slo = max(1, int(lo / self.MARGIN))
        shi = int(hi * self.MARGIN)
        if shi + 1 >= len(ac) or ac[0] <= 0:
            return None
        lag = slo + int(np.argmax(ac[slo:shi + 1]))
        if lag <= slo or lag >= shi:                     # the search's own limit
            return None
        if not (ac[lag] > ac[lag - 1] and ac[lag] > ac[lag + 1]):
            return None                                  # a slope, not a peak
        if lag < lo or lag > hi:                          # real, but out of range
            return None
        if ac[lag] < self.PEAK * ac[0]:
            return None                                  # no real periodicity
        return 60.0 * self.ENV_FS / lag


class BreathCounter:
    """Breaths per minute from the band envelope. Same autocorrelation idea as
    BeatCounter, two decades slower, and stricter about admitting defeat.

    TWO filter stages, not one. A single 2 Hz low-pass at 32 kHz sits at
    1.25e-4 of Nyquist, where the poles crowd the unit circle. Low-passing at
    15 Hz, decimating to 100 Hz and only then low-passing at 2 Hz keeps every
    filter in a comfortable numerical range for the same end result.

    BOTH zi ARE SCALED TO THE FIRST SAMPLE, for the same reason the band-pass
    is on a mode switch. sosfilt_zi returns the steady state for a UNIT input,
    but an envelope is of order 0.02 FS, so an unscaled zi starts the chain
    ~50x too high. That transient is short in time but it sits in the 40 s
    history until it rolls out, and while it is there it dominates the mean and
    ac[0] — measured: the first valid reading did not appear until 42 s, and
    once the spike left, near-zero residue autocorrelated to a confident-looking
    29.4 breaths/min on a DC input. Scaling zi removes both symptoms at once.

    REJECTION. argmax over a fixed lag range always returns something, and when
    there is no periodicity what it returns is a boundary — which reads out as
    a confident 6.0 or 30.0 breaths/min. The heart rate had exactly this bug.
    A lag is only accepted if the envelope actually swings (a constant envelope
    is not breathing, whatever its autocorrelation says), and the lag is
    strictly inside the search range, is a genuine local maximum, and stands
    well above zero lag. Otherwise: nothing, and the readout stays "--"."""

    ENV_FS = 100          # stage-1 envelope rate, Hz
    SUB    = 4            # stage-2 decimation -> 25 Hz for the autocorrelation
    AC_FS  = ENV_FS // SUB
    LP2_HZ = 2.0          # stage-2 low-pass, Hz
    WINDOW = 40.0         # seconds of history kept
    MIN_S  = 20.0         # no number at all until this much history exists
    LAG_LO = 2.0          # 30 breaths/min, the fastest rate REPORTED
    LAG_HI = 10.0         # 6 breaths/min, the slowest rate REPORTED
    MARGIN = 1.25         # search this much wider than the reported range
    PEAK   = 0.30         # ac[lag] must reach this fraction of ac[0]
    MIN_MOD = 0.05        # envelope std/mean floor: below this it is not breathing
    EVERY  = 2.0          # recompute this often, seconds

    def __init__(self, fs):
        self.fs = fs
        self.step = max(1, fs // self.ENV_FS)
        self.sos1 = signal.butter(2, 15.0, "low", fs=fs, output="sos")
        self.sos2 = signal.butter(2, self.LP2_HZ, "low", fs=self.ENV_FS, output="sos")
        self.zi1 = None     # scaled to the first sample on the first push
        self.zi2 = None
        self.hist = np.zeros(0)
        self.n = 0          # stage-1 input samples, keeps decimation phase
        self.m = 0          # stage-2 input samples, same reason
        self.since = 0

    def push(self, x):
        ax = np.abs(x)
        if self.zi1 is None:
            self.zi1 = signal.sosfilt_zi(self.sos1) * (ax[0] if len(ax) else 0.0)
        env, self.zi1 = signal.sosfilt(self.sos1, ax, zi=self.zi1)
        first = (-self.n) % self.step
        self.n += len(env)
        e1 = env[first::self.step]
        if len(e1):
            if self.zi2 is None:
                self.zi2 = signal.sosfilt_zi(self.sos2) * e1[0]
            e2, self.zi2 = signal.sosfilt(self.sos2, e1, zi=self.zi2)
            f2 = (-self.m) % self.SUB
            self.m += len(e2)
            self.hist = np.concatenate([self.hist, e2[f2::self.SUB]])
            self.hist = self.hist[-int(self.AC_FS * self.WINDOW):]
        self.since += len(x)

        if self.since < self.fs * self.EVERY or len(self.hist) < self.AC_FS * self.MIN_S:
            return None
        self.since = 0

        # A flat envelope is silence, DC or a steady tone, not breathing. Its
        # residue after mean removal is numerical noise, and the autocorrelation
        # of numerical noise will happily report a confident rate.
        mu = self.hist.mean()
        if mu <= 0 or self.hist.std() < self.MIN_MOD * mu:
            return None

        e = self.hist - self.hist.mean()
        ac = signal.correlate(e, e, mode="full")[len(e) - 1:]
        # SEARCH wider than we REPORT. Rejecting only lag == lo or lag == hi is
        # not enough: on the rig this reported 6.0 breaths/min from lag 249 of
        # 250, the edge in everything but the integer comparison. A guard band
        # inside the range would work but is asymmetric in rate — 10 lag
        # samples costs 5 breaths/min at the fast end and 0.25 at the slow end,
        # so it would have quietly capped the range at 25. Searching a wider
        # window instead lets a peak at a true 6 or 30 stand as an interior
        # maximum, while a peak that is really the search limit falls in the
        # margin and is discarded.
        lo, hi = int(self.LAG_LO * self.AC_FS), int(self.LAG_HI * self.AC_FS)
        slo = max(1, int(lo / self.MARGIN))
        shi = int(hi * self.MARGIN)
        if shi + 1 >= len(ac) or ac[0] <= 0:
            return None
        lag = slo + int(np.argmax(ac[slo:shi + 1]))
        if lag <= slo or lag >= shi:                     # the search's own limit
            return None
        if not (ac[lag] > ac[lag - 1] and ac[lag] > ac[lag + 1]):
            return None                                  # a slope, not a peak
        if lag < lo or lag > hi:                         # real, but out of range
            return None
        if ac[lag] < self.PEAK * ac[0]:
            return None                                  # no real periodicity
        return 60.0 * self.AC_FS / lag


# ------------------------------------------------------------------ display helpers
class Ring:
    """Fixed-length circular buffer, UI thread only."""
    def __init__(self, n):
        self.buf = np.zeros(n); self.n = n; self.wp = 0; self.filled = 0

    def push(self, x):
        k = len(x)
        if k >= self.n:
            self.buf[:] = x[-self.n:]; self.wp = 0; self.filled = self.n; return
        end = self.wp + k
        if end <= self.n:
            self.buf[self.wp:end] = x
        else:
            f = self.n - self.wp
            self.buf[self.wp:] = x[:f]; self.buf[:k - f] = x[f:]
        self.wp = end % self.n
        self.filled = min(self.n, self.filled + k)

    def ordered(self):
        return np.concatenate([self.buf[self.wp:], self.buf[:self.wp]])


def envelope(x, nbins):
    """Min/max envelope decimation for rendering: two points per bin."""
    n = len(x); b = max(1, n // nbins); m = n // b
    v = x[n - m * b:].reshape(m, b)
    y = np.empty(2 * m)
    y[0::2] = v.min(axis=1)
    y[1::2] = v.max(axis=1)
    return y


def snap_range(rms):
    """Power-of-two range at least 8x the RMS, within [1/512, 1] FS.

    RMS, not peak: the ADC noise on this rig is spiky, and a single spike in
    the first half second would snap the range to full scale and flatten the
    heart sounds into a line. Spikes above the range are simply clipped by the
    axes; the status line still reports the true peak and rail percentage."""
    if not np.isfinite(rms) or rms <= 0:
        return 1.0 / 64
    r = 2.0 ** math.ceil(math.log2(8.0 * rms))
    return float(min(1.0, max(1.0 / 512, r)))


# ------------------------------------------------------------------ main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--fs-in", dest="fs", type=int, default=16000)
    ap.add_argument("--wire16", action="store_true", help="int16 samples on the wire")
    ap.add_argument("--gain", type=float, default=300.0,
                    help="playback gain. NOT changed by the mode buttons")
    ap.add_argument("--mode", choices=("heart", "lung"), default="heart",
                    help="startup mode; H/L or the buttons switch it live")
    ap.add_argument("--hp", type=float, choices=(60.0, 80.0, 100.0), default=80.0,
                    help="LUNG high-pass corner, Hz. 60 = CORSA/ERS, 80 = most "
                         "published pipelines (default), 100 = ICBHI. Higher "
                         "rejects more heart bleed and loses more low breath sound")
    ap.add_argument("--band", nargs=2, type=float, default=None,
                    help="override the startup mode's band-pass, Hz")
    ap.add_argument("--device", default=None,
                    help="output device INDEX (a number) or part of its name")
    ap.add_argument("--list-devices", action="store_true")
    a = ap.parse_args()

    # --hp rewrites ONLY the low edge of the lung band, before any --band
    # override, so an explicit --band still wins. Applied to the dict rather
    # than carried separately: PRESETS stays the single place a band lives.
    PRESETS["lung"]["band"] = (float(a.hp), PRESETS["lung"]["band"][1])

    # An explicit --band replaces that ONE preset's band for this session, so
    # the H/L buttons keep working and the override survives switching away and
    # back. The other mode is untouched.
    if a.band is not None:
        PRESETS[a.mode]["band"] = (float(a.band[0]), float(a.band[1]))
        print(f"--band overrides the {a.mode} preset for this run: "
              f"{a.band[0]:.0f}-{a.band[1]:.0f} Hz")

    if a.list_devices:
        print(sd.query_devices()); return

    # "--device 3" arrives as the string "3": an INDEX, not a name search.
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
    rail = 0.999 if a.wire16 else (FULL - 1) / FULL   # |x| at or beyond the ADC rails

    ser = serial.Serial(a.port, a.baud, timeout=2)
    time.sleep(2.0)
    ser.reset_input_buffer()

    q, stop = queue.Queue(maxsize=200), threading.Event()
    q_ui = queue.Queue(maxsize=4096)          # ~32 s of 256-sample blocks at 32 kHz
    stats = {"dropped": 0, "samples": 0, "hist": deque(maxlen=4000)}
    astats = {"n": 0, "last": None, "maxgap": 0.0, "gap5": 0.0, "gapsw": 0.0,
              "flags": 0, "ui_drop": 0}
    threading.Thread(target=reader,
                     args=(ser, q, width, dtype, scale, stop, stats),
                     daemon=True).start()

    # sos and zi are ONE value, not two. A new sos with the old zi is a filter
    # with the wrong state and it rings; the lock exists so that pair is never
    # seen half-swapped. It is held for a few microseconds at a time and never
    # across sosfilt, so the UI thread can never stall the audio callback.
    sos0 = signal.butter(4, list(PRESETS[a.mode]["band"]), "bandpass",
                         fs=fs, output="sos")
    filt = {"sos": sos0, "zi": signal.sosfilt_zi(sos0), "last": 0.0}
    flock = threading.Lock()
    tail = np.zeros(0)
    meter = {"clip": False}

    # ---- audio callback: identical to pcg_listen.py except for the
    # non-blocking hand-off of (raw, filtered) to the UI queue, timing stats,
    # and reading the filter through the lock.
    def callback(outdata, frames, t, status):
        nonlocal tail
        now = time.perf_counter()
        if astats["last"] is not None:
            gap = now - astats["last"]
            if gap > astats["maxgap"]: astats["maxgap"] = gap
            if gap > astats["gap5"]:   astats["gap5"] = gap
            if gap > astats["gapsw"]:  astats["gapsw"] = gap
        astats["last"] = now
        astats["n"] += 1
        if status:
            astats["flags"] += 1
        while len(tail) < frames:
            try:
                blk = q.get(timeout=0.5)
            except queue.Empty:
                outdata[:] = 0; return
            filt["last"] = float(blk[-1])     # DC level the next zi is scaled to
            with flock:
                sos_now, zi_now = filt["sos"], filt["zi"]
            y, zi_now = signal.sosfilt(sos_now, blk, zi=zi_now)
            with flock:
                # If the UI swapped the filter while this block was running,
                # the zi just produced belongs to the retired sos. Drop it: the
                # pair the UI installed is already correct for the next block.
                if filt["sos"] is sos_now:
                    filt["zi"] = zi_now
            try:
                q_ui.put_nowait((blk, y))      # never blocks on the UI
            except queue.Full:
                astats["ui_drop"] += 1
            tail = np.concatenate([tail, y])
        chunk, tail = tail[:frames], tail[frames:]
        out = chunk * a.gain
        meter["clip"] = bool(np.abs(out).max() > 1.0)
        outdata[:, 0] = np.clip(out, -1.0, 1.0)

    # ---- UI state
    win = int(WINDOW_S * fs)
    raw_ring, filt_ring = Ring(win), Ring(win)
    mode = {"name": a.mode, "switches": 0, "report": False, "line": "", "t": 0.0}
    rmeter = {"engine": None, "val": None, "t": 0.0}
    nspec = int(SPEC_S * fs)
    nperseg = 2048 if fs >= 16000 else 1024
    fr_full = np.fft.rfftfreq(nperseg, 1.0 / fs)
    # keep / fr / hold are rebuilt on every mode switch, because fmax changes.
    spec = {"keep": None, "fr": None, "hold": None, "spans": []}

    # envelope x positions: bin centres, two points per bin, fixed
    nb = win // max(1, win // PX)
    b = win // nb
    xe = np.repeat((np.arange(nb) * b + b / 2.0) / fs - WINDOW_S, 2)

    fig, (ax0, ax1, ax2) = plt.subplots(3, 1, figsize=(12, 8.5))
    fig.canvas.manager.set_window_title("pcg_scope")
    fig.subplots_adjust(top=0.90, bottom=0.135, hspace=0.45)
    status_txt = fig.text(0.01, 0.975, "starting ...", family="monospace",
                          fontsize=9, animated=True)
    fig.text(0.99, 0.945, "H: heart   L: lung   UP/DOWN arrow: zoom y   q: quit",
             ha="right", fontsize=8, color="#5A6480")

    (l_raw,)  = ax0.plot(xe, np.zeros_like(xe), lw=0.5, color="#0C1745", animated=True)
    (l_filt,) = ax1.plot(xe, np.zeros_like(xe), lw=0.5, color=COL_S1S2, animated=True)
    (l_spec,) = ax2.plot([0.0], [SPEC_YLIM[0]], lw=0.8,
                         color="#0C1745", animated=True, label="Welch 0.5 s")
    (l_hold,) = ax2.plot([0.0], [SPEC_YLIM[0]], lw=0.8, color="#D85A30", animated=True,
                         label=f"peak hold, -{PEAK_DECAY_DB_S:.0f} dB/s")
    for ax in (ax0, ax1):
        ax.set_xlim(-WINDOW_S, 0)
        ax.set_xlabel("s", fontsize=8); ax.set_ylabel("FS", fontsize=8)
        ax.tick_params(labelsize=8)
    ax0.set_title("RAW — before filtering", fontsize=10, loc="left")
    ax2.set_ylim(*SPEC_YLIM)
    ax2.set_title("RAW spectrum — Welch, last 0.5 s (fixed dB axis)", fontsize=10, loc="left")
    ax2.set_xlabel("Hz", fontsize=8); ax2.set_ylabel("dB re FS²/Hz", fontsize=8)
    ax2.tick_params(labelsize=8)

    ui = {"yr": None, "stale": True, "bg": None, "draw_ms": 0.0}

    # ---- mode buttons. drawon=False and hovercolor==color keep matplotlib from
    # issuing its own draws behind the blit loop; every repaint goes through
    # redraw_background() so the saved background and the screen never diverge.
    ax_h = fig.add_axes([0.355, 0.022, 0.13, 0.052])
    ax_l = fig.add_axes([0.515, 0.022, 0.13, 0.052])
    btn = {"heart": Button(ax_h, "HEART  (H)"), "lung": Button(ax_l, "LUNG  (L)")}
    for b in btn.values():
        b.drawon = False
        b.label.set_fontsize(9)

    def paint_buttons():
        for key, b in btn.items():
            on = (key == mode["name"])
            b.color = b.hovercolor = BTN_ON if on else BTN_OFF
            b.ax.set_facecolor(b.color)
            b.label.set_color("white" if on else "#0C1745")
            b.label.set_fontweight("bold" if on else "normal")
            for s in b.ax.spines.values():
                s.set_linewidth(2.0 if on else 0.8)
                s.set_color("#0C1745" if on else "#9AA3B8")

    def apply_mode(name, first=False):
        """Switch every mode-dependent number at once. UI thread only."""
        t_sw = time.perf_counter()
        p = PRESETS[name]
        mode["name"] = name

        # 1. filter. Build BOTH halves, then publish them as one value. zi is
        #    scaled to the current sample so the band-pass starts already in
        #    steady state and there is no step transient into the headphones.
        new_sos = signal.butter(4, list(p["band"]), "bandpass", fs=fs, output="sos")
        new_zi = signal.sosfilt_zi(new_sos) * filt["last"]
        with flock:
            filt["sos"], filt["zi"] = new_sos, new_zi

        # 2. rate engine. Envelope history from the previous band describes a
        #    different signal, so it is discarded rather than mixed in.
        rmeter["engine"] = BeatCounter(fs) if p["rate"] == "bpm" else BreathCounter(fs)
        rmeter["val"], rmeter["t"] = None, time.perf_counter()

        # 3. fixed y-range, only if this preset names one. Never data-derived.
        if p["yr"] is not None:
            ui["yr"] = float(p["yr"])

        # 4. spectrum axis, shading and the filtered-panel title
        spec["keep"] = fr_full <= p["fmax"]
        spec["fr"] = fr_full[spec["keep"]]
        spec["hold"] = np.full(spec["fr"].shape, SPEC_YLIM[0])
        l_spec.set_data(spec["fr"], np.full(spec["fr"].shape, SPEC_YLIM[0]))
        l_hold.set_data(spec["fr"], spec["hold"])
        ax2.set_xlim(0, p["fmax"])
        for patch in spec["spans"]:
            patch.remove()
        spec["spans"] = [ax2.axvspan(lo, hi, color=c, alpha=.12, label=lab)
                         for lo, hi, c, lab in p["shade"]]
        ax2.legend(fontsize=7, loc="upper right")
        ax1.set_title(f"FILTERED — {p['band'][0]:.0f}-{p['band'][1]:.0f} Hz band-pass",
                      fontsize=10, loc="left")
        paint_buttons()

        # 5. the shading and the x-axis just moved, so the saved blit
        #    background is stale. Full redraw + recapture on the next frame.
        ui["stale"] = True

        gap = astats["gapsw"]; astats["gapsw"] = 0.0
        yr_s = f"±{ui['yr']:.5g} FS FIXED" if ui["yr"] else "pending (first 0.5 s)"
        if first:
            mode["report"] = False
            print(f"MODE = {p['label']}  band {p['band'][0]:.0f}-{p['band'][1]:.0f} Hz"
                  f" | lung high-pass {a.hp:.0f} Hz"
                  f" | playback gain {a.gain:g}", flush=True)
            return
        mode["switches"] += 1
        mode["t"] = t_sw
        mode["line"] = (
            f"MODE -> {p['label']}  band {p['band'][0]:.0f}-{p['band'][1]:.0f} Hz"
            f" | lung-hp {a.hp:.0f} Hz"
            f" | playback gain {a.gain:g} (unchanged by mode)"
            f" | y {yr_s}"
            f" | switch #{mode['switches']}"
            f", swap {1000 * (time.perf_counter() - t_sw):.2f} ms"
            f", max audio gap since last switch {1000 * gap:.1f} ms"
            f", dropped {stats['dropped']}, ui-drop {astats['ui_drop']}")
        mode["report"] = True

    def on_key(ev):
        if ev.key in ("h", "H"):
            apply_mode("heart"); return
        if ev.key in ("l", "L"):
            apply_mode("lung"); return
        if ev.key == "up":
            ui["yr"] = max(1.0 / 4096, ui["yr"] / 2)
        elif ev.key == "down":
            ui["yr"] = min(1.0, ui["yr"] * 2)
        elif ev.key == "q":
            stop.set(); return
        else:
            return
        ui["stale"] = True
        print(f"y-range changed: ±{ui['yr']:.5g} FS (fixed until next key)", flush=True)

    btn["heart"].on_clicked(lambda ev: apply_mode("heart"))
    btn["lung"].on_clicked(lambda ev: apply_mode("lung"))
    fig.canvas.mpl_connect("key_press_event", on_key)
    fig.canvas.mpl_connect("resize_event", lambda ev: ui.__setitem__("stale", True))
    fig.canvas.mpl_connect("close_event", lambda ev: stop.set())

    def redraw_background():
        t0 = time.perf_counter()
        ax0.set_ylim(-ui["yr"], ui["yr"]); ax1.set_ylim(-ui["yr"], ui["yr"])
        fig.canvas.draw()
        ui["bg"] = fig.canvas.copy_from_bbox(fig.bbox)
        # Blit only the regions that change: the three axes interiors and the
        # status strip along the top. Measured about twice as fast as blitting
        # the whole figure at 1200x850. The buttons are outside every blit
        # region, so they survive from the background untouched.
        W, H = fig.canvas.get_width_height()
        ui["blit"] = [ax0.bbox, ax1.bbox, ax2.bbox,
                      Bbox.from_extents(0, 0.962 * H, W, H)]
        ui["stale"] = False
        ui["draw_ms"] = 1000 * (time.perf_counter() - t0)
        if mode["report"]:
            mode["report"] = False
            print(f"{mode['line']}, redraw {ui['draw_ms']:.1f} ms"
                  f", total {1000 * (time.perf_counter() - mode['t']):.1f} ms", flush=True)

    def drain_ui_queue():
        got = 0
        while True:
            try:
                blk, y = q_ui.get_nowait()
            except queue.Empty:
                break
            raw_ring.push(blk); filt_ring.push(y); got += len(blk)
            r = rmeter["engine"].push(y)
            now = time.perf_counter()
            if r:
                rmeter["val"], rmeter["t"] = r, now
            elif rmeter["val"] is not None and \
                    now - rmeter["t"] > PRESETS[mode["name"]]["hold"]:
                rmeter["val"] = None      # nothing confirms it any more: "--"
        return got

    apply_mode(a.mode, first=True)
    print(f"listening on {a.port}, --fs-in {fs} Hz, playback gain {a.gain:g}")
    print("HEADPHONES ON. Opening scope window ...", flush=True)

    frames = 0; ft_sum = 0.0; ft_max = 0.0; fps_frames = 0
    t_start = time.perf_counter(); t_console = t_start
    rate_warned = False
    # Windows rounds time.sleep() up to the 15.6 ms scheduler tick, which turns
    # the 8 ms frame-cap sleep into 16 ms and pins the loop at ~21 fps. Ask for
    # 1 ms resolution while the scope runs.
    try:
        ctypes.windll.winmm.timeBeginPeriod(1)
    except Exception:
        pass
    plt.show(block=False)

    try:
        with sd.OutputStream(samplerate=fs, channels=1, dtype="float32",
                             blocksize=512, device=dev, callback=callback):
            t_stream = time.perf_counter()
            # ---- pick the FIXED y range once from the first 0.5 s of data
            t0 = t_stream; have = 0; said = False
            while not stop.is_set() and have < nspec and time.perf_counter() - t0 < 6.0:
                have += drain_ui_queue()
                fig.canvas.flush_events()
                if not said and time.perf_counter() - t0 > 2.0 and have == 0:
                    said = True
                    print("  no blocks decoded yet — see the message above if one appeared",
                          flush=True)
                time.sleep(0.02)
            if have == 0:
                if not stop.is_set():
                    print("  no data within 6 s, giving up", flush=True)
                return
            first = raw_ring.ordered()[-have:]
            rms0 = float(np.sqrt(np.mean((first - first.mean()) ** 2)))
            pk0 = float(np.abs(first).max())
            ui["yr"] = snap_range(rms0)
            print(f"y-range chosen once: ±{ui['yr']:.5g} FS "
                  f"(first {have/fs:.2f} s: rms {rms0:.4g} FS, peak {pk0:.4g} FS). "
                  f"FIXED from here on; UP/DOWN arrows to zoom.")
            print(f"spectrum axis fixed at {SPEC_YLIM[0]:.0f} .. {SPEC_YLIM[1]:.0f} dB", flush=True)
            redraw_background()

            # ---- render loop
            while not stop.is_set():
                tf = time.perf_counter()
                drain_ui_queue()
                if ui["stale"]:
                    redraw_background()

                raw = raw_ring.ordered(); flt = filt_ring.ordered()
                l_raw.set_ydata(envelope(raw, nb))
                l_filt.set_ydata(envelope(flt, nb))

                seg = raw[-nspec:]
                _, p = signal.welch(seg, fs, nperseg=nperseg)
                db = 10.0 * np.log10(p[spec["keep"]] + 1e-20)
                l_spec.set_ydata(db)
                spec["hold"] -= PEAK_DECAY_DB_S / TARGET_FPS
                np.maximum(spec["hold"], db, out=spec["hold"])
                l_hold.set_ydata(spec["hold"])

                rate = wire_rate(stats)
                pk = float(np.abs(raw).max()) if raw_ring.filled else 0.0
                pk_db = 20 * np.log10(pk) if pk > 1e-12 else -120.0
                at_rail = 100.0 * float(np.mean(np.abs(raw) >= rail)) if raw_ring.filled else 0.0
                rate_s = f"{rate:6.0f} Hz" if rate else "   --- Hz"
                # The reader blocks on the full audio queue until the stream
                # opens, then bursts through the backlog, so the rolling rate is
                # inflated for the first seconds. Judge it only once the 5 s
                # window is entirely steady-state.
                mismatch = rate is not None and abs(rate - fs) > 0.05 * fs \
                    and time.perf_counter() - t_stream >= 8.0
                if mismatch and not rate_warned:
                    rate_warned = True
                    print(f"\n  *** RATE MISMATCH: wire {rate:.0f} Hz vs --fs-in {fs} ***"
                          f"\n  re-run with --fs-in {rate:.0f}; bands and playback are wrong until then\n",
                          flush=True)
                if PRESETS[mode["name"]]["rate"] == "bpm":
                    rd = f"{rmeter['val']:5.0f} bpm  " if rmeter["val"] else "   -- bpm  "
                else:
                    rd = f"{rmeter['val']:5.1f} breaths/min" if rmeter["val"] \
                         else "   -- breaths/min"
                # lung-hp is labelled even in HEART mode, where it is not in
                # use: it is a session constant, and an unlabelled corner
                # frequency next to a HEART band would read as the heart one.
                line = (f"MODE: {PRESETS[mode['name']]['label']:<5}"
                        f" | lung-hp {a.hp:.0f} Hz"
                        f" | wire {rate_s} (--fs-in {fs})"
                        f"{'  *** RATE MISMATCH ***' if mismatch else ''}"
                        f" | dropped {stats['dropped']}"
                        f" | peak {pk_db:6.1f} dBFS | at rail {at_rail:5.2f} %"
                        f" | {rd} | y ±{ui['yr']:.5g} FS FIXED"
                        f"{'  CLIPPING - lower --gain' if meter['clip'] else ''}"
                        f"{'  ui-drop ' + str(astats['ui_drop']) if astats['ui_drop'] else ''}")
                status_txt.set_text(line)

                fig.canvas.restore_region(ui["bg"])
                ax0.draw_artist(l_raw); ax1.draw_artist(l_filt)
                ax2.draw_artist(l_spec); ax2.draw_artist(l_hold)
                fig.draw_artist(status_txt)
                for bb in ui["blit"]:
                    fig.canvas.blit(bb)
                fig.canvas.flush_events()

                dt = time.perf_counter() - tf
                frames += 1; fps_frames += 1; ft_sum += dt; ft_max = max(ft_max, dt)
                now = time.perf_counter()
                if now - t_console >= 5.0:
                    span = now - t_console
                    print(f"[{now - t_start:6.1f} s] fps {fps_frames / span:4.1f} | "
                          f"frame {1000 * ft_sum / max(1, fps_frames):5.1f} ms mean, "
                          f"{1000 * ft_max:5.1f} ms max | {line} | "
                          f"audio cb {astats['n']}, max gap {1000 * astats['gap5']:.0f} ms (5 s), "
                          f"status flags {astats['flags']}", flush=True)
                    t_console = now; fps_frames = 0; ft_sum = 0.0; ft_max = 0.0
                    astats["gap5"] = 0.0
                rest = 1.0 / TARGET_FPS - dt
                if rest > 0:
                    time.sleep(rest)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set(); ser.close()
        el = time.perf_counter() - t_start
        print(f"\nstopped after {el:.1f} s: {frames} frames ({frames / max(el, 1e-9):.1f} fps avg), "
              f"dropped {stats['dropped']}, audio callbacks {astats['n']}, "
              f"max callback gap {1000 * astats['maxgap']:.0f} ms, "
              f"status flags {astats['flags']}, ui-dropped blocks {astats['ui_drop']}")
        try:
            ctypes.windll.winmm.timeEndPeriod(1)
        except Exception:
            pass
        plt.close(fig)


if __name__ == "__main__":
    main()

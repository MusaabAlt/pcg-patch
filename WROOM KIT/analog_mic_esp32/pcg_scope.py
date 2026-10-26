#!/usr/bin/env python3
"""
pcg_scope.py — live scope for the PCG rig. Same serial input, same audio path
and the same flags as pcg_listen.py, plus a window with three stacked panels
and a status line. pcg_listen.py is untouched and remains the fallback.

    python pcg_scope.py --port COM15 --wire16 --fs-in 32000 --gain 100

  Panel 1  RAW, before the band-pass. Scrolling 5 s window.
  Panel 2  FILTERED, after the band-pass. Same 5 s window, same sample index,
           so what the filter removed is visible by comparing the two.
  Panel 3  Live spectrum of the RAW signal: Welch over the last 0.5 s, plus a
           slow-decaying peak-hold trace. 20-150 Hz (S1/S2) and 200-600 Hz
           (murmur) are shaded, same colours as analog_mic_record.py.

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

Controls: UP / DOWN = zoom y, q or window close = quit, Ctrl+C in console.
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

MAGIC    = b"\xA5\x5A\xA5\x5A"
FULL     = 2 ** 23
WINDOW_S = 5.0        # waveform panels, seconds
SPEC_S   = 0.5        # Welch window, seconds
PX       = 2000       # envelope bins per waveform panel
FMAX     = 1000.0     # spectrum x limit, Hz
SPEC_YLIM = (-120.0, -10.0)   # dB re FS^2/Hz, fixed
PEAK_DECAY_DB_S = 3.0         # peak-hold decay
TARGET_FPS = 30.0
COL_S1S2   = "#004AAD"        # same shading colours as analog_mic_record.py
COL_MURMUR = "#5A6480"


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
    """Copied from pcg_listen.py. Heart rate from envelope autocorrelation."""

    ENV_FS  = 100
    WINDOW  = 8
    LAG_LO  = 0.40
    LAG_HI  = 1.60

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
        if hi >= len(ac):
            return None
        lag = lo + int(np.argmax(ac[lo:hi]))
        if ac[lag] < 0.25 * ac[0]:
            return None
        return 60.0 * self.ENV_FS / lag


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
    ap.add_argument("--gain", type=float, default=300.0, help="playback gain")
    ap.add_argument("--band", nargs=2, type=float, default=[20, 400])
    ap.add_argument("--device", default=None,
                    help="output device INDEX (a number) or part of its name")
    ap.add_argument("--list-devices", action="store_true")
    a = ap.parse_args()

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
    astats = {"n": 0, "last": None, "maxgap": 0.0, "gap5": 0.0, "flags": 0, "ui_drop": 0}
    threading.Thread(target=reader,
                     args=(ser, q, width, dtype, scale, stop, stats),
                     daemon=True).start()

    sos = signal.butter(4, a.band, "bandpass", fs=fs, output="sos")
    zi = signal.sosfilt_zi(sos)
    tail = np.zeros(0)
    meter = {"clip": False}

    # ---- audio callback: identical to pcg_listen.py except for the
    # non-blocking hand-off of (raw, filtered) to the UI queue and timing stats.
    def callback(outdata, frames, t, status):
        nonlocal zi, tail
        now = time.perf_counter()
        if astats["last"] is not None:
            gap = now - astats["last"]
            if gap > astats["maxgap"]: astats["maxgap"] = gap
            if gap > astats["gap5"]:   astats["gap5"] = gap
        astats["last"] = now
        astats["n"] += 1
        if status:
            astats["flags"] += 1
        while len(tail) < frames:
            try:
                blk = q.get(timeout=0.5)
            except queue.Empty:
                outdata[:] = 0; return
            y, zi = signal.sosfilt(sos, blk, zi=zi)
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
    beats = BeatCounter(fs)
    bpm = None
    nspec = int(SPEC_S * fs)
    nperseg = 2048 if fs >= 16000 else 1024
    fr_full = np.fft.rfftfreq(nperseg, 1.0 / fs)
    keep = fr_full <= FMAX
    fr = fr_full[keep]
    peak_hold = np.full(fr.shape, SPEC_YLIM[0])

    # envelope x positions: bin centres, two points per bin, fixed
    nb = win // max(1, win // PX)
    b = win // nb
    xe = np.repeat((np.arange(nb) * b + b / 2.0) / fs - WINDOW_S, 2)

    fig, (ax0, ax1, ax2) = plt.subplots(3, 1, figsize=(12, 8.5))
    fig.canvas.manager.set_window_title("pcg_scope")
    fig.subplots_adjust(top=0.90, bottom=0.06, hspace=0.45)
    status_txt = fig.text(0.01, 0.975, "starting ...", family="monospace",
                          fontsize=9, animated=True)
    fig.text(0.99, 0.945, "UP/DOWN arrow: zoom y (both waveform panels)   q: quit",
             ha="right", fontsize=8, color="#5A6480")

    (l_raw,)  = ax0.plot(xe, np.zeros_like(xe), lw=0.5, color="#0C1745", animated=True)
    (l_filt,) = ax1.plot(xe, np.zeros_like(xe), lw=0.5, color=COL_S1S2, animated=True)
    (l_spec,) = ax2.plot(fr, np.full(fr.shape, SPEC_YLIM[0]), lw=0.8,
                         color="#0C1745", animated=True, label="Welch 0.5 s")
    (l_hold,) = ax2.plot(fr, peak_hold, lw=0.8, color="#D85A30", animated=True,
                         label=f"peak hold, -{PEAK_DECAY_DB_S:.0f} dB/s")
    for ax, title in ((ax0, "RAW — before filtering"),
                      (ax1, f"FILTERED — {a.band[0]:.0f}-{a.band[1]:.0f} Hz band-pass")):
        ax.set_xlim(-WINDOW_S, 0); ax.set_title(title, fontsize=10, loc="left")
        ax.set_xlabel("s", fontsize=8); ax.set_ylabel("FS", fontsize=8)
        ax.tick_params(labelsize=8)
    ax2.set_xlim(0, FMAX); ax2.set_ylim(*SPEC_YLIM)
    ax2.axvspan(20, 150, color=COL_S1S2, alpha=.12, label="S1/S2 20-150 Hz")
    ax2.axvspan(200, 600, color=COL_MURMUR, alpha=.12, label="murmur 200-600 Hz")
    ax2.set_title("RAW spectrum — Welch, last 0.5 s (fixed dB axis)", fontsize=10, loc="left")
    ax2.set_xlabel("Hz", fontsize=8); ax2.set_ylabel("dB re FS²/Hz", fontsize=8)
    ax2.tick_params(labelsize=8); ax2.legend(fontsize=7, loc="upper right")

    ui = {"yr": None, "stale": True, "bg": None}

    def on_key(ev):
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

    fig.canvas.mpl_connect("key_press_event", on_key)
    fig.canvas.mpl_connect("resize_event", lambda ev: ui.__setitem__("stale", True))
    fig.canvas.mpl_connect("close_event", lambda ev: stop.set())

    def redraw_background():
        ax0.set_ylim(-ui["yr"], ui["yr"]); ax1.set_ylim(-ui["yr"], ui["yr"])
        fig.canvas.draw()
        ui["bg"] = fig.canvas.copy_from_bbox(fig.bbox)
        # Blit only the regions that change: the three axes interiors and the
        # status strip along the top. Measured about twice as fast as blitting
        # the whole figure at 1200x850.
        W, H = fig.canvas.get_width_height()
        ui["blit"] = [ax0.bbox, ax1.bbox, ax2.bbox,
                      Bbox.from_extents(0, 0.962 * H, W, H)]
        ui["stale"] = False

    def drain_ui_queue():
        nonlocal bpm
        got = 0
        while True:
            try:
                blk, y = q_ui.get_nowait()
            except queue.Empty:
                break
            raw_ring.push(blk); filt_ring.push(y); got += len(blk)
            r = beats.push(y)
            if r:
                bpm = r
        return got

    print(f"listening on {a.port}, --fs-in {fs} Hz, band {a.band[0]:.0f}-{a.band[1]:.0f} Hz, "
          f"playback gain {a.gain:g}")
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
                db = 10.0 * np.log10(p[keep] + 1e-20)
                l_spec.set_ydata(db)
                peak_hold -= PEAK_DECAY_DB_S / TARGET_FPS
                np.maximum(peak_hold, db, out=peak_hold)
                l_hold.set_ydata(peak_hold)

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
                bpm_s = f"{bpm:5.0f} bpm" if bpm else "   -- bpm"
                line = (f"wire {rate_s} (--fs-in {fs})"
                        f"{'  *** RATE MISMATCH ***' if mismatch else ''}"
                        f" | dropped {stats['dropped']}"
                        f" | peak {pk_db:6.1f} dBFS | at rail {at_rail:5.2f} %"
                        f" | {bpm_s} | y ±{ui['yr']:.5g} FS FIXED"
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

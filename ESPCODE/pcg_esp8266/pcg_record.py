#!/usr/bin/env python3
"""
pcg_record.py — receive raw PCM from pcg_esp32.ino, save WAVs, plot, estimate BPM.

    pip install pyserial numpy scipy matplotlib

    python pcg_record.py --port COM5   --seconds 30 --out silence
    python pcg_record.py --port COM5   --seconds 30 --out chest01 --wire16   # ESP8266
    python pcg_record.py --port COM5   --seconds 30 --out chest01
    python pcg_record.py --port /dev/ttyUSB0 --seconds 30 --out chest01

Writes:
    <out>_raw16k.wav    unfiltered, on-wire rate, float32   <- open this in Audacity
    <out>_filt4k.wav    20-400 Hz band-pass, 4 kHz, float32
    <out>.png           waveform + zoom + spectrum
"""

import argparse, sys, time
import numpy as np
import serial
from scipy import signal
from scipy.io import wavfile
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

FS_IN   = 16000            # default; override with --fs-in
FS_OUT  = 4000
MAGIC   = b"\xA5\x5A\xA5\x5A"
FULL    = 2 ** 23          # 24-bit full scale


# ---------------------------------------------------------------- capture
def capture(port, baud, seconds, fs_in, wire16=False):
    """wire16=True for the ESP8266 sketch (int16 samples), False for ESP32 (int32)."""
    width, dtype, scale = (2, "<i2", 256) if wire16 else (4, "<i4", 1)
    need = int(fs_in * seconds)
    buf = np.empty(need, dtype=np.int32)
    got = 0

    ser = serial.Serial(port, baud, timeout=2)
    time.sleep(2.0)                    # let the board reset and settle
    ser.reset_input_buffer()
    exp_seq, dropped, blocks = None, 0, 0

    print(f"recording {seconds} s ... hold still, hold your breath if you can")
    t0 = time.time()
    window = b""
    while got < need:
        # resync on magic
        while MAGIC not in window:
            chunk = ser.read(512)
            if not chunk:
                raise RuntimeError("no data. wrong port, or STREAM is still 0 in the sketch.")
            window = (window + chunk)[-4096:]
        window = window[window.index(MAGIC) + 4:]

        while len(window) < 4:
            window += ser.read(4 - len(window))
        seq = int.from_bytes(window[0:2], "little")
        cnt = int.from_bytes(window[2:4], "little")
        window = window[4:]
        blocks += 1
        if exp_seq is not None and seq != exp_seq:
            dropped += (seq - exp_seq) % 65536
        exp_seq = (seq + 1) % 65536
        nbytes = cnt * width
        while len(window) < nbytes:
            window += ser.read(nbytes - len(window))
        block = np.frombuffer(window[:nbytes], dtype=dtype).astype(np.int32) * scale
        window = window[nbytes:]

        take = min(len(block), need - got)
        buf[got:got + take] = block[:take]
        got += take
        if got % (fs_in * 5) < len(block):
            print(f"  {got/fs_in:5.1f} s", flush=True)

    ser.close()
    el = time.time() - t0
    print(f"done in {el:.1f} s")
    print(f"  blocks {blocks}, dropped {dropped}"
          + ("   <-- TRANSPORT LOSS, clicks in the waveform are not acoustic"
             if dropped else "   (clean, no transport loss)"))
    print(f"  measured wire rate {got/el:.0f} Hz")
    return buf, blocks, dropped


# ---------------------------------------------------------------- analysis
def sanity_check(raw, wire16, dropped, blocks):
    """Catch a wire-format mismatch before anyone reads meaning into the plot.

    Samples are divided by 2**23, so |raw| can never exceed 1.0. If it does,
    the parser is gluing pairs of int16 into one int32 (or splitting an int32
    across two reads) and every number downstream is fiction. The overshoot is
    exactly 2**8 = 256 in the common case.
    """
    peak = float(np.abs(raw).max())
    bad = []
    if peak > 1.5:
        bad.append(
            f"peak is {peak:.1f}, but the maximum possible is 1.0 "
            f"(overshoot {peak:.0f}x ~ 2**{np.log2(max(peak,1)):.0f})\n"
            "    -> wire format mismatch. "
            + ("Remove --wire16 (ESP32 sends int32)."
               if wire16 else "Add --wire16 (ESP8266 sends int16).")
        )
    if blocks and dropped > blocks * 0.2:
        bad.append(f"dropped {dropped} of {blocks} blocks "
                   "-> framing is broken, not just lossy")
    if bad:
        print("\n  *** RECORDING INVALID ***")
        for b in bad:
            print("    " + b)
        print("    Files were still written, but do not interpret them.\n")
    return not bad


def bpm_estimate(x, fs):
    """envelope autocorrelation -> beats per minute"""
    env = np.abs(signal.hilbert(x))
    b, a = signal.butter(2, 20 / (fs / 2), "low")
    env = signal.filtfilt(b, a, env)
    env -= env.mean()
    ac = signal.correlate(env, env, mode="full")[len(env) - 1:]
    lo, hi = int(0.4 * fs), int(1.6 * fs)          # 37-150 bpm
    if hi >= len(ac):
        return None
    lag = lo + int(np.argmax(ac[lo:hi]))
    # With no real periodicity the argmax lands on the first bin of the search
    # window and this returns exactly 60/0.4 = 150.0 every time. That is a rail,
    # not a measurement. Require the peak to stand above the zero-lag energy.
    if ac[lag] < 0.25 * ac[0]:
        return None
    return 60.0 * fs / lag


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--seconds", type=float, default=30)
    ap.add_argument("--out", default="rec")
    ap.add_argument("--band", nargs=2, type=float, default=[20, 400])
    ap.add_argument("--fs-in", dest="fs_in", type=int, default=FS_IN,
                    help="on-wire sample rate set in the sketch (16000 or 24000)")
    ap.add_argument("--wire16", action="store_true",
                    help="ESP8266 sketch: samples arrive as int16, not int32")
    a = ap.parse_args()

    fs_in = a.fs_in
    # The ESP8266 clock divider cannot hit arbitrary rates (16 kHz lands on
    # 16384). Decimate by the nearest whole factor and report the rate we
    # actually got, instead of refusing to run.
    q = max(1, int(round(fs_in / FS_OUT)))
    fs_out = fs_in / q
    if abs(fs_out - FS_OUT) > 1:
        print(f"  note: decimating {fs_in} Hz by {q} -> {fs_out:.0f} Hz")

    raw, blocks, dropped = capture(a.port, a.baud, a.seconds, fs_in, a.wire16)
    raw = raw.astype(np.float64) / FULL
    sanity_check(raw, a.wire16, dropped, blocks)
    raw -= raw.mean()                                # kill DC offset

    sos = signal.butter(4, [a.band[0], a.band[1]], "bandpass", fs=fs_in, output="sos")
    filt = signal.sosfiltfilt(sos, raw)
    dec = signal.decimate(filt, q, ftype="fir", zero_phase=True)

    wavfile.write(f"{a.out}_raw{fs_in//1000}k.wav", fs_in, raw.astype(np.float32))
    wavfile.write(f"{a.out}_filt4k.wav", int(fs_out), dec.astype(np.float32))

    # ---- numbers that tell you whether this worked ----
    def dbfs(v):
        return 20 * np.log10(v) if v > 1e-12 else -200.0
    print()
    print(f"  peak, unfiltered      {dbfs(np.abs(raw).max()):7.1f} dBFS"
          f"   {'<-- CLIPPING' if np.abs(raw).max() > 0.98 else ''}")
    print(f"  rms, unfiltered       {dbfs(raw.std()):7.1f} dBFS")
    print(f"  rms, 20-400 Hz        {dbfs(dec.std()):7.1f} dBFS")
    q_lsb = np.unique(np.abs(np.diff(np.round(raw * FULL))))
    step = q_lsb[q_lsb > 0].min() if (q_lsb > 0).any() else 1
    print(f"  quantisation step     {step:7.0f} LSB"
          f"   ({'16-bit source, low 8 bits absent' if step >= 256 else '24-bit source'})")
    print(f"  peak in quant. steps  {np.abs(raw).max()*FULL/step:7.0f} counts"
          f"   {'<-- too few, signal is in the noise' if np.abs(raw).max()*FULL/step < 20 else ''}")
    # Clipping vs corrupted data. Real audio, even hard-clipped, stays
    # correlated sample to sample; a bad parse produces uncorrelated noise.
    r = raw / (np.abs(raw).max() + 1e-12)
    clip_pct = 100.0 * np.mean(np.abs(r) > 0.999)
    lag1 = float(np.corrcoef(raw[:-1], raw[1:])[0, 1])
    print(f"  samples at full scale  {clip_pct:6.2f} %"
          + ("   <-- CLIPPED. Amplitude information is destroyed."
             if clip_pct > 0.5 else ""))
    print(f"  lag-1 correlation      {lag1:6.3f}"
          + ("   (real signal, correlated)" if lag1 > 0.5
             else "   <-- near zero: this is not audio, the parse is wrong"))
    bpm = bpm_estimate(dec, fs_out)
    print(f"  dominant periodicity  {bpm:7.1f} bpm" if bpm
          else "  dominant periodicity     none   (no rhythm above the noise)")

    # ---- plot ----
    t = np.arange(len(dec)) / fs_out
    fig, ax = plt.subplots(3, 1, figsize=(12, 9))

    ax[0].plot(np.arange(len(raw)) / fs_in, raw, lw=0.3)
    ax[0].set_title(f"unfiltered, {fs_in/1000:g} kHz"); ax[0].set_xlabel("s")

    ax[1].plot(t, dec, lw=0.6, color="#004AAD")
    win = min(6.0, float(t[-1]))
    ax[1].set_xlim(0, win)
    # Scale y to the visible window, not the whole record. A single loud event
    # at t=13 s otherwise squashes the first 6 s into a flat-looking line.
    vis = dec[: int(win * fs_out)]
    if len(vis) and np.abs(vis).max() > 0:
        m = 1.15 * np.abs(vis).max()
        ax[1].set_ylim(-m, m)
    ax[1].set_title(f"{a.band[0]:.0f}-{a.band[1]:.0f} Hz — first {win:.0f} s, "
                    f"y scaled to this window"
                    + (f"   ({bpm:.0f} bpm)" if bpm else "   (no rhythm)"))
    ax[1].set_xlabel("s")

    f, p = signal.welch(raw, fs_in, nperseg=8192)
    ax[2].semilogy(f, p, lw=0.8, color="#0C1745")
    ax[2].set_xlim(0, 1000)
    ax[2].axvspan(20, 150, color="#004AAD", alpha=.12, label="S1/S2 20-150 Hz")
    ax[2].axvspan(200, 600, color="#5A6480", alpha=.12, label="murmur 200-600 Hz")
    ax[2].set_title("power spectral density (unfiltered)")
    ax[2].set_xlabel("Hz"); ax[2].legend()

    fig.tight_layout(); fig.savefig(f"{a.out}.png", dpi=130)
    print(f"\nwrote {a.out}_raw{fs_in//1000}k.wav  {a.out}_filt4k.wav  {a.out}.png")


if __name__ == "__main__":
    main()
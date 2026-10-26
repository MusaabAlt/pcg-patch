#!/usr/bin/env python3
"""
analog_mic_record.py — record from the 3-pin analogue mic on the ESP32.

    pip install pyserial numpy scipy matplotlib
    python analog_mic_record.py --port COM14 --seconds 30 --out amic01

Writes <out>_raw16k.wav, <out>_filt4k.wav, <out>.png

Separate from pcg_record.py on purpose — the failure modes of an analogue
front end are different and need different checks. For LIVE listening the
existing tool works unmodified, because the wire format is the same:

    python pcg_listen.py --port COM14 --wire16 --gain 300
"""

import argparse, sys, time
import numpy as np
import serial
from scipy import signal
from scipy.io import wavfile
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

MAGIC  = b"\xA5\x5A\xA5\x5A"
FS_IN  = 32000            # default; ALWAYS override with --fs-in if the
                          # sketch banner reports a different rate
FS_OUT = 4000
FULL   = 32768.0          # int16 full scale as sent by the sketch
ADC_LSB = 16              # sketch shifts 12-bit ADC left by 4


def capture(port, baud, seconds, fs_in):
    ser = serial.Serial(port, baud, timeout=2)
    time.sleep(2.0)
    ser.reset_input_buffer()
    need, got = int(fs_in * seconds), 0
    buf = np.empty(need, dtype=np.int32)
    w, exp, dropped, blocks = b"", None, 0, 0
    t0 = time.time()
    print(f"recording {seconds} s ...")

    while got < need:
        while MAGIC not in w:
            c = ser.read(512)
            if not c:
                raise RuntimeError("no data. Is STREAM set to 1 in the sketch?")
            w = (w + c)[-8192:]
        w = w[w.index(MAGIC) + 4:]
        while len(w) < 4:
            w += ser.read(4 - len(w))
        seq = int.from_bytes(w[0:2], "little")
        cnt = int.from_bytes(w[2:4], "little")
        w = w[4:]
        nb = cnt * 2
        while len(w) < nb:
            w += ser.read(nb - len(w))
        blk = np.frombuffer(w[:nb], dtype="<i2").astype(np.int32)
        w = w[nb:]
        blocks += 1
        if exp is not None and seq != exp:
            dropped += (seq - exp) % 65536
        exp = (seq + 1) % 65536
        take = min(len(blk), need - got)
        buf[got:got + take] = blk[:take]
        got += take
        if got % (fs_in * 5) < len(blk):
            print(f"  {got/fs_in:5.1f} s", flush=True)

    ser.close()
    el = time.time() - t0
    print(f"done in {el:.1f} s")
    print(f"  blocks {blocks}, dropped {dropped}"
          + ("   <-- TRANSPORT LOSS" if dropped else "   (clean)"))
    rate = got / el
    print(f"  measured wire rate {rate:.0f} Hz  (assumed {fs_in})")
    if abs(rate - fs_in) > fs_in * 0.05:
        print(f"  *** RATE MISMATCH — re-run with --fs-in {rate:.0f} ***")
        print("      Filter bands, PSD axes and playback speed are all wrong")
        print("      until this matches.")
    return buf


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--seconds", type=float, default=30)
    ap.add_argument("--out", default="amic")
    ap.add_argument("--band", nargs=2, type=float, default=[20, 400])
    ap.add_argument("--fs-in", dest="fs_in", type=int, default=FS_IN,
                    help="on-wire rate from the sketch banner, not what you hoped for")
    a = ap.parse_args()

    fs_in = a.fs_in
    q = max(1, int(round(fs_in / FS_OUT)))
    fs_out = fs_in / q
    raw_i = capture(a.port, a.baud, a.seconds, fs_in)
    x = raw_i.astype(np.float64) / FULL

    # ---- analogue-front-end specific checks ----
    adc = raw_i / ADC_LSB + 2048          # back to raw ADC counts
    print(f"\n  ADC mean         {adc.mean():7.0f} / 4095   (2048 is mid-rail)")
    print(f"  ADC min / max    {adc.min():7.0f} / {adc.max():.0f}")
    rail = 100.0 * np.mean((adc <= 1) | (adc >= 4094))
    if rail > 0.1:
        print(f"  AT THE RAILS     {rail:6.2f} %   <-- turn the gain trimpot DOWN")
    step = np.diff(np.unique(np.abs(raw_i)))
    step = step[step > 0].min() if len(step) else ADC_LSB
    print(f"  quantisation step{step:7.0f}   (16 = one 12-bit ADC count)")

    lag1 = float(np.corrcoef(x[:-1], x[1:])[0, 1])
    print(f"  lag-1 correlation{lag1:7.3f}"
          + ("   (real audio)" if lag1 > 0.5
             else "   <-- near zero: not audio. Check SWAP_PAIRS in the sketch."))

    def db(v):
        return 20 * np.log10(v + 1e-12)
    x = x - x.mean()
    print(f"  peak             {db(np.abs(x).max()):7.1f} dBFS")
    print(f"  rms              {db(x.std()):7.1f} dBFS")

    sos = signal.butter(4, a.band, "bandpass", fs=fs_in, output="sos")
    f_x = signal.sosfiltfilt(sos, x)
    dec = signal.decimate(f_x, q, ftype="fir", zero_phase=True)
    print(f"  rms {a.band[0]:.0f}-{a.band[1]:.0f} Hz  {db(dec.std()):7.1f} dBFS")

    # target window: loud enough to use the ADC, not so loud it rails
    pk = db(np.abs(x).max())
    if pk < -20:
        print("  -> too quiet. Turn the gain trimpot UP, aim for -12 to -3 dBFS.")
    elif rail > 0.1:
        print("  -> clipping. Turn the gain trimpot DOWN.")
    else:
        print("  -> level is in the usable window.")

    wavfile.write(f"{a.out}_raw{fs_in//1000}k.wav", fs_in, x.astype(np.float32))
    wavfile.write(f"{a.out}_filt4k.wav", int(fs_out), dec.astype(np.float32))

    t = np.arange(len(dec)) / fs_out
    win = min(6.0, float(t[-1]))
    fig, ax = plt.subplots(3, 1, figsize=(12, 9))
    ax[0].plot(np.arange(len(x)) / fs_in, x, lw=0.3)
    ax[0].axhline(1.0, color="#D85A30", lw=0.6, ls="--")
    ax[0].axhline(-1.0, color="#D85A30", lw=0.6, ls="--")
    ax[0].set_title("unfiltered — dashed lines are the ADC rails"); ax[0].set_xlabel("s")

    vis = dec[: int(win * fs_out)]
    ax[1].plot(t, dec, lw=0.6, color="#004AAD")
    ax[1].set_xlim(0, win)
    if len(vis) and np.abs(vis).max() > 0:
        m = 1.15 * np.abs(vis).max(); ax[1].set_ylim(-m, m)
    ax[1].set_title(f"{a.band[0]:.0f}-{a.band[1]:.0f} Hz — first {win:.0f} s")
    ax[1].set_xlabel("s")

    fr, p = signal.welch(x, fs_in, nperseg=8192)
    ax[2].semilogy(fr, p, lw=0.8, color="#0C1745")
    ax[2].set_xlim(0, 1000)
    ax[2].axvspan(20, 150, color="#004AAD", alpha=.12, label="S1/S2 20-150 Hz")
    ax[2].axvspan(200, 600, color="#5A6480", alpha=.12, label="murmur 200-600 Hz")
    ax[2].set_xlabel("Hz"); ax[2].legend(); ax[2].set_title("power spectral density")

    fig.tight_layout(); fig.savefig(f"{a.out}.png", dpi=130)
    print(f"\nwrote {a.out}_raw{fs_in//1000}k.wav  {a.out}_filt4k.wav  {a.out}.png")


if __name__ == "__main__":
    main()
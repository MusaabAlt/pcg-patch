# PCG patch — bench bring-up

A wearable adhesive phonocardiography (PCG) patch that records heart sounds from the chest.
This repo is the **bench rig**, not the product: the goal right now is to see repeating S1/S2
pairs at 60–90 bpm in a recording, and nothing else. The product build uses different parts
(nRF5340, IM69D130, IIS3DWB) and none of its constraints transfer here.

Full project history, frozen decisions and the ESP8266 open question live in
[`ESPCODE/pcg_esp8266/CLAUDE.md`](ESPCODE/pcg_esp8266/CLAUDE.md). **Read it before changing
anything.** This README summarises the system and the traps; CLAUDE.md is the authority on
what has been decided and why.

> **The one constraint that outranks everything else: no automatic gain control anywhere in
> the record path.** See [No AGC, ever](#no-agc-ever). If you read only one section, read that one.

---

## Read this before you trust anything else in the repo

This README was written by auditing every file on disk, not from memory. Four things did not
match the project's own documentation, and they will bite you:

1. **Only two hardware variants exist**, not four. There is no ESP32 + digital-mic sketch
   (mono or stereo) and no combined ESP32 + ECG sketch anywhere in this tree. A file named
   `pcg_esp32.ino` is referenced by `pcg_record.py` and by `analog_mic_esp32.ino`, but it is
   **not in the repo**. If you were told those variants exist, they are somewhere else or gone.
2. **CLAUDE.md says "the ESP32 variant sends int32 and omits the flag." That is wrong for the
   ESP32 rig that actually exists.** The analog ESP32 sketch sends **int16** and needs
   `--wire16`, exactly like the ESP8266. The int32 path is currently used by no sketch here.
3. **There are two diverged copies of `pcg_listen.py`.** The one in `WROOM KIT/analog_mic_esp32/`
   is current. See [Duplicate files](#duplicate-and-stale-files).
4. **CLAUDE.md predates the ESP32 work.** It is dated 2026-09-01 and is scoped to the ESP8266
   rig. Everything about the analog ESP32 rig, including the sample-rate clamp below, happened
   after it was written and is documented only here.

---

## Hardware variants

| Variant | Board / port | Sensor | Firmware | Receivers | Wire | Status |
|---|---|---|---|---|---|---|
| **ESP32 + analog mic** | ESP32-WROOM-32, COM15, 240 MHz | MAX4466 electret module, trimpot 28–42 dB, **no AGC** | `WROOM KIT/analog_mic_esp32/analog_mic_esp32.ino` | `analog_mic_record.py`, `pcg_listen.py`, `pcg_scope.py` | int16 @ 32000 Hz | **Current working reference.** Clock verified, live scope passed acceptance testing 2026-09-03. |
| **ESP8266 + digital I2S mic** | NodeMCU V3 (ESP-12E), COM14, 160 MHz | INMP441-style I2S MEMS, identity unconfirmed | `ESPCODE/pcg_esp8266/pcg_esp8266.ino` | `pcg_record.py`, `pcg_listen.py` | int16 @ ~16384 Hz | **Exploratory, blocked.** I2S works, but the noise-floor question in CLAUDE.md is unresolved. |
| ESP32 + digital mic (mono/stereo) | — | — | — | — | — | **Does not exist in this repo.** |
| ESP32 + ECG combined | — | — | — | — | — | **Does not exist.** GPIO 34 is reserved for it in the analog sketch, nothing more. |

The ESP32 analog rig is the one to use. It is the only rig whose sample rate has been
verified end to end and the only one with a live scope.

### Files, in full

```
EKG/
├── README.md                                   <- you are here
├── ESPCODE/pcg_esp8266/
│   ├── CLAUDE.md                               <- project history, frozen decisions
│   ├── pcg_esp8266.ino                         firmware, ESP8266 + I2S mic
│   ├── pcg_listen.py                           STALE COPY — do not use
│   └── pcg_record.py                           recorder, has --wire16, 24-bit full scale
└── WROOM KIT/analog_mic_esp32/
    ├── analog_mic_esp32.ino                    firmware, ESP32 + analog mic
    ├── analog_mic_record.py                    recorder, analog-specific diagnostics
    ├── pcg_listen.py                           CURRENT copy — live audio only
    └── pcg_scope.py                            live audio + three-panel scope UI
```

### Duplicate and stale files

**`pcg_listen.py` exists twice and the copies have diverged.** The copy in
`WROOM KIT/analog_mic_esp32/` (2026-09-02, 9851 bytes) is current. The copy in
`ESPCODE/pcg_esp8266/` (2026-09-01, 7107 bytes) is missing three fixes that each cost real
debugging time: the "receiving data, but no sync word found" detector, the `--device`
numeric-index fix, and the Bluetooth Hands-Free warning. Do not run the ESP8266 copy.

Verify which copy you have before trusting it:

```bash
grep -c "no sync word found" pcg_listen.py     # 1 = current, 0 = stale
```

**The two recorders are genuinely different and are not interchangeable.**
`pcg_record.py` takes `--wire16` and normalises to 2^23 (24-bit full scale, correct for the
ESP8266's digital mic). `analog_mic_record.py` hardcodes int16, has no `--wire16` flag, and
normalises to 32768 (correct for the ESP32's 12-bit ADC shifted left by 4). Because the two
use different full-scale references, **dBFS numbers from the two rigs are not comparable in
absolute terms** — each is a fraction of its own rig's full scale.

---

## The signal chain

### a. Digital mic vs analog mic, and what changes

**ESP8266, digital I2S MEMS mic.** The mic does its own analog-to-digital conversion and
delivers a clean digital word. Its bandwidth is already limited by an internal decimation
filter, so aliasing is not a concern. The catch is that the ESP8266's I2S peripheral captures
16 bits per sample while the mic sends 24 — you get the top 16 and the bottom 8 do not exist
on this chip. That is an accepted limitation of the bench rig, not a bug.

**ESP32, analog electret module.** The MAX4466 outputs an analog voltage that the ESP32's
built-in ADC samples through the I2S peripheral in `ADC_BUILT_IN` mode — DMA-paced hardware
sampling, not `analogRead()` in a loop, because `analogRead()` jitters and the jitter appears
as noise. Two consequences follow, and both matter:

- **The ADC is roughly 10 effective bits**, not 12. Combined with the module's own noise this
  sets a floor the microphone cannot get below.
- **There is no anti-alias filter fitted.** The module's bandwidth runs past 20 kHz. Everything
  above Nyquist folds down into the audio band irreversibly — once sampled it is
  indistinguishable from real signal and no digital filter can remove it. The firmware header
  documents the two-component fix (1 kΩ series + 100 nF to ground, corner 1.6 kHz, above the
  600 Hz murmur band and below Nyquist). **It is not fitted in hardware yet.** Raising the
  sample rate to 32 kHz moved Nyquist to 16 kHz, which reduces how much folds in, but does not
  eliminate it.

**What changes downstream:** both rigs send **int16**, so both need `--wire16` on
`pcg_listen.py` and `pcg_scope.py`. `analog_mic_record.py` assumes int16 unconditionally and
has no such flag. `pcg_record.py` needs `--wire16` explicitly.

### b. Sample rate — two silent-failure traps

Neither board reliably gives you the rate you asked for, and **neither one raises an error when
it does not.** Every band edge, every spectrum axis and the playback speed depend on the rate
being right, so a wrong rate silently corrupts every number downstream.

**Trap 1 — ESP32 ADC mode clamps the clock below ~26.4 kHz.** In `ADC_BUILT_IN` mode the mclk
divider is capped at 255. A 16 kHz request needs a divider near 420, so the driver clamps it,
logs an error to the boot console, and **`i2s_driver_install()` still returns `ESP_OK`.** The
sketch cannot see the failure. Measured result: **9739 Hz on the wire while the sketch said
16000** — the arithmetic is 16000 × 255/420 ≈ 9714 Hz. Everything downstream was then wrong by
a factor of 1.64, including `pcg_listen.py`, whose audio callback consumed 16000 samples per
second while being fed 9739, underran, and produced gaps and a level that appeared to drift on
its own.

The boot log looked like this:

```
E (316) i2s(legacy): i2s_calculate_adc_dac_clock(765): sample rate is too small, the mclk division exceed the maximum value 255
E (316) i2s(legacy): i2s_calculate_clock(860): ADC/DAC clock calculate failed
warm-up: 9728 samples in 1 s (expect ~16000)
```

**The fix is to raise the rate, not to enable APLL.** The APLL path is not honoured in ADC
mode; the divider cap is the actual constraint. The minimum rate that clears it is
16000 × 420/255 ≈ 26.4 kHz, so `SAMPLE_RATE` is now **32000**: comfortably clear of the cap, it
divides by 8 to the 4 kHz analysis rate exactly, and it puts Nyquist at 16 kHz. The banner now
calls `i2s_get_clk()` and prints the rate the driver actually produced:

```
requested 32000 Hz | driver reports 32000 Hz
warm-up: 31744 samples in 1 s (expect ~32000)
```

**Trap 2 — the board's own warm-up counter is not a rate measurement.** On the ESP8266 the
warm-up loop accumulates in 256-sample chunks and keeps going until 1000 ms have elapsed, so
the call that crosses the boundary completes past 1 s. Its resolution is therefore ±256
samples per second, and it overshoots. CLAUDE.md records "true wire rate is 16384 Hz, not
16000" on the strength of a `warm-up: 16384 samples in 1 s` line. That reading is *consistent*
with a true 16384 (256 divides 16384 evenly), and a true 16000 would print 15872 or 16128
instead — but the counter is a coarse sanity check, not a measurement, and **the ESP8266 rate
has never been confirmed on the PC side.**

**Trust the receiver, not the board.** Every receiver prints a `measured wire rate` line
computed from samples actually delivered over wall-clock elapsed time.
`analog_mic_record.py` and `pcg_scope.py` go further and warn when it differs from `--fs-in`
by more than 5 %:

```
  *** RATE MISMATCH — re-run with --fs-in 31800 ***
```

Procedure for any rig, every time: run once, read `measured wire rate`, then pass that number
as `--fs-in`. For the ESP32 analog rig the driver reports 32000 and the receiver measures
31750–31825, well inside 5 %; use `--fs-in 32000`. The 0.6 % shortfall is host-side USB read
timing, not the ADC clock.

### c. The filter

**Band: 20–400 Hz, fourth-order Butterworth band-pass.**

The physiology sets the edges. **S1 and S2 sit at 20–150 Hz**; **murmurs sit at 200–600 Hz**.
The 20 Hz lower edge removes DC offset, breathing and motion artefact, which are large and
carry no diagnostic content. The 400 Hz upper edge covers all of S1/S2 and the lower half of
the murmur band while excluding most room noise and speech energy above it — a deliberate trade
that favours S1/S2 clarity, which is the current goal. **If you are chasing murmurs rather than
S1/S2, pass `--band 20 600` explicitly**; every script takes `--band` and the default is not
the whole murmur band. Both shaded regions are drawn on every spectrum plot so you can see
where the energy actually landed relative to both bands.

**Why Butterworth.** It is maximally flat in the passband, with no ripple. Chebyshev and
elliptic designs buy steeper skirts by accepting passband ripple — and ripple means different
frequencies inside the band come out scaled differently. Since the amplitude relationships
inside the band *are* the measurement here, ripple is not an acceptable trade. Fourth order
gives enough rejection without the long ringing that a higher order would smear across S1,
which is a transient a few tens of milliseconds long.

**Why second-order sections (`sos`) and not transfer-function `b, a` coefficients.** A
fourth-order band-pass is eighth order overall, and with a 20 Hz corner at 32 kHz its poles sit
at radius 0.9995 — right against the unit circle. Converting that to direct-form coefficients
is numerically ill-conditioned: SciPy emits *"the filter's denominator is extremely small"* and
the coefficient magnitudes span ~63× before rounding error is even considered. Cascaded
second-order sections keep each stage well conditioned. This is not stylistic; a direct-form
implementation of this filter can go unstable.

#### No AGC, ever

**The amplitude ratio between S1, S2 and any murmur IS the diagnostic information.** A murmur
is graded by how loud it is *relative to* the heart sounds around it. S2 louder than S1, or a
murmur that rises and falls across the cardiac cycle, is the finding. Any process that varies
gain over time rewrites exactly that relationship and destroys the measurement while leaving a
recording that still sounds fine — which is what makes it dangerous. It fails silently and
convincingly.

This rules out, with no exceptions and no "just for listening" carve-outs:

- **Automatic gain control**, in hardware or software. The MAX4466 was chosen over the MAX9814
  specifically because the MAX9814's AGC cannot be disabled.
- **Spectral subtraction and any noise reduction that rewrites levels.**
- **Per-frame autoscaling of a display y-axis.** This is the subtle one. An axis that rescales
  every frame is a visual AGC: a weak S2 next to a strong S1 renders identically to two equal
  beats. `pcg_scope.py` therefore chooses its waveform y-range **once** at startup, prints it,
  and freezes it. Zoom is manual only, on the arrow keys, and every change is printed. There is
  no autoscale option, on or off.

Fixed gain is set once with the trimpot and left alone. Two operations that look like gain
changes but are not, and are therefore allowed: the firmware's fixed subtraction of the 2048
mid-rail DC offset, identical on every sample; and `--gain`, which scales headphone playback
only and never touches the wire or anything recorded.

#### `sosfiltfilt` offline vs `sosfilt` streaming — and why the delay is correct

The recorders (`pcg_record.py`, `analog_mic_record.py`) use **`sosfiltfilt`**: it runs the
filter forwards and then backwards over the whole array. The two passes cancel each other's
phase, so the result is **zero-phase** — no time shift at all — at the cost of being
**non-causal**, since computing an output sample requires samples that come after it. That is
fine offline, where the entire recording is already on disk, and it is the right choice there
because a phase shift that varies with frequency would smear the shape of S1 and S2 and change
where the peaks appear relative to each other.

The live tools (`pcg_listen.py`, `pcg_scope.py`) use **`sosfilt` with carried state** (`zi`),
which processes each arriving block and remembers the filter state for the next one. This is
**causal** and therefore has group delay:

| Frequency | Group delay |
|---|---|
| 30 Hz | 13.6 ms |
| 60 Hz | 3.6 ms |
| 100 Hz | 2.0 ms |
| 150 Hz | 1.5 ms |
| 300 Hz | 1.6 ms |

These figures are identical on both rigs to within 0.1 ms, because the delay is set by the
filter's corner frequencies rather than by the sample rate.

**That delay is correct behaviour and must not be "fixed".** Zero-phase filtering in real time
would require knowing the future. The only ways to hide the delay are to buffer ahead, which
adds far more latency than it removes, or to time-shift the display, which would misalign the
filtered panel against the raw one and defeat the whole point of showing both. A few
milliseconds is far below the ~40 ms gap between S1 and S2, so it changes nothing you are
trying to see. Carrying `zi` across blocks is also what prevents a filter transient at every
block boundary, which would otherwise appear as a click at the block rate.

#### Why both scope panels matter

`pcg_scope.py` shows the raw signal and the filtered signal on **the same sample indices and
the same time axis**, so the two panels are aligned by construction. The comparison is the
diagnostic:

- A transient present in both at the same x position is **in band** — a real heart sound, a tap,
  a genuine acoustic event.
- Something large in raw and largely gone in filtered is **out of band** — speech, handling
  noise, rumble. That is the filter doing its job, and seeing it work is how you gain confidence
  that what survives is real.
- Something that appears in filtered but has no counterpart in raw is a **filter artefact or a
  parse error**, not sound.

Judging the filtered panel alone tells you nothing about what was removed, which is why a
single-panel display is not sufficient.

### d. Wire protocol

Identical on both rigs, deliberately, so the same receivers work unmodified:

```
A5 5A A5 5A | uint16 seq (LE) | uint16 count (LE) | count x int16 (LE)
```

**The magic word** is a resync point. Serial is a byte stream with no inherent framing; if the
receiver starts mid-block or loses bytes, it needs a pattern to hunt for to recover alignment.

**`seq` increments once per block and wraps at 65536**, and every receiver checks it and reports
`dropped N`. This exists to answer one specific question: *is that click a real sound or a
transport artefact?* Lost blocks get spliced together into a step discontinuity, which is a
broadband click that looks and sounds exactly like a genuine impulsive event. Without `seq` the
two are indistinguishable, and you can spend a long time chasing an acoustic explanation for a
USB problem. **Non-zero `dropped` means clicks in that waveform are artefacts. Do not interpret
them.**

**int16 on both rigs, and the misparse signature.** The ESP8266 has only 16 bits to give, and
halving the byte rate to 32 kB/s keeps the UART clear of dropouts; the ESP32 analog rig sends
its 12-bit ADC value shifted left by 4, which also fits in int16. The int32 path in the
receivers exists for a digital-mic ESP32 sketch that is not in this repo.

Getting this flag wrong is a silent, convincing failure, so learn its fingerprint. If a
receiver parses an int16 stream as int32 it glues sample pairs together and consumes twice the
bytes per block, swallowing the next block's header into the data. Two signatures follow, both
reproduced in simulation:

- **The y-axis reads about ±250** instead of ±1. The arithmetic is exact: two int16 at full
  scale glued into one int32 give 2^31, and the receivers divide by 2^23, so 2^31 / 2^23 = **256**.
  A waveform that cannot exceed 1.0 by construction but shows 250 is not loud — it is misparsed.
  The magic word itself, read as one int32, lands at 181 and appears as a periodic impulse.
- **A harmonic comb at 125 Hz.** The swallowed headers recur once per block, so the artefact
  frequency is the block rate. Both rigs land on the same number by coincidence:
  ESP8266 is 16000/128 and ESP32 analog is 32000/256, and **both equal 125 Hz**. Simulation of
  the misparse puts the strongest lines at 125 Hz and multiples of roughly 250 Hz. A comb of
  evenly spaced lines with no physiological meaning is a framing artefact, not a spectral
  feature of anything acoustic — heart sounds are broadband transients and do not produce one.

`pcg_record.py` catches this automatically and refuses to let you read meaning into the plot:

```
  *** RECORDING INVALID ***
    peak is 250.3, but the maximum possible is 1.0 (overshoot 250x ~ 2**8)
    -> wire format mismatch. Add --wire16 (ESP8266 sends int16).
    Files were still written, but do not interpret them.
```

There is a second, independent check for the same class of bug: **lag-1 correlation**. Real
audio is strongly correlated sample to sample, so lag-1 runs above 0.5. A bad parse produces
near-zero correlation because consecutive values are unrelated. Both recorders print it.

---

## Quickstart

### Shared setup

All paths below are relative to the repo root — the directory containing this README. Commands
are PowerShell on Windows; the shell matters only for the `$CLI` variable syntax.

**Python.** 3.10 or later:

```powershell
pip install pyserial numpy scipy matplotlib sounddevice
```

**arduino-cli.** v1.5.1 is installed on this bench but **not on PATH** — it is bundled inside
the Arduino IDE. Set this once per shell session; every flash command below uses it:

```powershell
$CLI = "C:\Users\HP\AppData\Local\Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe"
```

If you do not have the IDE, install `arduino-cli` from <https://arduino.github.io/arduino-cli/>
and use `arduino-cli` in place of `& $CLI`.

**Board cores.** Install whichever you need. The versions below are what this bench is verified
against — the ESP32 clock-clamp behaviour in [Trap 1](#b-sample-rate--two-silent-failure-traps)
is specific to core 3.x, which uses the ESP-IDF 5.x legacy I2S driver:

```powershell
& $CLI core install esp32:esp32           # verified at 3.3.11
& $CLI core install esp8266:esp8266       # verified at 3.1.2
& $CLI core list                          # confirm what you actually have
```

**Find your board's port.** COM15 and COM14 below are this bench's assignments and yours will
differ. List them:

```powershell
& $CLI board list
```

On Windows, if no port appears at all, install the **CH340 USB-serial driver** first — the
NodeMCU V3 and many ESP32 dev boards use that chip and Windows does not ship a driver for it.

**Only one program can hold a COM port.** Close the Serial Monitor before running any Python
script, and close Python before opening the monitor. This is the single most common way to
waste five minutes here. The error you get is in the
[failures table](#common-failures).

### ESP32 + analog mic (the working reference)

**Wiring.** GPIO 35 is ADC1 channel 7 and is input-only. GPIO 34 is left free deliberately —
that is where the ECG goes later.

| MAX4466 | ESP32 |
|---|---|
| VCC | 3V3 |
| GND | GND |
| OUT | GPIO 35 |

**Flash settings.** Board `esp32:esp32:esp32`, CPU 240 MHz, port COM15.

**Step 1 — flash with `STREAM 0` and read the banner.** Set `#define STREAM 0` in
`analog_mic_esp32.ino`, then:

```powershell
cd "WROOM KIT\analog_mic_esp32"
& $CLI compile --fqbn esp32:esp32:esp32:CPUFreq=240 .
& $CLI upload  --fqbn esp32:esp32:esp32:CPUFreq=240 --port COM15 .
& $CLI monitor -p COM15 -c baudrate=921600
```

Three lines decide whether to continue:

```
requested 32000 Hz | driver reports 32000 Hz     <- must match, or see Trap 1
warm-up: 31744 samples in 1 s (expect ~32000)
DC bias: 1864 / 4095  (1.50 V)                   <- must be near mid-rail
```

**DC bias near 2048 means the mic is powered and on the right pin.** Near 0 or near 4095 means
it is not, and every recording after that would be meaningless. The measured 1864 is normal for
this module. Use `STREAM 0` exactly once, here.

**Step 2 — set `#define STREAM 1`, re-flash, and confirm the line reads `#define STREAM 1`.**
Stay there for all recording. Close the monitor.

**Step 3 — record, listen, or scope.**

```powershell
python analog_mic_record.py --port COM15 --fs-in 32000 --seconds 20 --out chest01
python pcg_listen.py --port COM15 --wire16 --fs-in 32000 --gain 30
python pcg_scope.py  --port COM15 --wire16 --fs-in 32000 --gain 30
```

`analog_mic_record.py` writes `chest01_raw32k.wav`, `chest01_filt4k.wav` and `chest01.png` **into
the current working directory**, and prints dropped blocks, measured wire rate, ADC mean/min/max,
rail percentage, quantisation step, lag-1 correlation, peak and RMS. Note it takes **no**
`--wire16` — it assumes int16.

`pcg_scope.py` adds a three-panel live window on top of the same audio path as `pcg_listen.py`:
raw, filtered, and a live spectrum with a slow-decaying peak-hold. It was verified on this bench
at 25 fps with zero dropped blocks, and audio was confirmed to keep playing through a deliberate
4-second freeze of the plotting thread. Use headphones — laptop speakers cannot reproduce
20–150 Hz and you will conclude the rig is broken when it is not.

In `pcg_scope.py`: click the plot for keyboard focus, then **UP** halves the y-range and
**DOWN** doubles it, each change printed. **q** or closing the window quits. Sensor placement is
the 4th intercostal space at the left sternal border — not the apex, because the apex beat moves
with posture and a fixed sensor cannot follow it.

### ESP8266 + digital I2S mic (exploratory)

**Wiring. These pins are fixed in silicon and cannot be moved.** Never touch S0 S1 S2 S3 SC SK
on the left header — those are GPIO6–11, wired to the flash chip, and shorting them bricks the boot.

| Mic | GPIO | NodeMCU V3 |
|---|---|---|
| VDD | — | 3V |
| GND | — | G |
| L/R | GND | G |
| WS | GPIO14 | D5 |
| SD | GPIO12 | D6 |
| SCK | GPIO13 | D7 |

**Flash settings.** CPU **must** be 160 MHz; at 80 MHz samples drop.

```powershell
cd "ESPCODE\pcg_esp8266"
& $CLI compile --fqbn "esp8266:esp8266:nodemcuv2:xtal=160,eesz=4M2M,baud=115200" .
& $CLI upload  --fqbn "esp8266:esp8266:nodemcuv2:xtal=160,eesz=4M2M,baud=115200" --port COM14 .
```

Serial for data is **921600**. The ROM boot log is **74880** — set the monitor there to see
resets and crash dumps.

Run `STREAM 0` first to confirm which slot the mic is on (`MIC_SLOT 0` = left is correct for
L/R tied to GND). The right slot sits at a constant −4.7 dBFS; that is a floating tri-stated
line, not audio, and is expected. Then `STREAM 1` and:

```powershell
python pcg_record.py --port COM14 --wire16 --fs-in 16384 --seconds 30 --out chest07
```

**Read the `measured wire rate` line and re-run with that value as `--fs-in`.** The 16384
default comes from the board's own coarse warm-up counter and has never been confirmed on the
PC side. Do not use the `pcg_listen.py` in this folder; it is the stale copy.

---

## Common failures

Every symptom below has actually happened on this bench and cost time.

| Symptom you will see | Cause | Fix |
|---|---|---|
| Level meter sits at exactly `-120.0 dBFS` forever, no error | `#define STREAM` is still `0`, so the board is emitting text and the receiver spins looking for a sync word that never comes | Set `STREAM 1`, re-flash, close the Serial Monitor |
| `*** receiving data, but no sync word found ***` followed by `It looks like TEXT, so STREAM is still 0 in the sketch.` | Same as above, but detected | Same as above. Only the current `pcg_listen.py` and `pcg_scope.py` print this |
| Waveform y-axis reads about **±250** when it cannot exceed 1.0 | int16 stream parsed as int32; 2^31/2^23 = 256 | Add `--wire16` |
| **Harmonic comb at 125 Hz** and multiples, mistaken for an acoustic feature | Same misparse. 125 Hz is the block rate on both rigs (16000/128 and 32000/256) | Add `--wire16`. Heart sounds never produce an evenly spaced comb |
| `*** RECORDING INVALID *** peak is 250.3, but the maximum possible is 1.0` | Same misparse, caught automatically | Add `--wire16`. The files were written but mean nothing |
| `lag-1 correlation 0.037 <-- near zero: this is not audio, the parse is wrong` | Same class of bug, independent detector | Check `--wire16` first. If that is right, try `#define SWAP_PAIRS 1` in `analog_mic_esp32.ino` — some builds deliver the two 16-bit halves of each DMA word reversed. Only touch it if the DC bias looked sane but the data does not |
| Boot log: `mclk division exceed the maximum value 255` and `ADC/DAC clock calculate failed`, then `warm-up: 9728 samples in 1 s (expect ~16000)` | ESP32 ADC-mode divider cap. Driver clamps to ~9714 Hz and still returns `ESP_OK` | Raise `SAMPLE_RATE` to 32000. **Not** `use_apll` — APLL is not honoured in ADC mode |
| `*** RATE MISMATCH — re-run with --fs-in 31800 ***` | Wire rate differs from `--fs-in` by more than 5 % | Re-run with the measured rate. Every band edge and the playback speed depend on it |
| Live audio has gaps, crackle, and a level that "changes constantly" on its own | `--fs-in` does not match the wire; the audio callback starves or overruns | Pass the measured rate. This was misread as an AGC symptom once |
| `*** WARNING: this is the Bluetooth HANDS-FREE profile.` and you hear nothing | HFP is 8 kHz telephony, 300–3400 Hz. It removes the entire 20–150 Hz heart band before you hear anything | Use wired headphones, or force the A2DP profile |
| `could not open port 'COM15': PermissionError(13, 'Access is denied.', None, 5)` | Another program holds the port — usually the Serial Monitor, or a Python script still running | Close the other program. Only one at a time |
| `--device 3` matches several devices and then errors | A numeric string is treated as a name search, not an index | Fixed in the current `pcg_listen.py` and `pcg_scope.py`. Run `--list-devices` and pass the bracketed number |
| Meter shows `CLIPPING - lower --gain` on the scope | Headphone playback only, not the ADC. `--gain 100` on real beats exceeds 1.0 | Lower `--gain`. The wire and any recording are unaffected |
| Recording is at the rails, `AT THE RAILS 3.20 %` | Analog trimpot set too high | Turn the trimpot **down**. Aim for a peak of −12 to −3 dBFS |
| Everything reads plausibly but `dropped` is non-zero | Blocks lost in transport, spliced into step discontinuities | The clicks are artefacts. Do not interpret that recording |

---

## What is open

### From CLAUDE.md — the ESP8266 noise floor (unresolved)

A 30 s "silence" recording came back at **−28.5 dBFS RMS, peak −10.2**. Back-calculated through
the INMP441 sensitivity that is about 91 dB SPL, which no desk produces. The energy is
concentrated in discrete transients every 1–2 s, not in a continuous floor: peak −10.2 at
roughly 1 % duty cycle predicts −30.2 dBFS RMS, within 2 dB of what was measured. **So the
transients are the entire reading and the real noise floor has not been seen yet.**

Three candidates, not yet separated:

1. Real impulsive sound in the room.
2. Something physically touching the board or wires.
3. Blocks lost in transport, spliced back into step discontinuities.

Candidate 3 is now detectable via `seq`. Candidates 1 and 2 separate with a muffled recording.
CLAUDE.md's position is that **nothing downstream is worth touching until this is resolved.**

### From the current ESP32 bring-up (documented here only)

The clock is fixed and the tooling is verified, but the noise investigation itself **has not been
run**. A four-recording plan is defined and pending, 20 s each via `analog_mic_record.py`:

- **A** — new sketch flashed, quiet room, mic untouched. Isolates WiFi/Bluetooth coupling, which
  was on during all earlier recordings and is now off in `setup()`.
- **B** — trimpot fully counter-clockwise, minimum analog gain. Isolates gain stacking.
- **C** — mic OUT disconnected, GPIO 35 wired straight to GND. Gives the floor the ADC and
  wiring contribute with no microphone at all. **This is the discriminator**: if A and B sit
  close to C, the microphone is irrelevant and a better capsule will not help.
- **D** — power split across 20–400 Hz, 400–2000 Hz and 2–8 kHz from C and B. A spectrum flat
  out to Nyquist points at aliasing or white ADC noise rather than anything acoustic.

Two observations are recorded but **not yet conclusions**, pending recording C:

- At the 9.7 kHz rate the stream had lag-1 correlation 0.037, essentially white at the sample
  level. At 32 kHz with a chest on the mic it reads 0.646.
- The idle floor is **spiky, not Gaussian**: a ±450-count min/max spread against a 13.9-count
  standard deviation, and on the scope it renders as narrow, largely negative-going spikes
  rather than a band. Both point toward the ADC, the supply or aliasing rather than the
  microphone, but C decides it.

Also open on this rig: **no anti-alias RC filter is fitted** (documented in the firmware header,
two components), and **no supply decoupling capacitor** is fitted near the module.

---

## For a fresh Claude Code session

**Read [`ESPCODE/pcg_esp8266/CLAUDE.md`](ESPCODE/pcg_esp8266/CLAUDE.md) before doing anything.**
It contains frozen decisions — sample rate, gain strategy, filter bands, wire protocol, mic
choice, sensor placement — each with the reasoning that fixed it. **Do not re-litigate them.**
In particular, do not propose AGC, spectral subtraction, noise reduction that rewrites levels,
or an autoscaling display axis. All four have been ruled out on the same grounds, and proposing
one signals you have not read the constraint.

Note that CLAUDE.md is scoped to the ESP8266 rig and predates the ESP32 work; where it and this
README disagree about the ESP32, this README was written by reading the files and CLAUDE.md was
not updated. The four known discrepancies are listed at the top.

**Verify file versions by grepping for markers before assuming a script is current.** This
project has been bitten by stale duplicate files more than once — there are two diverged copies
of `pcg_listen.py` right now, and two files reference a `pcg_esp32.ino` that does not exist.
Check a marker, not a filename:

```bash
grep -c "no sync word found" pcg_listen.py        # 1 = current copy
grep -n "i2s_get_clk" analog_mic_esp32.ino        # present = has the rate check
grep -n "RATE MISMATCH" analog_mic_record.py      # present = warns on rate drift
grep -n "^#define STREAM" analog_mic_esp32.ino    # 0 = text banner, 1 = binary
```

**Always report verbatim command output, not summaries.** Compile it, flash it, read the actual
serial output. Never say something works because the code looks correct — the two worst bugs on
this rig, the clock clamp and the wire-format mismatch, both produced code that looked right and
output that looked plausible.

**Keep the fail-loud property.** Any code path that can stall must print something before it can
block and must be bounded by a timeout. Silence should mean "crashed", never "still starting up".

**Small diffs. One change, one test, one result.** Do not add features — no WiFi, no web UI, no
SD card, no BLE, no second microphone — until a clean S1/S2 trace exists. If something seems
like a good idea, say so and stop.

---

## Repo status

This directory is **not currently a git repository**; there is no `.git` anywhere in the tree.
The intended remote is <https://github.com/MusaabAlt/pcg-patch>. To initialise:

```bash
cd "C:/Users/HP/Documents/Arduino/EKG"
git init
git add .
git commit -m "Bench bring-up: ESP8266 digital mic and ESP32 analog mic rigs"
git remote add origin https://github.com/MusaabAlt/pcg-patch.git
```

Consider a `.gitignore` for `__pycache__/`, `.idea/`, `*.wav` and `*.png` — recordings are large
and regenerable, and one already exceeds 1.9 MB.

# CLAUDE.md — PCG patch, ESP8266 bench rig

## What this is

A wearable phonocardiography (PCG) prototype. A MEMS microphone in a sealed
cavity is pressed to the chest; the goal right now is to see repeating S1/S2
heart-sound pairs at 60–90 bpm in a recording. Nothing else.

This is bench bring-up, not a product. The product build uses different parts
(nRF5340, IM69D130, IIS3DWB) and none of its constraints transfer here.

## Hardware actually on the desk

| | |
|---|---|
| Board | NodeMCU V3 "LoLin", ESP8266MOD (ESP-12E), CH340 USB-serial |
| Port | COM14, Windows, Python 3.10 |
| Mic A | sold as "I2S MIC", exact part unconfirmed |
| Mic B | sold as INMP441, chip marked `CU21 2973`, identity **unconfirmed** |
| Coupler | stethoscope bell or bottle cap, sealed air cavity |

Only one mic is wired at a time right now.

## ESP8266 facts that constrain everything

- **I2S input pins are fixed in silicon.** No pin matrix. They cannot be moved.
  - `GPIO12` = D6 = I2SI_DATA -> mic SD
  - `GPIO13` = D7 = I2SI_BCK  -> mic SCK
  - `GPIO14` = D5 = I2SI_WS   -> mic WS
  - mic L/R -> GND, VDD -> 3V3
- **Never touch GPIO6-11** (`S0 S1 S2 S3 SC SK` on the left header). Flash bus.
- **I2S capture is 16-bit only.** The mic sends 24. The bottom 8 bits do not
  exist on this chip. A known, accepted limitation of the bench rig.
- **Core 3.x renamed the header** to `core_esp8266_i2s.h`. Core 2.x used
  `i2s.h`. The sketch picks between them with `__has_include`. Do not
  "simplify" that block.
- **WiFi must stay off** (`WiFi.mode(WIFI_OFF)`). It starves the I2S DMA.
- **CPU must run at 160 MHz.** At 80 MHz samples drop.
- Serial for data is **921600**. The ROM boot log is **74880** — set the
  monitor there to see resets and crash dumps.

## Signal chain, frozen

- Capture at **16 kHz** on the wire. BCLK = fs x 32 = 512 kHz, just above the
  INMP441's 0.5 MHz minimum. If data looks wrong, **24000** is the fallback
  (768 kHz BCLK); the Python side then needs `--fs-in 24000`.
- **Fixed gain. No AGC, ever.** The amplitude ratio between S1, S2 and any
  murmur is the diagnostic information. Anything that normalises level
  destroys it. Not negotiable, not a performance tradeoff.
- **Lossless only.** No lossy codec anywhere in the record path.
- Band-pass 20-400 Hz for display and listening. Decimate to 4 kHz for storage.
- Placement: 4th intercostal space, left sternal border. Not the apex — the
  apex beat moves with posture and a fixed sensor cannot follow it.

## Files

| File | Role |
|---|---|
| `pcg_esp8266.ino` | firmware. Two modes via `#define STREAM`: 0 = level meter (text), 1 = binary stream |
| `pcg_listen.py` | live monitor: serial -> band-pass -> headphones, level bar + BPM |
| `pcg_record.py` | records N seconds -> WAV files + diagnostic PNG |

Wire protocol, ESP8266 -> PC:

```
A5 5A A5 5A | uint16 seq (LE) | uint16 count (LE) | count x int16 (LE)
```

`seq` increments once per block and wraps at 65536. The receivers check it and
report `dropped N`. A gap means blocks were lost in transport, which appears in
the waveform as a step discontinuity — a broadband click. Without `seq` a
transport click is indistinguishable from a real acoustic one.

int16, not int32 — the ESP8266 has no more bits to give, and halving the byte
rate to 32 kB/s keeps the UART clear of dropouts. Python needs `--wire16`.
The ESP32 variant of this sketch sends int32 and omits the flag.

## Current state and the open question

I2S works. The mic responds to sound on the left slot. That is settled.

**Open: a 30 s "silence" recording came back at -28.5 dBFS RMS, peak -10.2.**
Back-calculated through the INMP441 sensitivity that is about 91 dB SPL, which
no desk produces. The energy is concentrated in discrete transients every 1-2 s,
not in a continuous floor: peak -10.2 at roughly 1% duty cycle predicts
-30.2 dBFS RMS, within 2 dB of what was measured. So the transients are the
entire reading and the real noise floor has not been seen yet.

Three candidates, not yet separated:
1. Real impulsive sound in the room.
2. Something physically touching the board or wires.
3. Blocks lost in transport, spliced back together into step discontinuities.

Candidate 3 is now detectable via `seq`. Candidates 1 and 2 separate with a
muffled recording. Nothing downstream is worth touching until this is resolved.

## How to work

- **Verify before claiming.** Compile it, flash it, read the serial output.
  Never say something works because the code looks correct.
- **Quote exact output.** Paste the real serial lines, not a summary.
- **Do not add features.** No WiFi, no web UI, no SD card, no battery, no BLE,
  no AI, no second microphone, until a clean S1/S2 trace exists. If something
  seems like a good idea, say so and stop.
- **Do not change these without asking:** sample rate, gain strategy, filter
  bands, wire protocol, mic choice, sensor placement. Those are decided.
- **Keep the fail-loud property.** Any code path that can stall must print
  something first and must be bounded by a timeout. Silence should only ever
  mean "crashed", never "still starting up".
- Small diffs. One change, one test, one result.

## Environment — verified 2026-09-01, do not reinstall, just confirm

| | |
|---|---|
| arduino-cli | v1.5.1, bundled in the IDE, **not on PATH**: `C:\Users\HP\AppData\Local\Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe` |
| core | `esp8266:esp8266` 3.1.2, already installed |
| FQBN | `esp8266:esp8266:nodemcuv2:xtal=160,eesz=4M2M,baud=115200` |
| Python | 3.10, `C:\Users\HP\AppData\Local\Programs\Python\Python310` |
| deps | pyserial, numpy, scipy, matplotlib, sounddevice |
| port | COM14 |

## Measured facts

- **True wire rate is 16384 Hz, not 16000.** The ESP8266 clock divider cannot
  hit 16000. `warm-up: 16384 samples in 1 s` confirmed it. The receivers
  decimate by the nearest whole factor, so 16384 -> 4096 Hz output. Harmless.
- **The mic is on the LEFT slot.** `MIC_SLOT 0` is correct.
- **The RIGHT slot is a floating line, not audio.** It sits at a constant
  -4.7 dBFS. A uniformly random 16-bit value has RMS 32768/sqrt(3), which is
  -4.77 dBFS — an exact match. Expected, since L/R tied to GND makes the mic
  tri-state during the right slot. Ignore it.

## Toolchain

Prefer `arduino-cli` over the IDE — it lets you compile, upload and read the
serial port without a human clicking anything.

```
arduino-cli core install esp8266:esp8266
arduino-cli compile --fqbn esp8266:esp8266:nodemcuv2:xtal=160 .
arduino-cli upload  --fqbn esp8266:esp8266:nodemcuv2:xtal=160 -p COM14 .
arduino-cli monitor -p COM14 -c baudrate=921600
```

If `arduino-cli` is missing, install it first; it is worth the five minutes.

Only one program can hold COM14. Close the monitor before running the Python
scripts, and vice versa.

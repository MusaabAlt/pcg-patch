/*
 * pcg_esp8266.ino  —  I2S MEMS mic -> raw PCM over USB serial, ESP8266 version
 *
 * READ THIS FIRST
 *   The ESP8266 I2S peripheral captures 16 bits per sample. The mic sends 24.
 *   You get the top 16 bits and lose the bottom 8. For loud sound that is fine.
 *   For heart sounds it costs you roughly 8 bits of the dynamic range you need.
 *   Use an ESP32 if you have one. This file exists so you are not blocked.
 *
 * WIRING — these pins are FIXED on the ESP8266. They cannot be changed.
 * On a NodeMCU V3 they are five consecutive pins on the RIGHT header.
 *
 *   MIC             GPIO      NodeMCU V3 pin
 *   VDD     -->     -         3V     (the one just below D4)
 *   GND     -->     -         G      (just below that 3V)
 *   L/R     -->     GND       G      (SEL on an SPH0645 board — link it to the
 *                                     mic module's own GND pad, they are adjacent)
 *   WS      -->     GPIO14    D5     (I2SI_WS)
 *   SD      -->     GPIO12    D6     (I2SI_DATA)
 *   SCK     -->     GPIO13    D7     (I2SI_BCK)
 *
 *   Never touch S0 S1 S2 S3 SC SK on the left header. Those are GPIO6-11,
 *   wired to the flash chip. Shorting them bricks the boot.
 *
 * BOARD SETTINGS
 *   Tools > Board          ->  NodeMCU 1.0 (ESP-12E Module)
 *   Tools > CPU Frequency  ->  160 MHz     (80 MHz will drop samples)
 *   Tools > Flash Size     ->  4MB (FS:2MB OTA:~1019KB)
 *   Tools > Upload Speed   ->  115200 if uploads fail at higher rates
 *   Windows: install the CH340 driver first or no port appears.
 *
 * TWO MODES — change STREAM below:
 *   STREAM 0 : RMS level meter for both slots. Run this first.
 *   STREAM 1 : binary stream. Run the receiver with:
 *                  python pcg_record.py --port COMx --wire16 --seconds 30 --out chest01
 */

#include <Arduino.h>
#include <ESP8266WiFi.h>

// The I2S header was renamed between core versions:
//   core 2.x -> i2s.h
//   core 3.x -> core_esp8266_i2s.h
// Both declare the same functions, so pick whichever is present.
#if __has_include(<core_esp8266_i2s.h>)
  #include <core_esp8266_i2s.h>
#else
  #include <i2s.h>
#endif

// ---------------- config ----------------
#define SAMPLE_RATE   16000   // BCLK = fs x 32 = 512 kHz, just above the mic's
                              // 0.5 MHz minimum. If data looks wrong, try 24000
                              // and run Python with --fs-in 24000.
#define BAUD          921600

#define STREAM         1      // 0 = level meter, 1 = binary stream
#define MIC_SLOT       0      // 0 = left (L/R to GND), 1 = right (L/R to 3V3)
#define TX_CLOCK       0      // fallback: if both slots read exactly 0, set to 1.
                              // Some core versions only generate I2S clocks when
                              // the TX side is enabled. Costs you GPIO2/3/15
                              // (D4, D8, serial RX). Serial TX still works.
// ----------------------------------------

#define FRAME 128
static int16_t out[FRAME];

// Read up to n samples without ever blocking indefinitely.
// Returns how many it actually got.
static int readSamples(int16_t *l, int16_t *r, int n, uint32_t ms) {
  int k = 0;
  uint32_t t0 = millis();
  while (k < n && (millis() - t0) < ms) {
    if (i2s_read_sample(&l[k], &r[k], false)) k++;
    else yield();
  }
  return k;
}

void setup() {
  Serial.begin(BAUD);
  delay(300);

  // Say something BEFORE anything can hang, so silence always means a hang
  // and never means "still starting up".
  Serial.println();
  Serial.println("=== pcg_esp8266 ===");
  Serial.printf("mode: %s | fs: %d Hz | slot: %s | TX_CLOCK: %d\n",
                STREAM ? "STREAM" : "METER", SAMPLE_RATE,
                MIC_SLOT ? "RIGHT" : "LEFT", TX_CLOCK);

  // WiFi steals CPU and starves the I2S DMA. Kill it.
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
  delay(10);

  if (!i2s_rxtx_begin(true, TX_CLOCK ? true : false)) {
    while (1) { Serial.println("i2s_rxtx_begin FAILED"); delay(1000); yield(); }
  }
  i2s_set_rate(SAMPLE_RATE);

  // Warm-up: the mic emits zeros for ~2^18 clocks after power-up. Discard those,
  // and at the same time measure whether I2S is delivering anything at all.
  static int16_t wl[256], wr[256];
  uint32_t got = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < 1000) got += readSamples(wl, wr, 256, 50);

  Serial.printf("warm-up: %u samples in 1 s (expect ~%d)\n", got, SAMPLE_RATE);
  if (got == 0) {
    Serial.println("*** NO I2S DATA ***");
    Serial.println("  check  SD->D6   SCK->D7   WS->D5   L/R->G   VDD->3V   GND->G");
    Serial.println("  if the wiring is right, set TX_CLOCK 1 and reflash");
  } else if (got < SAMPLE_RATE / 2) {
    Serial.println("  (low sample count - clock may be misconfigured)");
  }

#if STREAM == 0
  Serial.println("LEVEL METER - the slot with the live mic moves when you talk.");
  Serial.println("Set MIC_SLOT to that slot, then STREAM 1, reflash.");
#endif
}

#if STREAM == 0
// ---------------- level meter ----------------
void bar(double rms) {
  double db = (rms > 0.5) ? 20.0 * log10(rms / 32768.0) : -110.0;
  char buf[16];
  snprintf(buf, sizeof(buf), "%7.1f dBFS ", db);
  Serial.print(buf);
  int n = (int)((db + 100.0) / 4.0);
  if (n < 0) n = 0; if (n > 25) n = 25;
  for (int i = 0; i < n; i++) Serial.print('#');
}

void loop() {
  static double accL = 0, accR = 0;
  static uint32_t cnt = 0, last = 0;
  static int16_t bl[FRAME], br[FRAME];

  int n = readSamples(bl, br, FRAME, 100);
  for (int i = 0; i < n; i++) {
    accL += (double)bl[i] * bl[i];  accR += (double)br[i] * br[i];  cnt++;
  }

  if (millis() - last > 250 && cnt == 0) {
    Serial.println("no samples - I2S has stopped delivering");
    last = millis();
  }

  if (millis() - last > 250 && cnt) {
    Serial.print("L "); bar(sqrt(accL / cnt));
    Serial.print("   R "); bar(sqrt(accR / cnt));
    Serial.println();
    accL = accR = 0; cnt = 0; last = millis();
    yield();
  }
}

#else
// ---------------- binary stream ----------------
// Block format:  A5 5A A5 5A | uint16 seq | uint16 count | count x int16 LE
//
// seq increments once per block and wraps at 65536. The receiver checks it.
// A gap means blocks were lost in transport, which shows up in the waveform
// as a step discontinuity - i.e. a broadband click. Without seq you cannot
// tell a transport click apart from a real acoustic one.
// NOTE: int16 here, not int32 like the ESP32 sketch. The ESP8266 only has
// 16 bits to give, and halving the byte rate (32 kB/s instead of 64 kB/s)
// keeps the UART well clear of dropouts.
//   -> run the receiver with:  --wire16 --fs-in 16000
void loop() {
  static int16_t bl[FRAME], br[FRAME];
  int n = readSamples(bl, br, FRAME, 100);
  if (n == 0) { yield(); return; }              // nothing to send, do not stall

  for (int i = 0; i < n; i++) out[i] = MIC_SLOT ? br[i] : bl[i];

  static const uint8_t magic[4] = {0xA5, 0x5A, 0xA5, 0x5A};
  static uint16_t seq = 0;
  uint16_t cnt = (uint16_t)n;
  Serial.write(magic, 4);
  Serial.write((uint8_t*)&seq, 2);
  Serial.write((uint8_t*)&cnt, 2);
  Serial.write((uint8_t*)out, n * sizeof(int16_t));
  seq++;
  yield();
}
#endif

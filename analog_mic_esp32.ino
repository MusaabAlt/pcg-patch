/*
 * analog_mic_esp32.ino — 3-pin ANALOG microphone (VCC / GND / OUT) on ESP32
 *
 * Separate project. Does not touch pcg_esp32.ino or pcg_esp8266.ino.
 *
 * WIRING
 *   MIC MODULE        ESP32
 *   VCC       -->     3V3
 *   GND       -->     GND
 *   OUT       -->     GPIO 35        (ADC1 channel 7, input-only pin)
 *
 *   GPIO 34 is left free on purpose — that is where the ECG goes later.
 *
 * HOW IT SAMPLES
 *   Not analogRead() in a loop. That jitters badly and the jitter shows up as
 *   noise. This uses the ESP32's I2S peripheral in ADC mode, which paces the
 *   converter in hardware and fills a DMA buffer. Same DMA structure as the
 *   digital mic sketches, so the timing is just as clean.
 *
 * WIRE FORMAT — deliberately identical to the ESP8266 sketch:
 *   A5 5A A5 5A | uint16 seq | uint16 count | count x int16 LE
 *
 *   That means the EXISTING tools work on this stream unmodified:
 *     python pcg_listen.py --port COMx --wire16 --fs-in 32000
 *     python analog_mic_record.py --port COMx --fs-in 32000
 *
 * *** CHECK THE CHIP ON YOUR MODULE FIRST ***
 *   MAX4466  -> correct. Manual gain trimpot, no AGC.
 *   MAX9812  -> acceptable. Fixed 20 dB, no AGC.
 *   MAX9814  -> has AGC that CANNOT be disabled. It continuously rewrites the
 *               loudness of what it hears, which destroys the S1:S2 amplitude
 *               ratio — the actual diagnostic information. Fine for listening,
 *               useless for measurement.
 *   LM393    -> that is a comparator board (KY-037/KY-038). It outputs a
 *               threshold flag, not audio. Will not work at all.
 *
 * WHY 32 kHz AND NOT 16 kHz
 *   In ADC_BUILT_IN mode the legacy I2S driver derives the converter clock
 *   through an mclk divider capped at 255. At 16 kHz the required divider is
 *   about 420, so the driver logs
 *       "sample rate is too small, the mclk division exceed the maximum 255"
 *   clamps the divider, and then RETURNS OK. The sketch cannot tell. Measured
 *   result: 16000 x 255/420 = 9714 Hz on the wire while the code believes
 *   16000. Everything downstream is then wrong by a factor of 1.64 — filter
 *   bands, PSD axes, and playback speed, which underruns the audio callback
 *   and produces gaps and crackle.
 *
 *   The minimum rate that keeps the divider under 255 is
 *       16000 x 420/255 = 26.4 kHz
 *   so 32 kHz is the first comfortable value, and it divides by 8 to the
 *   4 kHz analysis rate exactly. It also moves Nyquist to 16 kHz, which
 *   reduces how much out-of-band energy folds into the audio band.
 *
 *   Do NOT "fix" this with use_apll. The APLL path is not honoured in ADC
 *   mode; the divider limit is the actual constraint and raising the rate is
 *   what satisfies it.
 *
 *   The banner prints the rate the driver ACTUALLY produced. Trust that
 *   number, not SAMPLE_RATE, and pass it to the Python scripts with --fs-in.
 *
 * ANTI-ALIAS FILTER — add this, it costs two components
 *   The module's bandwidth runs past 20 kHz. Sampled at 16 kHz with nothing
 *   filtering the input, everything above 8 kHz folds down into the audio
 *   band as broadband hiss that no digital filter can ever remove — by the
 *   time it is sampled, it is indistinguishable from real signal.
 *   Digital MEMS mics have internal decimation filters and never show this.
 *   An analogue front end has no such protection.
 *
 *     MIC OUT ---[ 1k ohm ]---+--- GPIO 35
 *                             |
 *                          [100 nF]
 *                             |
 *                            GND
 *
 *   Corner = 1 / (2*pi*1000*100e-9) = 1.6 kHz. Well above the 600 Hz murmur
 *   band, well below the 8 kHz Nyquist limit.
 *
 * TWO MODES — change STREAM below:
 *   STREAM 0 : text meter. Shows DC bias and level. Run this first.
 *   STREAM 1 : binary stream for the Python scripts.
 */

#include <WiFi.h>
#include <driver/i2s.h>
#include <driver/adc.h>
#include <string.h>

// ---------------- config ----------------
#define ADC_CHANNEL   ADC1_CHANNEL_7   // GPIO 35. CH6 = GPIO34, CH4 = GPIO32.
#define SAMPLE_RATE   32000   // NOT 16000 — see below
#define BAUD          921600

#define STREAM         1      // 0 = meter, 1 = binary stream
#define SWAP_PAIRS     0      // set to 1 only if the meter shows nonsense (see below)
// ----------------------------------------

#define FRAME 256
static uint16_t rawADC[FRAME];
static int16_t  out[FRAME];

// The I2S-ADC path returns 16-bit words: bits 15..12 hold the channel number,
// bits 11..0 hold the 12-bit conversion. Mask before using.
static inline int adcValue(uint16_t w) { return (int)(w & 0x0FFF); }

void setup() {
  Serial.begin(BAUD);
  delay(300);
  Serial.println();
  Serial.println("=== analog_mic_esp32 ===");
  Serial.printf("mode: %s | fs: %d Hz | ADC1 ch %d\n",
                STREAM ? "STREAM" : "METER", SAMPLE_RATE, (int)ADC_CHANNEL);

  // The radio injects noise into the 3.3 V rail that feeds the microphone
  // module, and the analogue front end has no rejection against it. Kill it.
  WiFi.mode(WIFI_OFF);
  WiFi.disconnect(true);
  btStop();
  delay(10);
  Serial.println("wifi + bluetooth off");

  i2s_config_t cfg;
  memset(&cfg, 0, sizeof(cfg));
  cfg.mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN);
  cfg.sample_rate          = SAMPLE_RATE;
  cfg.bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT;   // ADC mode requires 16
  cfg.channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT;
  cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  cfg.intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1;
  cfg.dma_buf_count        = 8;
  cfg.dma_buf_len          = FRAME;
  cfg.use_apll             = false;

  if (i2s_driver_install(I2S_NUM_0, &cfg, 0, NULL) != ESP_OK) {
    while (1) { Serial.println("i2s_driver_install FAILED"); delay(1000); }
  }

  adc1_config_width(ADC_WIDTH_BIT_12);
#ifdef ADC_ATTEN_DB_12
  adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN_DB_12);   // full 0-3.3 V
#else
  adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN_DB_11);
#endif
  i2s_set_adc_mode(ADC_UNIT_1, ADC_CHANNEL);
  i2s_adc_enable(I2S_NUM_0);

  // i2s_driver_install returns OK even when the clock request was clamped,
  // so ask the driver what it actually produced and report it loudly.
  float actual = i2s_get_clk(I2S_NUM_0);
  Serial.printf("requested %d Hz | driver reports %.0f Hz\n", SAMPLE_RATE, actual);
  if (fabs(actual - SAMPLE_RATE) > SAMPLE_RATE * 0.02) {
    Serial.println("*** RATE MISMATCH - the clock request was clamped ***");
    Serial.printf("    pass --fs-in %.0f to every Python script\n", actual);
  }

  // Bounded warm-up: a wiring fault prints a diagnosis instead of hanging.
  size_t n; uint32_t got = 0; double sum = 0; uint32_t t0 = millis();
  while (millis() - t0 < 1000) {
    if (i2s_read(I2S_NUM_0, rawADC, sizeof(rawADC), &n, pdMS_TO_TICKS(50)) == ESP_OK) {
      int k = n / sizeof(uint16_t);
      for (int i = 0; i < k; i++) sum += adcValue(rawADC[i]);
      got += k;
    }
  }
  double dc = got ? sum / got : 0;
  Serial.printf("warm-up: %u samples in 1 s (expect ~%d)\n", got, SAMPLE_RATE);
  Serial.printf("DC bias: %.0f / 4095  (%.2f V)\n", dc, dc * 3.3 / 4095.0);

  if (got == 0) {
    Serial.println("*** NO DATA *** check OUT->GPIO35, VCC->3V3, GND->GND");
  } else if (dc < 200) {
    Serial.println("*** DC near 0 *** module not powered, or OUT on the wrong pin");
  } else if (dc > 3900) {
    Serial.println("*** DC near full scale *** OUT may be a digital pin, not analogue");
  } else if (dc < 1200 || dc > 2900) {
    Serial.println("  (unusual bias - most analogue mic modules idle near 2048)");
  } else {
    Serial.println("  bias looks correct: the module idles mid-rail as expected");
  }

#if STREAM == 0
  Serial.println("METER - talk at the mic, the bar should move.");
#endif
}

#if STREAM == 0
void loop() {
  static double acc = 0, dcAcc = 0; static uint32_t cnt = 0, last = 0;
  static int lo = 4095, hi = 0;
  size_t n;
  if (i2s_read(I2S_NUM_0, rawADC, sizeof(rawADC), &n, pdMS_TO_TICKS(100)) != ESP_OK) return;
  int k = n / sizeof(uint16_t);
  for (int i = 0; i < k; i++) {
    int v = adcValue(rawADC[i]);
    dcAcc += v; if (v < lo) lo = v; if (v > hi) hi = v;
    double d = v - 2048.0; acc += d * d; cnt++;
  }

  if (millis() - last > 250 && cnt) {
    double rms = sqrt(acc / cnt);
    double db  = 20.0 * log10((rms + 0.01) / 2048.0);
    Serial.printf("%7.1f dBFS  dc %4.0f  min %4d  max %4d  ",
                  db, dcAcc / cnt, lo, hi);
    int b = (int)((db + 80.0) / 3.0); if (b < 0) b = 0; if (b > 26) b = 26;
    for (int i = 0; i < b; i++) Serial.print('#');
    if (lo <= 1 || hi >= 4094) Serial.print("  CLIPPING - lower the gain");
    Serial.println();
    acc = dcAcc = 0; cnt = 0; lo = 4095; hi = 0; last = millis();
  }
}
#else
void loop() {
  size_t n;
  if (i2s_read(I2S_NUM_0, rawADC, sizeof(rawADC), &n, pdMS_TO_TICKS(100)) != ESP_OK
      || n == 0) return;
  int k = n / sizeof(uint16_t);

#if SWAP_PAIRS
  // Some builds deliver the two 16-bit halves of each DMA word in reverse.
  // Only enable this if the meter shows a sensible DC bias but a recorded
  // pure tone comes back distorted or at the wrong frequency.
  for (int i = 0; i + 1 < k; i += 2) {
    uint16_t t = rawADC[i]; rawADC[i] = rawADC[i + 1]; rawADC[i + 1] = t;
  }
#endif

  // Centre on the mid-rail bias and scale 12 bits up to 16 so that the
  // existing --wire16 tools see full scale as full scale. Removing the DC
  // offset is not gain control: it is a fixed subtraction, identical every
  // sample, and it preserves every amplitude ratio in the signal.
  for (int i = 0; i < k; i++) out[i] = (int16_t)((adcValue(rawADC[i]) - 2048) << 4);
 
  static const uint8_t magic[4] = {0xA5, 0x5A, 0xA5, 0x5A};
  static uint16_t seq = 0;
  uint16_t cnt = (uint16_t)k;
  Serial.write(magic, 4);
  Serial.write((uint8_t*)&seq, 2);
  Serial.write((uint8_t*)&cnt, 2);
  Serial.write((uint8_t*)out, k * sizeof(int16_t));
  seq++;
}
#endif

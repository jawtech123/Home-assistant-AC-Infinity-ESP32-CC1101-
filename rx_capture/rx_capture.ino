/**
 * rx_capture -- raw OOK envelope capture for the AC Infinity CTR63A remote.
 *
 * Puts the CC1101 in asynchronous direct mode: the demodulated OOK envelope
 * appears directly on GDO0, with no packet engine, sync word or clock recovery
 * in the way. The ESP32 timestamps every edge and reconstructs the pulse train.
 *
 * Wiring (confirmed):
 *   CC1101 GDO0 -> GPIO4      CSN  -> GPIO5
 *          SCK  -> GPIO18     MISO -> GPIO19     MOSI -> GPIO23
 *   GDO2 is not connected (async mode only needs GDO0).
 *
 * Serial @ 115200. Commands:
 *   r  toggle raw pulse-train output
 *   +  increase AGC gain limit (more sensitive, more noise)
 *   -  decrease AGC gain limit (less sensitive, quieter)
 *   ?  print current settings
 */

// Exposes SPIsetRegValue() so the AGC registers can be tuned. RadioLib's
// documented escape hatch; must be defined before the include.
#define RADIOLIB_LOW_LEVEL 1
#include <RadioLib.h>

// --- Pins --------------------------------------------------------------------

#define PIN_GDO0  4
#define PIN_CSN   5
#define PIN_SCK   18
#define PIN_MISO  19
#define PIN_MOSI  23

// Module(cs, irq, rst, gpio) -- for the CC1101, irq is GDO0 and gpio is GDO2.
CC1101 radio = new Module(PIN_CSN, PIN_GDO0, RADIOLIB_NC, RADIOLIB_NC);

// --- Radio configuration -----------------------------------------------------

static const float FREQ_MHZ   = 433.92;
static const float BITRATE_KBPS = 4.8;   // sets the demod filter, not a symbol rate
/* 325 kHz matches the configuration that successfully captured this remote on
   2026-07-22. A cheap remote's crystal can sit 100-200 kHz off nominal, so a
   narrower filter risks missing it entirely. Must be one of the CC1101's
   discrete steps: 58.04 67.71 81.25 101.56 116.07 135.42 162.5 203.13 232.14
   270.83 325 406.25 464.29 541.67 650 812.5. */
static float rxBwKhz = 325.0;
static const int8_t TX_POWER  = 10;      // unused here, but keeps PA_TABLE sane

#define CC1101_REG_AGCCTRL2  0x1B
#define CC1101_REG_AGCCTRL1  0x1C
#define CC1101_REG_AGCCTRL0  0x1D

/* An OOK receiver with no signal present will crank the AGC until the noise
   floor crosses the slicer threshold, producing a continuous edge storm. The
   fix is to cap the digital gain: AGCCTRL2 bits 7:6 (MAX_DVGA_GAIN) reduce the
   maximum gain by that many steps. 3 = quietest, 0 = most sensitive. This is
   the main tuning knob and is adjustable at runtime with '+' / '-'. */
static uint8_t maxDvgaGain = 3;

#define AGCCTRL2_VALUE(dvga)  (((dvga) << 6) | 0x07)  // MAX_LNA_GAIN=0, MAGN_TARGET=42dB
static const uint8_t AGCCTRL1_VALUE = 0x00;
static const uint8_t AGCCTRL0_VALUE = 0x91;

// --- Protocol ----------------------------------------------------------------

static const uint16_t MARK_US    = 535;
static const uint16_t SPACE_0_US = 515;
static const uint16_t SPACE_1_US = 1067;
static const uint16_t SPACE_THRESHOLD_US = (SPACE_0_US + SPACE_1_US) / 2;

static const uint8_t FRAME_BITS = 16;
static const uint8_t SPEED_MAX  = 10;

// A pulse counts as a symbol if within this fraction of nominal.
static const float TOLERANCE = 0.45;

// Checksum: GF(2^8) multiply by 0x43 (poly 0x131) for speeds 1-9; 0 and 10 are
// exceptions. See protocol.md.
static uint8_t gfMul(uint8_t a, uint8_t b) {
  uint16_t wide = a;
  uint8_t result = 0;
  while (b) {
    if (b & 1) result ^= (uint8_t)wide;
    b >>= 1;
    wide <<= 1;
    if (wide & 0x100) wide ^= 0x131;
  }
  return result;
}

static uint8_t speedChecksum(uint8_t speed) {
  if (speed == 0)  return 0xAC;
  if (speed == 10) return 0x82;
  return gfMul(0x43, speed);
}

static void speedName(uint8_t speed, char *out, size_t len) {
  if (speed == 0)       snprintf(out, len, "OFF");
  else if (speed == 10) snprintf(out, len, "MAX");
  else                  snprintf(out, len, "%u", speed);
}

// --- Edge capture ------------------------------------------------------------

#define PULSE_BUF_LEN 1024

/* Signed microseconds: positive = mark (carrier on), negative = space. */
static volatile int32_t  pulseBuf[PULSE_BUF_LEN];
static volatile uint16_t pulseCount = 0;
static volatile uint32_t lastEdgeUs = 0;
static volatile bool     bufOverflow = false;

// A transmission is 8 frames of 16 bits, so ~270 pulses. Require enough for at
// least one frame before bothering to decode.
static const uint16_t MIN_PULSES = FRAME_BITS * 2;
static const uint32_t IDLE_GAP_US = 5000;

/* Shortest real element is ~515us; observed noise spikes are 26-156us. Edges
   closer together than this are treated as glitches and dropped, which lets the
   receiver run at useful sensitivity without the noise floor filling the buffer. */
static const uint32_t MIN_PULSE_US = 250;

// Counters so the heartbeat can distinguish "receiver is deaf" from "signal
// arriving but not decoding" from "drowning in noise".
static volatile uint32_t totalEdges = 0;
static volatile uint32_t glitchEdges = 0;

IRAM_ATTR static void onEdge() {
  uint32_t now = micros();
  uint32_t duration = now - lastEdgeUs;
  totalEdges++;

  /* Ignore the edge outright rather than recording a short pulse: lastEdgeUs is
     left alone so the current pulse keeps accumulating, which is what makes a
     narrow spike vanish instead of splitting one real pulse into three. */
  if (duration < MIN_PULSE_US) {
    glitchEdges++;
    return;
  }

  lastEdgeUs = now;

  // The level that just ended is the opposite of the level now present.
  bool markEnded = (digitalRead(PIN_GDO0) == LOW);

  if (duration > 0x7FFFFFFF) duration = 0x7FFFFFFF;

  if (pulseCount < PULSE_BUF_LEN) {
    pulseBuf[pulseCount++] = markEnded ? (int32_t)duration : -(int32_t)duration;
  } else {
    bufOverflow = true;
  }
}

static void resetCapture() {
  noInterrupts();
  pulseCount = 0;
  bufOverflow = false;
  interrupts();
}

// --- Decoding ----------------------------------------------------------------

static bool near(uint32_t value, uint32_t nominal) {
  uint32_t slack = (uint32_t)(nominal * TOLERANCE);
  return value + slack >= nominal && value <= nominal + slack;
}

/* Walk mark/space pairs, emitting a frame every 16 bits. Anything that is not a
   valid pair (sync burst, inter-frame gap, noise) resets the bit accumulator. */
static int decodeFrames(const int32_t *pulses, uint16_t count,
                        uint16_t *frames, int maxFrames) {
  int frameCount = 0;
  uint16_t bits = 0;
  uint8_t bitCount = 0;

  uint16_t i = 0;
  while (i + 1 < count) {
    int32_t mark = pulses[i];
    int32_t space = pulses[i + 1];

    bool isBit = mark > 0 && space < 0 && near(mark, MARK_US) &&
                 (near(-space, SPACE_0_US) || near(-space, SPACE_1_US));

    if (isBit) {
      bits = (bits << 1) | ((-space > SPACE_THRESHOLD_US) ? 1 : 0);
      if (++bitCount == FRAME_BITS) {
        if (frameCount < maxFrames) frames[frameCount++] = bits;
        bits = 0;
        bitCount = 0;
      }
      i += 2;
    } else {
      bits = 0;
      bitCount = 0;
      i += 1;
    }
  }
  return frameCount;
}

// --- Reporting ---------------------------------------------------------------

static bool rawOutput = true;

static void reportStructure(const int32_t *pulses, uint16_t count) {
  // Long elements carry the sync/gap structure we need for the transmitter.
  uint32_t longMarkMin = 0xFFFFFFFF, longMarkMax = 0, longMarkSum = 0;
  uint32_t longGapMin = 0xFFFFFFFF, longGapMax = 0, longGapSum = 0;
  uint16_t longMarks = 0, longGaps = 0;

  for (uint16_t i = 0; i < count; i++) {
    int32_t p = pulses[i];
    uint32_t mag = p < 0 ? -p : p;
    if (mag < MARK_US * 2) continue;
    if (p > 0) {
      longMarks++; longMarkSum += mag;
      if (mag < longMarkMin) longMarkMin = mag;
      if (mag > longMarkMax) longMarkMax = mag;
    } else if (mag > SPACE_1_US * 2) {
      longGaps++; longGapSum += mag;
      if (mag < longGapMin) longGapMin = mag;
      if (mag > longGapMax) longGapMax = mag;
    }
  }

  if (longMarks) {
    Serial.printf("  sync marks: n=%u avg=%lu min=%lu max=%lu us\n",
                  longMarks, (unsigned long)(longMarkSum / longMarks),
                  (unsigned long)longMarkMin, (unsigned long)longMarkMax);
  }
  if (longGaps) {
    Serial.printf("  gaps:       n=%u avg=%lu min=%lu max=%lu us\n",
                  longGaps, (unsigned long)(longGapSum / longGaps),
                  (unsigned long)longGapMin, (unsigned long)longGapMax);
  }
}

static void reportFrames(const uint16_t *frames, int count) {
  // Frames repeat; collapse to distinct payloads with counts.
  uint16_t seen[16];
  uint8_t seenCount[16];
  int distinct = 0;

  for (int i = 0; i < count; i++) {
    int found = -1;
    for (int j = 0; j < distinct; j++) {
      if (seen[j] == frames[i]) { found = j; break; }
    }
    if (found >= 0) {
      seenCount[found]++;
    } else if (distinct < 16) {
      seen[distinct] = frames[i];
      seenCount[distinct] = 1;
      distinct++;
    }
  }

  for (int i = 0; i < distinct; i++) {
    uint16_t payload = seen[i];
    uint8_t addr  = (payload >> 12) & 0x0F;
    uint8_t speed = (payload >> 8) & 0x0F;
    uint8_t csum  = payload & 0xFF;

    char name[8];
    if (speed <= SPEED_MAX) speedName(speed, name, sizeof(name));
    else                    snprintf(name, sizeof(name), "?%u", speed);

    Serial.printf("  CODE=0x%04X speed=%s reps=%u", payload, name, seenCount[i]);
    if (addr) Serial.printf(" addr=%u", addr);
    if (speed <= SPEED_MAX && csum != speedChecksum(speed)) {
      Serial.printf("  BAD-CHECKSUM(got 0x%02X want 0x%02X)", csum, speedChecksum(speed));
    }
    Serial.println();
  }
}

// --- AGC ---------------------------------------------------------------------

static void applyAgc() {
  radio.SPIsetRegValue(CC1101_REG_AGCCTRL2, AGCCTRL2_VALUE(maxDvgaGain));
  radio.SPIsetRegValue(CC1101_REG_AGCCTRL1, AGCCTRL1_VALUE);
  radio.SPIsetRegValue(CC1101_REG_AGCCTRL0, AGCCTRL0_VALUE);
}

static void printSettings() {
  Serial.printf("[cfg] %.2f MHz OOK, rxBW %.1f kHz, MAX_DVGA_GAIN=%u, raw=%s\n",
                FREQ_MHZ, rxBwKhz, maxDvgaGain, rawOutput ? "on" : "off");
}

// --- Chip probe --------------------------------------------------------------

/* RadioLib's begin() reads the ID registers WITHOUT resetting the chip first,
   unlike mfurga/cc1101 which does a hardReset(). A CC1101 that has not had a
   power-on reset does not answer reliably, so begin() returns
   RADIOLIB_ERR_CHIP_NOT_FOUND even on perfectly good wiring. Initialising the
   module and issuing the manual reset sequence first fixes that.

   Note: do NOT probe SCK/MOSI by floating them. They are chip inputs, so a
   floating SCK lets noise clock the SPI shift register and corrupts chip state. */
static int beginRadio() {
  radio.getMod()->init();     // configure CS + SPI so reset() can talk to the chip
  radio.reset();              // manual power-on reset + SRES
  delay(10);

  int state = radio.begin(FREQ_MHZ, BITRATE_KBPS, 0.0, rxBwKhz, TX_POWER, 16);

  Serial.printf("[probe] VERSION=0x%02X (expect 0x04 or 0x14)\n", radio.getChipVersion());
  return state;
}

// --- Setup / loop ------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("\n=== AC Infinity rx_capture (async OOK) ===");

  int state = beginRadio();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("CC1101 begin() failed: %d\n", state);
    if (state == RADIOLIB_ERR_CHIP_NOT_FOUND) {
      Serial.println("  RadioLib accepts VERSION 0x04/0x14/0x17 only. 0x00 or 0xFF means");
      Serial.println("  SPI reads are not getting through at all.");
    }
    while (true) delay(1000);
  }

  state = radio.setOOK(true);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("setOOK failed: %d\n", state);
    while (true) delay(1000);
  }

  applyAgc();

  // Async direct mode: GDO0 becomes the raw demodulated data output.
  state = radio.receiveDirectAsync();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("receiveDirectAsync failed: %d\n", state);
    while (true) delay(1000);
  }

  /* Confirm the chip really is in async mode rather than trusting the call:
     IOCFG0 should be 0x0D (serial data out) and PKTCTRL0 0x32 (async, infinite
     length). If GDO0 stays silent, this is the first thing to check. */
  Serial.printf("[cfg] IOCFG0=0x%02X (expect 0x0D)  PKTCTRL0=0x%02X (expect 0x32)  MDMCFG2=0x%02X\n",
                radio.SPIgetRegValue(0x02), radio.SPIgetRegValue(0x08),
                radio.SPIgetRegValue(0x12));

  pinMode(PIN_GDO0, INPUT);
  lastEdgeUs = micros();
  radio.setGdo0Action(onEdge, CHANGE);

  printSettings();
  Serial.println("Ready -- press the remote button.\n");
}

static void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'r':
        rawOutput = !rawOutput;
        printSettings();
        break;
      case '+':
        if (maxDvgaGain > 0) maxDvgaGain--;
        applyAgc();
        printSettings();
        break;
      case '-':
        if (maxDvgaGain < 3) maxDvgaGain++;
        applyAgc();
        printSettings();
        break;
      case '?':
        printSettings();
        break;
      default:
        break;
    }
  }
}

/* Do NOT use radio.getRSSI() here. It branches on directModeEnabled, which
   receiveDirectAsync() sets to *false* (that flag means synchronous direct mode),
   so it returns a cached packet-mode rawRSSI that async mode never populates --
   a constant -74.0 dBm regardless of what is on the air. Read the status
   register directly instead. SPIgetRegValue adds the status-register access bit
   for us. */
static int16_t readRssiDbm() {
  uint8_t raw = (uint8_t)radio.SPIgetRegValue(RADIOLIB_CC1101_REG_RSSI);
  int16_t signedRaw = (raw >= 128) ? ((int16_t)raw - 256) : (int16_t)raw;
  return (signedRaw / 2) - 74;
}

/* A transmission lasts only ~250ms, so sampling once per heartbeat misses it
   ~95% of the time. Poll continuously and report the peak instead: that answers
   "is any RF actually arriving?" independently of whether it decodes. */
static int16_t peakRssi = -200;

/* Peak within the current pulse train, so a burst can be reported with the
   signal strength it actually arrived at rather than the level at the moment we
   got round to processing it. */
static int16_t trainPeakRssi = -200;

// Above this, a burst is treated as a real transmission worth dumping in full.
static const int16_t STRONG_RSSI_DBM = -65;

static void pollRssi() {
  int16_t rssi = readRssiDbm();
  if (rssi > peakRssi) peakRssi = rssi;
  if (rssi > trainPeakRssi) trainPeakRssi = rssi;
}

/* Periodic sign of life. Without this the sketch is silent whenever nothing
   decodes, which makes "no signal reaching GDO0" look identical to "nobody
   pressed the button". */
static void heartbeat() {
  static uint32_t lastBeatMs = 0;
  static uint32_t lastEdgeTotal = 0;

  if (millis() - lastBeatMs < 5000) return;
  lastBeatMs = millis();

  uint32_t edges = totalEdges;
  Serial.printf("[idle] edges=%lu (+%lu in 5s, %lu glitches) rssi_now=%d peak=%d dBm buffered=%u gain=%u bw=%.0fkHz\n",
                (unsigned long)edges, (unsigned long)(edges - lastEdgeTotal),
                (unsigned long)glitchEdges, (int)readRssiDbm(), (int)peakRssi,
                pulseCount, maxDvgaGain, rxBwKhz);
  lastEdgeTotal = edges;
  peakRssi = -200;   // reset for the next interval
}

void loop() {
  handleSerial();
  pollRssi();
  heartbeat();

  uint16_t count = pulseCount;
  if (count < MIN_PULSES) return;
  /* Process on idle, or when the buffer is nearly full -- otherwise a noisy
     band never goes quiet for IDLE_GAP_US and nothing is ever examined. */
  if (micros() - lastEdgeUs < IDLE_GAP_US && count < PULSE_BUF_LEN - 32) return;

  // Snapshot the buffer so the ISR can keep running against a fresh one.
  static int32_t snapshot[PULSE_BUF_LEN];
  noInterrupts();
  count = pulseCount;
  memcpy(snapshot, (const void *)pulseBuf, count * sizeof(int32_t));
  bool overflowed = bufOverflow;
  pulseCount = 0;
  bufOverflow = false;
  interrupts();

  static uint16_t frames[64];
  int frameCount = decodeFrames(snapshot, count, frames, 64);

  int16_t burstRssi = trainPeakRssi;
  trainPeakRssi = -200;

  if (frameCount == 0) {
    /* A strong burst that fails to decode is the interesting case -- dump it in
       full so the timings can be analysed offline. Weak trains are the AGC noise
       floor and only get a rate-limited sample. */
    bool strong = burstRssi > STRONG_RSSI_DBM;
    static uint32_t lastUndecodedMs = 0;
    if (count >= MIN_PULSES && (strong || millis() - lastUndecodedMs > 1000)) {
      lastUndecodedMs = millis();
      uint16_t limit = strong ? count : 20;
      Serial.printf("[nodecode%s] %u pulses, peak=%d dBm:",
                    strong ? " STRONG" : "", count, (int)burstRssi);
      for (uint16_t i = 0; i < count && i < limit; i++) {
        Serial.printf(" %+ld", (long)snapshot[i]);
      }
      Serial.println();
    }
    return;
  }

  Serial.printf("--- %u pulses, RSSI=%d dBm%s\n",
                count, (int)readRssiDbm(), overflowed ? " (BUFFER OVERFLOW)" : "");
  reportFrames(frames, frameCount);
  reportStructure(snapshot, count);

  if (rawOutput) {
    Serial.print("PULSES:");
    for (uint16_t i = 0; i < count; i++) {
      Serial.printf(" %+ld", (long)snapshot[i]);
    }
    Serial.println();
  }
  Serial.println();
}

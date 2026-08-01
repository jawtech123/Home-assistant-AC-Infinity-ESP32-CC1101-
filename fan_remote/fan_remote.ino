/**
 * fan_remote -- clone of the AC Infinity CTR63A remote (ESP32 + CC1101).
 *
 * Type a speed 0-10 over serial and the fan obeys. Confirmed working against
 * real hardware on 2026-08-01.
 *
 * MODULATION IS 2-FSK, NOT OOK. This is the single fact the whole thing hinges
 * on, and it cost most of a day to find. The remote is frequency-shift keyed:
 *   - Captured with an FSK demodulator it yields clean, decodable data.
 *   - Captured with an amplitude detector it reads as solid carrier (a whole
 *     packet of 0xFF), because FSK has a constant envelope.
 * Transmitting the same bit pattern as OOK puts energy on 433.92 MHz -- enough
 * to jam the real remote convincingly -- but the fan can never decode it. Every
 * register reads correct, the waveform matches the recording element for
 * element, and nothing happens. If this ever stops working, check modulation
 * first.
 *
 * The bit rate matters too. Unlike OOK, where the rate is just envelope
 * sampling resolution, an FSK receiver recovers its clock at a specific rate.
 * 4.8 kBaud is the rate at which a verbatim capture replay drove the fan.
 *
 * Signal (see protocol.md):
 *   433.92 MHz, 2-FSK, 4.8 kBaud, 35 kHz deviation
 *   PWM encoded: fixed mark, variable space, 16 bits MSB first, 8 repetitions
 *   mark 531.8 us | space 518.0 us = 0 | space 1066.7 us = 1
 *   each frame preceded by a 6667 us mark and a 3333 us gap
 *
 * Wiring: GDO0->GPIO4  CSN->GPIO5  SCK->GPIO18  MISO->GPIO19  MOSI->GPIO23
 *
 * Serial @ 115200:
 *   0..10    set fan speed (0 = OFF, 10 = MAX)
 *   ?        print the code table
 *   D<kHz>   change FSK deviation (default 35)
 *   a<n>     set address 0-15 (only 0 is verified; see protocol.md)
 *   m        switch the unverified address-checksum model
 *   x<n>     dump the envelope for speed n, decodable by tools/decode_capture.py
 */

#include <cc1101.h>

#define PIN_CSN   5
#define PIN_SCK   18
#define PIN_MISO  19
#define PIN_MOSI  23
#define PIN_GDO0  4

CC1101::Radio radio(PIN_CSN, PIN_SCK, PIN_MISO, PIN_MOSI, PIN_GDO0, PIN_UNUSED);

// --- Radio -------------------------------------------------------------------

static const double FREQ_MHZ     = 433.92;
static const double BITRATE_KBPS = 4.8;    // 208.33 us per sample
static const int8_t TX_POWER     = 10;
static double fskDevKhz          = 35.0;   // runtime-settable with D<kHz>

// --- Waveform (microseconds; see protocol.md) --------------------------------

/* Recovered statistically from 2112 mark samples across 132 decoded frames, so
   accurate to a few microseconds despite the reference capture being quantised
   to 208 us. */
static const double MARK_US   = 531.8;
static const double SPACE0_US = 518.0;
static const double SPACE1_US = 1066.7;
static const double SYNC_US   = 6667.0;    // long mark leading each frame
static const double GAP_US    = 3333.0;    // gap between sync and frame data

static const uint8_t FRAME_BITS  = 16;
static const uint8_t REPETITIONS = 8;
static const uint8_t SPEED_MAX   = 10;

// --- Protocol ----------------------------------------------------------------

/* Checksum is a GF(2^8) multiply of the speed by 0x43, reduction polynomial
   0x131, for speeds 1-9. Speeds 0 and 10 are exceptions. Derived from six
   observed speeds, and later confirmed against all eleven. See protocol.md. */
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

/* Address (payload bits 15-12). Every capture taken has this as 0, so 0 is the
   only value verified to work. It is called an address because that is what the
   original notes called it; it could equally be a device type or a constant
   header. */
static uint8_t fanAddress = 0;

/* How the address affects the checksum is NOT verified -- no capture contains a
   non-zero address, so both models below are conjecture and they disagree for
   every non-zero address. One capture from a remote at a different address
   would settle it. Switch models at runtime with 'm'.

   ADDR_GF: the checksum is 0x43 (x) over the whole [addr:4][speed:4] byte. This
   is the natural extension of the verified addr=0 rule, since GF multiplication
   is linear: checksum(addr,speed) = checksum(0,speed) XOR (0x43 (x) (addr<<4)).

   ADDR_LEGACY: the claim in the pre-existing ac_infinity_protocol.txt, that the
   checksum is XORed with (addr * K) mod 256 using ordinary integer arithmetic,
   K = 0xEE for speed 0, 0xE2 for speeds 1-9, 0x95 for speed 10. That document's
   checksum table turned out to be a real GF rule rather than the lookup it
   appeared to be, so its address claim is not simply dismissible -- but it was
   never verified either. */
enum AddrModel { ADDR_GF, ADDR_LEGACY };
static AddrModel addrModel = ADDR_GF;

/* --- Address handling -------------------------------------------------------
   Measured 2026-08-01 by changing the remote's DIP switches and capturing.

   Established: for a given address, ONE XOR constant applied to the addr=0
   checksum covers speeds 1-9. Speed 0 uses a different constant, and speed 10
   sometimes does too (it does at addr 7, but not at addr 4).

   For speeds 1-9 the constant is a carry-less multiply of the address by 0x31,
   confirmed for addresses 0, 1, 2, 4 and 6 (including 6 = 4 XOR 2 combining
   linearly). Address 7 is the sole exception: measured 0x2E where the rule
   gives 0x97. Addresses 3 and 5 have never been captured and are filled in from
   the rule -- likely, but unconfirmed, and address 7 proves the rule is not
   universal.

   Speed 0 follows no rule found so far and is measured-only.

   To add an address: set it on the remote, capture with rx_packet.ino, decode,
   and read the constant off the BAD-CHECKSUM message. */
static const uint8_t ADDR_KNOWN[]      = {0, 1, 2, 3, 4, 5, 6, 7};
static const uint8_t ADDR_XOR_1_9[]    = {0x00, 0x31, 0x62, 0x53, 0xC4, 0xF5, 0xA6, 0x2E};
/*                        measured:  yes   yes   yes  PRED   yes  PRED   yes   yes  */
static const uint8_t ADDR_XOR_SPEED0[] = {0x00, 0x9D, 0xCE, 0x00, 0x68, 0x00, 0x0A, 0x82};
/*  speed 0 follows NO known rule; addresses 3 and 5 are unknown, not 0 */
static const uint8_t ADDR_XOR_SPEED10[]= {0x00, 0x31, 0x62, 0x53, 0xC4, 0xF5, 0xA6, 0x13};
/*  speed 10 measured only for 0,2,4,7; 7 differs from its 1-9 value */

static bool addrXor(uint8_t addr, uint8_t speed, uint8_t *out) {
  for (uint8_t i = 0; i < sizeof(ADDR_KNOWN); i++) {
    if (ADDR_KNOWN[i] != addr) continue;
    *out = (speed == 0)  ? ADDR_XOR_SPEED0[i]
         : (speed == 10) ? ADDR_XOR_SPEED10[i]
                         : ADDR_XOR_1_9[i];
    return true;
  }
  return false;   /* address never captured -- do not invent a value */
}

static uint8_t speedChecksum(uint8_t speed, uint8_t addr) {
  uint8_t base = (speed == 0) ? 0xAC : (speed == 10) ? 0x82 : gfMul(0x43, speed);
  uint8_t x = 0;
  if (addr == 0 || !addrXor(addr, speed, &x)) return base ^ x;
  return base ^ x;
}

// Payload: [addr:4][speed:4][checksum:8].
static uint16_t buildPayload(uint8_t speed) {
  return ((uint16_t)(fanAddress & 0x0F) << 12)
       | ((uint16_t)speed << 8)
       | speedChecksum(speed, fanAddress & 0x0F);
}

static void speedName(uint8_t speed, char *out, size_t len) {
  if (speed == 0)       snprintf(out, len, "OFF");
  else if (speed == 10) snprintf(out, len, "MAX");
  else                  snprintf(out, len, "%u", speed);
}

// --- Waveform builder --------------------------------------------------------

/* All 8 repetitions fit in one 255-byte packet, so the whole burst is clocked
   out by the radio's own bit clock with no gaps: 8 * (6667 + 3333 + 16 bits)
   is about 233 ms, or 140 bytes at 4.8 kBaud. Splitting it across packets would
   insert several ms of dead air at each boundary, since every transmit() costs
   an IDLE -> flush -> fill FIFO -> PLL calibrate -> TX -> IDLE round trip. */
static uint8_t  txBuf[255];
static uint16_t txBits;
static double   txTimeUs;

/* Build in microseconds and sample at the bit period rather than rounding each
   element to whole bits: at 4.8 kBaud a 531.8 us mark is 2.55 bits, so rounding
   would shift every edge by up to 100 us and accumulate across a frame. This
   reproduces the alternating 2- and 3-sample runs the real recording shows. */
static void appendUs(bool level, double us) {
  txTimeUs += us;
  while (txBits * (1e6 / (BITRATE_KBPS * 1000.0)) < txTimeUs) {
    uint16_t byteIdx = txBits >> 3;
    if (byteIdx >= sizeof(txBuf)) return;
    if (level) txBuf[byteIdx] |= (uint8_t)(0x80 >> (txBits & 7));
    txBits++;
  }
}

static uint8_t buildBurst(uint16_t payload) {
  memset(txBuf, 0, sizeof(txBuf));
  txBits = 0;
  txTimeUs = 0.0;

  for (uint8_t rep = 0; rep < REPETITIONS; rep++) {
    appendUs(true,  SYNC_US);
    appendUs(false, GAP_US);
    for (int i = FRAME_BITS - 1; i >= 0; i--) {
      appendUs(true, MARK_US);
      appendUs(false, (payload >> i) & 1 ? SPACE1_US : SPACE0_US);
    }
  }
  return (uint8_t)((txBits + 7) / 8);
}

// --- Transmit ----------------------------------------------------------------

static bool transmitSpeed(uint8_t speed) {
  uint16_t payload = buildPayload(speed);
  uint8_t len = buildBurst(payload);

  radio.setPacketLengthMode(CC1101::PKT_LEN_MODE_FIXED, len);
  CC1101::Status st = radio.transmit(txBuf, len);

  char name[8];
  speedName(speed, name, sizeof(name));
  if (st != CC1101::STATUS_OK) {
    Serial.printf("TX speed=%s FAILED: %d\n", name, st);
    return false;
  }
  Serial.printf("TX speed=%s payload=0x%04X (%u frames, %u bytes)\n",
                name, payload, REPETITIONS, len);
  return true;
}

// --- Serial ------------------------------------------------------------------

static void printTable() {
  Serial.printf("address %u, model %s\n", fanAddress,
                addrModel == ADDR_GF ? "GF(2^8)" : "legacy");
  Serial.println("speed  payload");
  for (uint8_t s = 0; s <= SPEED_MAX; s++) {
    char name[8];
    speedName(s, name, sizeof(name));
    Serial.printf("%5s  0x%04X\n", name, buildPayload(s));
  }
}

static void handleLine(char *line) {
  while (*line == ' ' || *line == '\t') line++;
  if (!*line) return;

  if (*line == '?') { printTable(); return; }

  if (*line == 'a') {
    long a = strtol(line + 1, nullptr, 10);
    if (a < 0 || a > 15) { Serial.println("address must be 0-15"); return; }
    fanAddress = (uint8_t)a;
    Serial.printf("address = %u%s\n", fanAddress,
                  fanAddress ? "  (UNVERIFIED -- only address 0 is confirmed)" : "");
    printTable();
    return;
  }

  if (*line == 'm') {
    addrModel = (addrModel == ADDR_GF) ? ADDR_LEGACY : ADDR_GF;
    Serial.printf("address checksum model = %s\n",
                  addrModel == ADDR_GF ? "GF(2^8) over the whole byte"
                                       : "legacy (addr * K mod 256)");
    if (fanAddress) printTable();
    return;
  }

  if (*line == 'D') {
    double dev = strtod(line + 1, nullptr);
    if (dev < 1.6 || dev > 380.0) { Serial.println("deviation must be 1.6-380 kHz"); return; }
    fskDevKhz = dev;
    radio.setFrequencyDeviation(fskDevKhz);
    Serial.printf("FSK deviation = %.1f kHz\n", fskDevKhz);
    return;
  }

  /* Dump the built waveform in the same format the receiver logs, so it can be
     decoded offline by tools/decode_capture.py -- an end-to-end check that what
     we are about to transmit really carries the intended payload. */
  if (*line == 'x') {
    long sp = strtol(line + 1, nullptr, 10);
    uint8_t speed = (sp >= 0 && sp <= SPEED_MAX) ? (uint8_t)sp : SPEED_MAX;
    uint8_t len = buildBurst(buildPayload(speed));
    Serial.printf("RSSI=-50 LEN=%u DATA=", len);
    for (uint8_t i = 0; i < len; i++) Serial.printf("%02X ", txBuf[i]);
    Serial.println();
    return;
  }

  char *end = nullptr;
  long value = strtol(line, &end, 10);
  if (end == line) {
    Serial.printf("Unrecognised '%s' -- enter 0-%u, ?, D<kHz> or x<n>\n", line, SPEED_MAX);
    return;
  }
  if (value < 0 || value > SPEED_MAX) {
    Serial.printf("Speed %ld out of range (0-%u)\n", value, SPEED_MAX);
    return;
  }
  transmitSpeed((uint8_t)value);
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n=== AC Infinity fan_remote (2-FSK) ===");

  CC1101::Status status = radio.begin(CC1101::MOD_2FSK, FREQ_MHZ, BITRATE_KBPS);
  Serial.printf("[probe] PARTNUM=0x%02X VERSION=0x%02X\n",
                radio.getChipPartNumber(), radio.getChipVersion());
  if (status != CC1101::STATUS_OK) {
    Serial.printf("CC1101 begin() failed: %d -- check wiring\n", status);
    while (true) delay(1000);
  }

  // Nothing but our own waveform may go on the air.
  radio.setCrc(false);
  radio.setDataWhitening(false);
  radio.setSyncMode(CC1101::SYNC_MODE_NO_PREAMBLE);
  radio.setFrequencyDeviation(fskDevKhz);
  radio.setOutputPower(TX_POWER);

  Serial.printf("Ready -- %.2f MHz 2-FSK, %.1f kBaud, %.0f kHz dev, %d dBm.\n",
                FREQ_MHZ, BITRATE_KBPS, fskDevKhz, TX_POWER);
  Serial.println("Enter 0-10 (0 = OFF, 10 = MAX), or ? for the code table.\n");
}

void loop() {
  static char buffer[16];
  static uint8_t len = 0;

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      buffer[len] = '\0';
      handleLine(buffer);
      len = 0;
    } else if (len < sizeof(buffer) - 1) {
      buffer[len++] = c;
    } else {
      len = 0;
    }
  }
}

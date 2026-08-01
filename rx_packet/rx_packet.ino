/**
 * rx_packet -- capture the AC Infinity remote using CC1101 packet mode.
 *
 * NOTE: this build uses ASK/OOK demodulation rather than 2-FSK. An FSK
 * discriminator's output sign depends on which side of the channel the carrier
 * sits, so an FSK capture cannot tell you whether a recorded '1' means carrier
 * present or absent -- the whole envelope may be inverted. An amplitude
 * detector has no such ambiguity: '1' is carrier present, full stop. This build
 * exists to settle the polarity of the recorded envelope.
 *
 * Reproduces the configuration that successfully captured this remote on
 * 2026-07-22 (see captures/signals_2026-07-22.txt). It does not demodulate OOK
 * properly at all: it runs the 2-FSK demodulator with sync-word detection
 * disabled and a fixed 255-byte packet, so the receiver free-runs and each
 * recovered bit is one sample of the OOK envelope. At 32 kBaud that is a ~31 us
 * sample period, oversampling the remote's ~520 us mark about 17x.
 *
 * The original 2026-07-22 capture used 4.8 kBaud (~208 us), which was enough to
 * decode the payloads but quantised every pulse width to a multiple of 208 us --
 * far too coarse to reconstruct a transmit waveform from. A faster sample rate
 * also completes a 255-byte packet in ~64 ms rather than ~425 ms, so many more
 * of the remote's ~250 ms bursts get caught.
 *
 * A sampling oscilloscope built out of a packet radio. Crude, but it produced
 * the only known-good capture of this remote, whereas async direct mode
 * (rx_capture.ino) yields a fragmented envelope on this hardware.
 *
 * Uses mfurga/cc1101 rather than RadioLib deliberately. The CC1101's FIFO is
 * only 64 bytes, so a 255-byte packet must be drained progressively while it
 * arrives; this library's readData() does that, RadioLib's returns immediately
 * with stale FIFO contents.
 *
 * Output format is identical to the 2026-07-22 capture so that
 * tools/decode_capture.py parses it directly:
 *     RSSI=-48 LEN=255 DATA=FF FF FF F8 00 07 39 ...
 *
 * Wiring: GDO0->GPIO4  CSN->GPIO5  SCK->GPIO18  MISO->GPIO19  MOSI->GPIO23
 *
 * Serial @ 115200. Commands:
 *   t  cycle the RSSI gate (-80 / -70 / -60 / off)
 *   ?  print current settings
 */

// Keep the blocking receive() short so the loop stays responsive. Must be
// defined before the include.
#define CC1101_RECV_TIMEOUT_MS 1000
#include <cc1101.h>

#define PIN_CSN   5
#define PIN_SCK   18
#define PIN_MISO  19
#define PIN_MOSI  23
#define PIN_GDO0  4

CC1101::Radio radio(PIN_CSN, PIN_SCK, PIN_MISO, PIN_MOSI, PIN_GDO0, PIN_UNUSED);

// --- Configuration (matches the 2026-07-22 capture) --------------------------

static const double FREQ_MHZ     = 433.92;
static const double BITRATE_KBPS = 4.8;    // -> ~208 us per envelope sample
static const double RXBW_KHZ     = 325.0;
static const uint8_t PACKET_LEN  = 255;

/* The remote arrives around -45 dBm while the noise floor sits near -85, so a
   strength gate cheaply separates real transmissions from the free-running
   receiver's noise. -128 logs everything. */
static const int8_t RSSI_GATES[] = {-80, -70, -60, -128};
static uint8_t gateIndex = 0;

static uint8_t buffer[PACKET_LEN];

/* An OOK receiver with no carrier present cranks the AGC until the noise floor
   crosses the detector threshold, and the output sticks permanently high --
   whole packets come back as 0xFF. Capping the digital gain fixes it:
   AGCCTRL2 bits 7:6 (MAX_DVGA_GAIN) reduce maximum gain by that many steps.
   3 = quietest, 0 = most sensitive. Adjustable at runtime with '+' / '-'. */
static uint8_t maxDvgaGain = 3;
#define AGCCTRL2_VALUE(dvga) (((dvga) << 6) | 0x07)   // MAX_LNA_GAIN=0, MAGN_TARGET=42dB

static void applyAgc() {
  radio.writeReg(0x1B, AGCCTRL2_VALUE(maxDvgaGain));   // AGCCTRL2
  radio.writeReg(0x1C, 0x00);                          // AGCCTRL1
  radio.writeReg(0x1D, 0x91);                          // AGCCTRL0, 8 dB ASK boundary
}

static void printSettings() {
  int8_t gate = RSSI_GATES[gateIndex];
  Serial.printf("[cfg] %.2f MHz 2-FSK bw=%.0fkHz %.1f kBaud len=%u gate=",
                FREQ_MHZ, RXBW_KHZ, BITRATE_KBPS, PACKET_LEN);
  if (gate == -128) Serial.println("off");
  else              Serial.printf("%d dBm\n", gate);
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n=== AC Infinity rx_packet (envelope sampling) ===");

  /* 2-FSK, not OOK. FREQEST estimates carrier offset from frequency modulation,
     so in ASK/OOK it always reads zero -- an earlier OOK build reported a
     useless FREQEST=0 on every packet. 2-FSK at 4.8 kBaud is also the
     configuration that reliably decodes this remote. */
  CC1101::Status status = radio.begin(CC1101::MOD_2FSK, FREQ_MHZ, BITRATE_KBPS);
  Serial.printf("[probe] PARTNUM=0x%02X VERSION=0x%02X\n",
                radio.getChipPartNumber(), radio.getChipVersion());
  if (status != CC1101::STATUS_OK) {
    Serial.printf("CC1101 begin() failed: %d\n", status);
    while (true) delay(1000);
  }

  // Free-run the receiver: no sync word, no CRC, no whitening, fixed length.
  radio.setCrc(false);
  radio.setDataWhitening(false);
  radio.setSyncMode(CC1101::SYNC_MODE_NO_PREAMBLE);
  radio.setPacketLengthMode(CC1101::PKT_LEN_MODE_FIXED, PACKET_LEN);
  radio.setRxBandwidth(RXBW_KHZ);
  applyAgc();

  printSettings();
  Serial.println("Ready -- press the remote button whenever. Logging continuously.\n");
}

static void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 't') {
      gateIndex = (gateIndex + 1) % (sizeof(RSSI_GATES) / sizeof(RSSI_GATES[0]));
      printSettings();
    } else if (c == '+') {
      if (maxDvgaGain > 0) maxDvgaGain--;
      applyAgc();
      Serial.printf("[cfg] MAX_DVGA_GAIN=%u\n", maxDvgaGain);
    } else if (c == '-') {
      if (maxDvgaGain < 3) maxDvgaGain++;
      applyAgc();
      Serial.printf("[cfg] MAX_DVGA_GAIN=%u\n", maxDvgaGain);
    } else if (c == '?') {
      printSettings();
    }
  }
}

void loop() {
  static uint32_t lastBeatMs = 0;
  static uint32_t received = 0, logged = 0;
  static int8_t peakRssi = -128;

  handleSerial();

  size_t bytesRead = 0;
  CC1101::Status status = radio.receive(buffer, sizeof(buffer), &bytesRead);

  if (status == CC1101::STATUS_OK && bytesRead > 0) {
    received++;
    int8_t rssi = radio.getRSSI();
    if (rssi > peakRssi) peakRssi = rssi;

    if (rssi >= RSSI_GATES[gateIndex]) {
      logged++;
      /* FREQEST is the demodulator's estimate of how far the received carrier
         sits from where we are tuned. It matters because a 325 kHz receive
         bandwidth happily receives a remote that is ~150 kHz off frequency,
         while the fan's own receiver may be narrow enough that transmitting at
         433.92 misses it. Units are 26 MHz / 2^14 = 1.587 kHz per LSB. */
      int8_t fe = (int8_t)radio.readReg(0x32);
      Serial.printf("FREQEST=%d (%.1f kHz offset) ", fe, fe * 26000.0 / 16384.0);
      Serial.printf("RSSI=%d LEN=%u DATA=", rssi, (unsigned)bytesRead);
      for (size_t i = 0; i < bytesRead; i++) {
        Serial.printf("%02X ", buffer[i]);
      }
      Serial.println();
    }
  }

  if (millis() - lastBeatMs >= 10000) {
    lastBeatMs = millis();
    Serial.printf("[idle] received=%lu logged=%lu peak=%d dBm\n",
                  (unsigned long)received, (unsigned long)logged, (int)peakRssi);
    peakRssi = -128;
  }
}

/**
 * fan_remote_ha -- Home Assistant control for the AC Infinity fan.
 *
 * ESP32 + CC1101 that joins your WiFi, connects to the Mosquitto broker, and
 * announces itself to Home Assistant over MQTT discovery. HA creates entities
 * automatically -- an OFF/1-10 dropdown, plus a standard fan entity -- with no
 * custom component and no YAML.
 *
 * Transmits the same 2-FSK protocol as fan_remote.ino. See protocol.md and
 * AC_INFINITY_REMOTE_SPEC.md. The critical detail is that the remote is 2-FSK,
 * NOT OOK: an OOK transmission of the same bits jams the band but is never
 * decodable by the fan.
 *
 * Setup:
 *   1. cp config.h.example config.h  and fill in your details
 *   2. Flash. The fan appears in HA under Settings -> Devices & Services -> MQTT
 *
 * Wiring: GDO0->GPIO4  CSN->GPIO5  SCK->GPIO18  MISO->GPIO19  MOSI->GPIO23
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <cc1101.h>
#include "config.h"

// --- Pins --------------------------------------------------------------------

#define PIN_CSN   5
#define PIN_SCK   18
#define PIN_MISO  19
#define PIN_MOSI  23
#define PIN_GDO0  4

CC1101::Radio radio(PIN_CSN, PIN_SCK, PIN_MISO, PIN_MOSI, PIN_GDO0, PIN_UNUSED);

WiFiClient   wifiClient;
PubSubClient mqtt(wifiClient);

// --- Radio -------------------------------------------------------------------

static const double FREQ_MHZ     = 433.92;
static const double BITRATE_KBPS = 4.8;    // FSK clock recovery needs this exact rate
static const double FSK_DEV_KHZ  = 35.0;
static const int8_t TX_POWER     = 10;

// Waveform, in microseconds (see protocol.md).
static const double MARK_US   = 531.8;
static const double SPACE0_US = 518.0;
static const double SPACE1_US = 1066.7;
static const double SYNC_US   = 6667.0;
static const double GAP_US    = 3333.0;

static const uint8_t FRAME_BITS  = 16;
static const uint8_t REPETITIONS = 8;    // frames per burst, as the remote sends

/* How many times to send the whole burst. The real remote emits 8 frames per
   button press, but a person pressing a button naturally sends several bursts
   while watching the fan respond. A single burst is occasionally missed --
   observed as "speed 6 works from 10 but not from OFF", which was really
   intermittent reception rather than anything speed-specific. Repeating the
   burst costs about a second of airtime and makes commands reliable. */
static const uint8_t BURST_REPEATS = 3;
static const uint16_t BURST_GAP_MS = 120;

/* Coming from fully OFF needs more: the fan's receiver is cold and appears to
   need longer to lock onto a signal than it does switching between two
   running speeds. Observed directly -- OFF/1 -> 6 needed a manual pause to
   land reliably, while running-speed changes did not. A longer lead-in delay
   plus more repeats reproduces that pause automatically instead of relying on
   the user to wait between commands. */
static const uint8_t  WAKE_BURST_REPEATS = 6;
static const uint16_t WAKE_LEAD_MS       = 200;

static const uint8_t SPEED_MAX   = 10;

// --- MQTT topics -------------------------------------------------------------

#define T_BASE       "acinfinity/" DEVICE_ID
#define T_STATE      T_BASE "/state"          // ON / OFF
#define T_CMD        T_BASE "/set"
#define T_PCT_STATE  T_BASE "/speed"          // 0-100
#define T_PCT_CMD    T_BASE "/speed/set"
#define T_PRESET_ST  T_BASE "/preset"          // "1".."10"
#define T_PRESET_CMD T_BASE "/preset/set"
#define T_SEL_ST     T_BASE "/select"          // "OFF" or "1".."10"
#define T_SEL_CMD    T_BASE "/select/set"
#define T_AVAIL      T_BASE "/available"
#define T_DISCOVERY  "homeassistant/fan/" DEVICE_ID "/config"
#define T_DISC_SEL   "homeassistant/select/" DEVICE_ID "_speed/config"

// --- Protocol ----------------------------------------------------------------

/* Checksum: GF(2^8) multiply of the speed by 0x43, reduction polynomial 0x131,
   for speeds 1-9. Speeds 0 and 10 are exceptions. */
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

/* ---------------------------------------------------------------------------
   ADDRESS NIBBLE -- what is and is not known
   ---------------------------------------------------------------------------
   Only address 0 is verified. Every capture taken has the top nibble as 0.

   Two candidate models exist for how a non-zero address changes the checksum,
   and they disagree for every non-zero address:

     Model A (used here) -- GF(2^8) over the whole [addr:4][speed:4] byte:
         checksum(addr,speed) = checksum(0,speed) XOR (0x43 (x) (addr<<4))
       This is the natural extension of the confirmed addr=0 rule, because GF
       multiplication is linear.

     Model B -- from the pre-existing ac_infinity_protocol.txt, an ordinary
     integer multiply mod 256:
         checksum(addr,speed) = checksum(0,speed) XOR ((addr * K) & 0xFF)
         K = 0xEE for speed 0, 0xE2 for speeds 1-9, 0x95 for speed 10

   XOR constant applied to the addr=0 checksum, under each model:

   addr | GF model | legacy  
   -----+----------+---------
      1 |   0xF4   |  0xE2
      2 |   0xD9   |  0xC4
      3 |   0x2D   |  0xA6
      4 |   0x83   |  0x88
      5 |   0x77   |  0x6A
      6 |   0x5A   |  0x4C
      7 |   0xAE   |  0x2E
      8 |   0x37   |  0x10
      9 |   0xC3   |  0xF2
     10 |   0xEE   |  0xD4
     11 |   0x1A   |  0xB6
     12 |   0xB4   |  0x98
     13 |   0x40   |  0x7A
     14 |   0x6D   |  0x5C
     15 |   0x99   |  0x3E

   To settle it: change a DIP switch on the remote, capture a press with
   rx_packet.ino, and decode. The top nibble gives the address; whichever model
   predicts the observed checksum is correct. Use a mid-range speed (1-9) --
   speeds 0 and 10 already break the GF rule at addr=0, so they are doubly
   uncertain.

   If Model B turns out to be right, change speedChecksum() below to:
       uint8_t k = (speed == 0) ? 0xEE : (speed == 10) ? 0x95 : 0xE2;
       return base ^ (uint8_t)(addr * k);
   --------------------------------------------------------------------------- */
#ifndef FAN_ADDRESS
#define FAN_ADDRESS 0
#endif

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

static uint16_t buildPayload(uint8_t speed) {
  const uint8_t addr = FAN_ADDRESS & 0x0F;
  return ((uint16_t)addr << 12) | ((uint16_t)speed << 8) | speedChecksum(speed, addr);
}

// --- Waveform ----------------------------------------------------------------

static uint8_t  txBuf[255];
static uint16_t txBits;
static double   txTimeUs;

/* Build in microseconds and sample at the bit period. At 4.8 kBaud a 531.8 us
   mark is 2.55 bit periods, so rounding elements to whole bits would shift every
   edge by up to 100 us and accumulate across the frame. */
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

// --- State -------------------------------------------------------------------

/* The fan gives no feedback, so this is the commanded speed rather than a
   reading. It survives as long as the ESP32 stays powered; HA shows it as the
   entity state. */
static uint8_t currentSpeed = 0;

static void publishState() {
  mqtt.publish(T_STATE, currentSpeed > 0 ? "ON" : "OFF", true);

  char pct[8];
  snprintf(pct, sizeof(pct), "%u", (unsigned)(currentSpeed * 10));   // 0-10 -> 0-100
  mqtt.publish(T_PCT_STATE, pct, true);

  /* Preset mode mirrors the speed as a discrete 1-10 label. Home Assistant
     renders presets as distinct selectable values rather than a continuous
     slider, which is what an 11-position fan actually wants. Speed 0 has no
     preset -- OFF is the entity's power state, and HA rejects a preset named
     "off". */
  if (currentSpeed > 0) {
    char preset[4];
    snprintf(preset, sizeof(preset), "%u", currentSpeed);
    mqtt.publish(T_PRESET_ST, preset, true);
  }

  /* The select entity carries OFF as a normal option, which a fan entity cannot
     -- Home Assistant reserves off/on as the fan's power state and rejects a
     preset named "off". A select therefore gives a single dropdown covering all
     11 states with no power buttons at all. */
  char sel[4];
  if (currentSpeed == 0) snprintf(sel, sizeof(sel), "OFF");
  else                   snprintf(sel, sizeof(sel), "%u", currentSpeed);
  mqtt.publish(T_SEL_ST, sel, true);
}

static void transmitSpeed(uint8_t speed) {
  if (speed > SPEED_MAX) speed = SPEED_MAX;
  uint16_t payload = buildPayload(speed);
  uint8_t len = buildBurst(payload);

  radio.setPacketLengthMode(CC1101::PKT_LEN_MODE_FIXED, len);

  bool waking = (currentSpeed == 0 && speed != 0);
  uint8_t repeats = waking ? WAKE_BURST_REPEATS : BURST_REPEATS;
  if (waking) delay(WAKE_LEAD_MS);

  bool ok = true;
  for (uint8_t i = 0; i < repeats; i++) {
    if (radio.transmit(txBuf, len) != CC1101::STATUS_OK) ok = false;
    if (i + 1 < repeats) delay(BURST_GAP_MS);
  }

  if (ok) {
    currentSpeed = speed;
    Serial.printf("TX speed=%u payload=0x%04X (%u bursts%s)\n", speed, payload, repeats,
                  waking ? ", wake" : "");
  } else {
    Serial.printf("TX speed=%u FAILED\n", speed);
  }
  publishState();
}

// --- MQTT --------------------------------------------------------------------

static void onMessage(char *topic, byte *payload, unsigned int length) {
  char msg[16] = {0};
  if (length >= sizeof(msg)) length = sizeof(msg) - 1;
  memcpy(msg, payload, length);

  if (!strcmp(topic, T_CMD)) {
    if (!strcmp(msg, "ON")) {
      // Turning on with no speed set: use the last speed, or 1 from cold.
      transmitSpeed(currentSpeed > 0 ? currentSpeed : 1);
    } else if (!strcmp(msg, "OFF")) {
      transmitSpeed(0);
    }
  } else if (!strcmp(topic, T_SEL_CMD)) {
    if (!strcasecmp(msg, "OFF")) {
      transmitSpeed(0);
    } else {
      long v = strtol(msg, nullptr, 10);
      if (v >= 0 && v <= SPEED_MAX) transmitSpeed((uint8_t)v);
      else Serial.printf("ignoring select '%s'\n", msg);
    }

  } else if (!strcmp(topic, T_PRESET_CMD)) {
    long v = strtol(msg, nullptr, 10);
    if (v >= 1 && v <= SPEED_MAX) transmitSpeed((uint8_t)v);
    else Serial.printf("ignoring preset '%s' (expect 1-%u)\n", msg, SPEED_MAX);

  } else if (!strcmp(topic, T_PCT_CMD)) {
    long pct = strtol(msg, nullptr, 10);
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    /* HA sends 0-100; the fan has 11 discrete steps. Round to nearest so the
       slider lands predictably, and let any non-zero percentage produce at
       least speed 1 rather than silently turning the fan off. */
    uint8_t speed = (uint8_t)((pct + 5) / 10);
    if (pct > 0 && speed == 0) speed = 1;
    transmitSpeed(speed);
  }
}

/* Home Assistant MQTT discovery. Publishing this retained makes HA create the
   fan entity by itself -- no YAML, no custom component.

   Deliberately advertises preset_modes and NOT percentage. Home Assistant
   renders any percentage-capable fan with a continuous slider, which is
   imprecise for a device with only 11 discrete positions. Presets give
   selectable values instead. The percentage command topic is still subscribed
   below, so manual MQTT publishes keep working, but HA never shows a slider. */
static void publishDiscovery() {
  static char cfg[1200];
  snprintf(cfg, sizeof(cfg),
    "{"
      "\"name\":\"%s\","
      "\"unique_id\":\"%s\","
      "\"state_topic\":\"%s\","
      "\"command_topic\":\"%s\","
      "\"preset_mode_state_topic\":\"%s\","
      "\"preset_mode_command_topic\":\"%s\","
      "\"preset_modes\":[\"1\",\"2\",\"3\",\"4\",\"5\",\"6\",\"7\",\"8\",\"9\",\"10\"],"
      "\"availability_topic\":\"%s\","
      "\"payload_available\":\"online\","
      "\"payload_not_available\":\"offline\","
      "\"optimistic\":true,"
      "\"device\":{"
        "\"identifiers\":[\"%s\"],"
        "\"name\":\"%s\","
        "\"manufacturer\":\"AC Infinity\","
        "\"model\":\"433 MHz remote clone (ESP32 + CC1101)\""
      "}"
    "}",
    DEVICE_NAME, DEVICE_ID, T_STATE, T_CMD,
    T_PRESET_ST, T_PRESET_CMD, T_AVAIL, DEVICE_ID, DEVICE_NAME);

  mqtt.publish(T_DISCOVERY, cfg, true);

  /* Second entity: a select listing OFF plus speeds 1-10. Shares the same
     device block so both appear under one device in Home Assistant. */
  snprintf(cfg, sizeof(cfg),
    "{"
      "\"name\":\"%s Speed\","
      "\"unique_id\":\"%s_speed\","
      "\"state_topic\":\"%s\","
      "\"command_topic\":\"%s\","
      "\"availability_topic\":\"%s\","
      "\"payload_available\":\"online\","
      "\"payload_not_available\":\"offline\","
      "\"options\":[\"OFF\",\"1\",\"2\",\"3\",\"4\",\"5\",\"6\",\"7\",\"8\",\"9\",\"10\"],"
      "\"icon\":\"mdi:fan\","
      "\"device\":{"
        "\"identifiers\":[\"%s\"],"
        "\"name\":\"%s\","
        "\"manufacturer\":\"AC Infinity\","
        "\"model\":\"433 MHz remote clone (ESP32 + CC1101)\""
      "}"
    "}",
    DEVICE_NAME, DEVICE_ID, T_SEL_ST, T_SEL_CMD, T_AVAIL, DEVICE_ID, DEVICE_NAME);
  mqtt.publish(T_DISC_SEL, cfg, true);

  Serial.println("published HA discovery config (fan + speed select)");
}

static void connectMqtt() {
  while (!mqtt.connected()) {
    Serial.printf("MQTT connecting to %s:%d ... ", MQTT_HOST, MQTT_PORT);
    // Last will marks us offline if the ESP32 drops off.
    if (mqtt.connect(DEVICE_ID, MQTT_USER, MQTT_PASSWORD, T_AVAIL, 0, true, "offline")) {
      Serial.println("connected");
      mqtt.publish(T_AVAIL, "online", true);
      publishDiscovery();
      mqtt.subscribe(T_CMD);
      mqtt.subscribe(T_PCT_CMD);
      mqtt.subscribe(T_PRESET_CMD);
      mqtt.subscribe(T_SEL_CMD);
      publishState();
    } else {
      Serial.printf("failed rc=%d, retrying in 5s\n", mqtt.state());
      delay(5000);
    }
  }
}

// --- Setup / loop ------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== AC Infinity fan -- Home Assistant bridge ===");

  CC1101::Status status = radio.begin(CC1101::MOD_2FSK, FREQ_MHZ, BITRATE_KBPS);
  if (status != CC1101::STATUS_OK) {
    Serial.printf("CC1101 begin() failed: %d -- check wiring\n", status);
    while (true) delay(1000);
  }
  radio.setCrc(false);
  radio.setDataWhitening(false);
  radio.setSyncMode(CC1101::SYNC_MODE_NO_PREAMBLE);
  radio.setFrequencyDeviation(FSK_DEV_KHZ);
  radio.setOutputPower(TX_POWER);
  Serial.println("CC1101 ready (433.92 MHz, 2-FSK, 4.8 kBaud)");

  WiFi.mode(WIFI_STA);
  WiFi.setHostname(DEVICE_ID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("WiFi connecting to %s", WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.printf(" connected, IP %s\n", WiFi.localIP().toString().c_str());

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMessage);
  mqtt.setBufferSize(1024);      // discovery payload exceeds the 256-byte default
  connectMqtt();
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi lost, reconnecting...");
    WiFi.reconnect();
    delay(2000);
    return;
  }
  if (!mqtt.connected()) connectMqtt();
  mqtt.loop();

  // Serial control still works, which is handy for testing without HA.
  static char buf[16];
  static uint8_t len = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      buf[len] = '\0';
      if (len) {
        long v = strtol(buf, nullptr, 10);
        if (v >= 0 && v <= SPEED_MAX) transmitSpeed((uint8_t)v);
        else Serial.printf("enter 0-%u\n", SPEED_MAX);
      }
      len = 0;
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = c;
    } else {
      len = 0;
    }
  }
}

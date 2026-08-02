# Home Assistant bridge

ESP32 + CC1101 that exposes the AC Infinity fan to Home Assistant as a native
fan entity over MQTT — on/off, a speed slider, automations and voice control.

No custom component and no YAML. The ESP32 announces itself via MQTT discovery
and HA creates the entity by itself.

## Setup

**1. Make sure you have an MQTT broker.**

In Home Assistant: *Settings → Add-ons → Add-on Store → Mosquitto broker →
Install → Start*. Then add the MQTT integration if it isn't already there
(*Settings → Devices & Services → Add Integration → MQTT*).

Create a user for the ESP32 under *Settings → People → Users* — a normal HA user
account is what Mosquitto authenticates against.

**2. Fill in your details.**

```bash
cp config.h.example config.h
```

Then edit `config.h`:

```c
#define WIFI_SSID       "your-wifi-name"
#define WIFI_PASSWORD   "your-wifi-password"
#define MQTT_HOST       "192.168.1.50"      // your HA host's IP
#define MQTT_USER       "mqtt-user"
#define MQTT_PASSWORD   "mqtt-password"
```

Use an IP address for `MQTT_HOST` rather than `homeassistant.local` — mDNS
resolution is unreliable on the ESP32.

**3. Flash.**

```bash
export PATH="$PWD/../bin:$PATH"
export ARDUINO_DATA_DIR="$PWD/../arduino-data"
FQBN="esp32:esp32:esp32:PSRAM=disabled,PartitionScheme=default,FlashMode=dio"

arduino-cli compile --fqbn "$FQBN" --output-dir build .
arduino-cli upload -p /dev/ttyUSB0 --fqbn "$FQBN" --input-dir build .
```

**4. Check it.**

Watch the serial output at 115200:

```
CC1101 ready (433.92 MHz, 2-FSK, 4.8 kBaud)
WiFi connecting to your-wifi-name... connected, IP 192.168.1.123
MQTT connecting to 192.168.1.50:1883 ... connected
published HA discovery config
```

The fan then appears under *Settings → Devices & Services → MQTT*, and as a fan
card on your dashboard.

## Two entities

The bridge creates both, under one device:

| Entity | Use |
|---|---|
| `select.ac_infinity_fan_speed` | **A single dropdown: OFF, 1–10.** No power buttons. Best for dashboards |
| `fan.ac_infinity_fan` | Standard fan entity — voice control, HomeKit, fan-aware automations |

A `select` can carry `OFF` as an ordinary option; a fan entity cannot, because
Home Assistant reserves off/on as the power state and rejects a preset named
"off". So the select gives all 11 states in one control with no toggle.

Put the select on your dashboard and ignore the fan entity, or use both.

```yaml
type: entities
entities:
  - entity: select.ac_infinity_fan_speed
    name: Fan speed
```

```yaml
service: select.select_option
target:
  entity_id: select.ac_infinity_fan_speed
data:
  option: "5"        # or "OFF"
```

## Using the fan entity

The entity is `fan.ac_infinity_fan` (name depends on `DEVICE_NAME`).

```yaml
# Turn on at speed 5
service: fan.set_percentage
target:
  entity_id: fan.ac_infinity_fan
data:
  percentage: 50

# Off
service: fan.turn_off
target:
  entity_id: fan.ac_infinity_fan
```

### Speeds

The fan has 11 discrete states, so the entity exposes **preset modes `1`–`10`**
alongside the power toggle. Presets are discrete values rather than a continuous
slider, which suits an 11-position fan — Home Assistant's percentage slider
feels imprecise when only 11 positions exist.

```yaml
service: fan.set_preset_mode
target:
  entity_id: fan.ac_infinity_fan
data:
  preset_mode: "5"
```

The entity deliberately does **not** advertise a percentage interface. Home
Assistant renders any percentage-capable fan with a continuous slider, which is
imprecise when the device has only 11 positions. Presets give discrete values.

The percentage command topic is still subscribed, so a manual
`acinfinity/<id>/speed/set` publish (0–100) still works — HA just never shows a
slider for it.

**For speed buttons on a dashboard:**

```yaml
type: tile
entity: fan.ac_infinity_fan
features:
  - type: fan-preset-modes
    style: icons          # omit for a dropdown instead
    preset_modes: ["1","2","3","4","5","6","7","8","9","10"]
```

Power on/off is on the tile itself, so all 11 states are reachable.

Serial control still works for testing without HA — type `0`–`10` and enter.

## How state is tracked

The fan sends nothing back, so there is no way to read its real speed. The
bridge reports the last speed it commanded, which HA treats as the entity state
(`optimistic: true`). Consequences:

- If someone uses the physical remote, HA won't know and will drift out of sync.
  Commanding any speed from HA resyncs it.
- After an ESP32 power cycle the bridge assumes OFF. If the fan was running, the
  first HA command will correct it.

The MQTT last-will marks the entity unavailable in HA if the ESP32 drops off the
network.

## Troubleshooting

**Entity never appears.** Check the serial log reaches "published HA discovery
config". If MQTT reports `rc=5` the credentials are wrong; `rc=-2` means it can't
reach the broker — check `MQTT_HOST`.

**Entity appears but the fan doesn't respond.** The radio is fine if
`fan_remote.ino` works. Confirm the antenna is attached and the CC1101 is within
range.

**Fan responds erratically.** Move the ESP32 closer, or reduce `TX_POWER` — a
very strong signal at close range can overload the fan's receiver.

## Protocol

See [`../AC_INFINITY_REMOTE_SPEC.md`](../AC_INFINITY_REMOTE_SPEC.md). The one
detail worth repeating: **the remote is 2-FSK, not OOK**. Transmitting the same
bits as OOK jams the band convincingly but the fan will never decode a frame.

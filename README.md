# AC Infinity fan → Home Assistant (ESP32 + CC1101)

Control an AC Infinity 433 MHz fan from Home Assistant, using an ESP32 and a
CC1101 radio. The remote's protocol was reverse engineered from scratch.

Home Assistant gets a **native fan entity** with on/off and a speed slider. No
custom component, no YAML.

☕ **Buy me a coffee:** https://paypal.me/ESP32CC1101/5

---

## What this does

- Replaces the handheld remote entirely — set any speed directly, no stepping
- Native HA fan entity via MQTT discovery, created automatically
- Two entities: a clean OFF/1-10 dropdown, plus a standard fan entity for voice and automations
- Works with automations, scenes, dashboards and voice
- Full protocol documentation so you can port it anywhere

## Hardware

An ESP32 dev board and a CC1101 433 MHz module. That's it.

| CC1101 | ESP32 |
|---|---|
| VCC | 3V3 — **3.3 V only, 5 V will damage the CC1101** |
| GND | GND |
| GDO0 | GPIO4 |
| CSN | GPIO5 |
| SCK | GPIO18 |
| MISO | GPIO19 |
| MOSI | GPIO23 |
| GDO2 | not connected |

An antenna is required — a 17.3 cm wire works fine at 433 MHz.

## Quick start

```bash
git clone https://github.com/jawtech123/Home-assistant-AC-Infinity-ESP32-CC1101-
cd Home-assistant-AC-Infinity-ESP32-CC1101-/fan_remote_ha
cp config.h.example config.h    # add your WiFi + MQTT details
```

Install the **Mosquitto broker** add-on in Home Assistant if you haven't already
(*Settings → Add-ons → Add-on Store*), then flash:

```bash
FQBN="esp32:esp32:esp32:PSRAM=disabled,PartitionScheme=default,FlashMode=dio"
arduino-cli compile --fqbn "$FQBN" --output-dir build .
arduino-cli upload -p /dev/ttyUSB0 --fqbn "$FQBN" --input-dir build .
```

The fan appears under *Settings → Devices & Services → MQTT*. Full instructions
in [`fan_remote_ha/README.md`](fan_remote_ha/README.md).

**Libraries:** [PubSubClient](https://github.com/knolleary/pubsubclient) and
[mfurga/cc1101](https://github.com/mfurga/cc1101).

## Repository layout

| Path | Purpose |
|---|---|
| `fan_remote_ha/` | **Home Assistant bridge** — WiFi + MQTT. Start here |
| `fan_remote/` | Standalone transmitter, serial-controlled. Good for testing |
| `rx_packet/` | Receiver — captures and decodes the real remote |
| `rx_capture/` | Hardware diagnostics (pin/register probing) |
| `tools/decode_capture.py` | Offline decoder. `--table`, `--selftest` need no hardware |
| `tools/monitor.py` | Serial logger |
| `captures/` | Reference captures the protocol was derived from |
| [`AC_INFINITY_REMOTE_SPEC.md`](AC_INFINITY_REMOTE_SPEC.md) | **Protocol specification** — start here to port it |
| [`protocol.md`](protocol.md) | Working notes, observed vs derived |

## The protocol, briefly

| Property | Value |
|---|---|
| Frequency | 433.92 MHz |
| Modulation | **2-FSK**, ~35 kHz deviation |
| Bit rate | 4.8 kBaud |
| Encoding | PWM — fixed mark, variable space |
| Frame | 16 bits MSB first, 8 repetitions |
| Payload | `[addr:4][speed:4][checksum:8]` |
| Security | None — static codes, fully replayable |

> **If you take one thing from this: the remote is 2-FSK, not OOK.**
>
> The demodulated stream looks exactly like a PWM on-off envelope, so OOK is the
> natural assumption. It's wrong. An OOK transmission of the correct bits puts
> energy on 433.92 MHz — enough to *jam the real remote* — while the fan decodes
> nothing at all. Every register, timing and waveform comparison looks perfect.
> This cost most of a day.
>
> Quick check: capture with an amplitude detector and you get a packet of solid
> `0xFF`. That's constant-envelope FSK, easily mistaken for AGC saturation.

The checksum is a multiply in GF(2⁸), not a CRC:

```python
checksum(speed) = 0x43 ⊗ speed        # poly 0x131, speeds 1-9
checksum(0) = 0xAC,  checksum(10) = 0x82
```

Full details, including the DIP-switch address handling, in
[`AC_INFINITY_REMOTE_SPEC.md`](AC_INFINITY_REMOTE_SPEC.md).

### Code table (address 0)

| Speed | Payload | | Speed | Payload |
|---|---|---|---|---|
| OFF | `0x00AC` | | 6 | `0x06BB` |
| 1 | `0x0143` | | 7 | `0x07F8` |
| 2 | `0x0286` | | 8 | `0x087A` |
| 3 | `0x03C5` | | 9 | `0x0939` |
| 4 | `0x043D` | | MAX | `0x0A82` |
| 5 | `0x057E` | | | |

All eleven observed on air. Three of them (`2`, `5`, `9`) were *predicted* by the
GF(2⁸) rule before ever being captured, then later confirmed exactly — which is
what gives confidence the rule is the real algorithm rather than a curve fit.

## Compatibility

Developed against a single-button AC Infinity remote with ten speed LEDs
(CTR63A-style). Other AC Infinity remotes on 433 MHz likely share the protocol
but this hasn't been tested.

Addresses are set by the remote's DIP switches; 0, 1, 2, 4, 6 and 7 have been
captured. See the spec for the address rule and its one known exception.

If you try this on other hardware, an issue or PR reporting what happened would
be welcome — especially captures of addresses 3 or 5.

## Known limitations

- **State is optimistic.** The fan sends nothing back, so the bridge reports the
  last speed it commanded. Using the physical remote will drift HA out of sync
  until the next command from HA.
- **Address 7 breaks the checksum rule** and is special-cased from measurement.
- **Speed 0 follows no known address rule** and uses a lookup.
- Addresses 3 and 5 are predicted, not measured.

## Contributing & contact

Questions, problems and contributions are all welcome — please
**[open an issue](https://github.com/jawtech123/Home-assistant-AC-Infinity-ESP32-CC1101-/issues)**.

Captures from other AC Infinity remotes are the most useful contribution,
particularly ones with different DIP switch settings. `rx_packet/` will record
them and `tools/decode_capture.py` will decode them.

If you don't have a GitHub account and can't open an issue, you can reach me at
**jawtech123@gmail.com**.

## Security note

This protocol has no rolling code, encryption or authentication. Any
transmission can be recorded and replayed indefinitely by anyone in radio range.
Typical for cheap 433 MHz appliance remotes, and fine for a fan — but don't rely
on it for anything where unauthorised control would matter.

## Disclaimer

This project was built with AI assistance (protocol analysis, code, and
documentation). It's provided as-is, with no warranty. Review the code before
flashing it to your own hardware, and use it at your own risk — I take no
responsibility or liability for any damage, malfunction, or other consequences
arising from its use.

## Licence

GPL-3.0 — see [LICENSE](LICENSE).

Not affiliated with or endorsed by AC Infinity. Protocol details were obtained
by observing radio transmissions from hardware I own, for interoperability.

---

If this saved you some time:

☕ **Buy me a coffee:** https://paypal.me/ESP32CC1101/5

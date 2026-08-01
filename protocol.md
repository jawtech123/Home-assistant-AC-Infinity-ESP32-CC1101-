# AC Infinity CTR63A remote — protocol

Reverse engineered from captures in `captures/`, and **confirmed by successfully
controlling the fan** on 2026-08-01. Every value below is observed or verified on
hardware; nothing is taken on faith from earlier notes.

Decode the reference capture yourself with:

```
tools/decode_capture.py captures/signals_2026-07-22.txt
```

## Radio

| Property | Value | Status |
|---|---|---|
| Carrier | 433.92 MHz | observed (offset measured at -3 to -5 kHz) |
| Modulation | **2-FSK** | confirmed on hardware |
| Deviation | ~35 kHz | confirmed working |
| Bit rate | 4.8 kBaud | confirmed working |
| Encoding | PWM — fixed mark, variable space | observed |
| Bit order | MSB first | observed |
| Frame length | 16 bits | observed |
| Repetitions | 8 per button press | observed |

## Timing

| Element | Duration | Meaning |
|---|---|---|
| Mark | 531.8 µs | precedes every bit |
| Space | 518.0 µs | bit `0` |
| Space | 1066.7 µs | bit `1` |
| Sync mark | 6667 µs | leads each frame |
| Gap | 3333 µs | between sync and frame data |
| Gap | ~12917 µs | after the 8th repetition |

Waveform per button press — the long mark *leads* each frame:

```
([6667 sync][3333 gap][16 bits]) x8
```

All timings were recovered statistically from 2112 mark samples across 132
decoded frames, so they are accurate to a few microseconds despite the reference
capture being quantised to 208 µs. Averaging many instances of the same element
cancels the quantisation.

## Modulation: 2-FSK, not OOK

This is the fact the whole thing depends on, and it is easy to get wrong.

The signal is **frequency-shift keyed**, not on-off keyed. The "mark" and
"space" above are the two FSK states, not carrier-present and carrier-absent.

The evidence, in the order it should have been read:

- Captured with a **2-FSK demodulator**, the signal yields clean, consistently
  decodable data every time.
- Captured with an **amplitude detector**, the same signal reads as solid
  carrier — a whole packet of `0xFF`, 2040 ones and 0 zeros. That is exactly
  what a constant-envelope FSK signal looks like to an amplitude detector.
- Transmitting the identical bit pattern as OOK does **nothing** to the fan, yet
  jams the real remote convincingly: the energy lands on 433.92 MHz, so it
  desenses the receiver, but no frame is ever decodable.
- Transmitting the same recording as 2-FSK drives the fan immediately.

The bit rate matters for the same reason. With OOK the rate is only the envelope
sampling resolution, but an FSK receiver recovers its clock at a specific rate;
4.8 kBaud is what works.

## Payload

16 bits, `[addr:4][speed:4][checksum:8]`.

`addr` is set by the remote's DIP switches. Addresses 0, 4, 6 and 7 have been
captured — see the dedicated section below.

`speed` is `0` for OFF, `1`–`9` for the numbered speeds, and `10` (`0xA`) for
MAX — matching the remote's 10 indicator LEDs and its OFF state.

### Checksum

For speeds 1–9 the checksum is a carry-less multiply of the speed by `0x43` in
GF(2⁸), reduction polynomial `0x131`:

```python
def gf_mul(a, b, poly=0x131):
    r = 0
    while b:
        if b & 1: r ^= a
        b >>= 1
        a <<= 1
        if a & 0x100: a ^= poly
    return r

checksum = gf_mul(0x43, speed)      # speeds 1-9
```

Speeds 0 and 10 do not follow the rule and are stored as exceptions:
`checksum(0) = 0xAC`, `checksum(10) = 0x82`. Both were observed directly.

This is worth stating precisely because it is what makes the three uncaptured
codes trustworthy. Fitting `checksum = k ⊗ speed` against **only the six speeds
that were actually observed** (1, 3, 4, 6, 7, 8) yields exactly one solution in
the whole search space of 255 multipliers × 256 polynomials: `k = 0x43`,
`poly = 0x131`. That solution then correctly predicts speeds 2, 5 and 9 — values
which appear nowhere in the capture. A rule uniquely determined by six points
that goes on to predict three more is strong evidence the rule is real, not a
curve fit.

Earlier notes recorded these checksums as an opaque 11-entry lookup table, and
also claimed an address-dependent term (`addr × 0xE2` and similar). The
address-dependent part is unverifiable — no capture contains a nonzero address —
and is omitted here rather than repeated as fact.


## Address (bits 15–12) — rule found, one exception

The address is set by the remote's DIP switches. Every value observed lies in
0–7, so the field is effectively 3 bits despite occupying a nibble.

A non-zero address XORs a constant into the checksum:

```
checksum(addr, speed) = checksum(0, speed) XOR ADDR_XOR[addr][speed_class]
```

### Speeds 1–9: carry-less multiply by 0x31

```
XOR(addr) = 0x31 ⊗ addr        (carry-less / GF(2) multiply)
```

| addr | measured | `0x31 ⊗ addr` | |
|---|---|---|---|
| 0 | `0x00` | `0x00` | ✓ |
| 1 | `0x31` | `0x31` | ✓ |
| 2 | `0x62` | `0x62` | ✓ |
| 3 | not captured | `0x53` | predicted |
| 4 | `0xC4` | `0xC4` | ✓ |
| 5 | not captured | `0xF5` | predicted |
| 6 | `0xA6` | `0xA6` | ✓ |
| 7 | `0x2E` | `0x97` | **✗ exception** |

Five of six measured addresses fit exactly, and address 6 (`= 4 XOR 2`) combines
linearly from its constituent bits — strong evidence this is the real
construction rather than a curve fit. Note that for addr ≤ 7 the product never
exceeds 8 bits, so no reduction polynomial is involved.

**Address 7 is a genuine exception**, measured at `0x2E` across five different
speeds and in two independent capture sessions. It also matches the address-7
row in the original notes. Curiously `0x31 ⊗ 15 = 0x2E`, which would suggest a
truncated address — but the frames genuinely carry `0111` with a full 16 bit
pairs, so that is coincidence rather than explanation.

Addresses 3 and 5 have never been captured. They are filled in from the rule and
should be treated as likely-but-unconfirmed; address 7 proves the rule is not
universal.

### Speed 0: no rule found

| addr | 0 | 1 | 2 | 4 | 6 | 7 |
|---|---|---|---|---|---|---|
| XOR | `0x00` | `0x9D` | `0xCE` | `0x68` | `0x0A` | `0x82` |

These fit no rule tested — not the `0x31` multiply, not GF(2) linearity in the
address bits, not an integer multiply. Speed 0 already breaks the checksum rule
at address 0, so it appears to be special-cased in the original firmware.
Measured values only; addresses 3 and 5 are unknown.

### Speed 10

Measured for addresses 0 (`0x00`), 2 (`0x62`), 4 (`0xC4`) and 7 (`0x13`). At 2
and 4 it equals the speeds-1–9 constant; at 7 it does not. Addresses 1 and 6 are
uncaptured.

### Note on the original document

The pre-existing `ac_infinity_protocol.txt` listed an address-7 table that had
never been verified. All of its entries match these captures exactly. That row
was real data, and the document has been right twice where it was doubted — its
apparently arbitrary checksum "lookup table" encoded a genuine GF(2⁸) rule. Its
*general* address formula, however, does not generalise.

### Adding an address

Set it on the remote, capture with `rx_packet.ino`, decode, and read the
constant as `observed_checksum XOR checksum(0, speed)`. Add it to
`ADDR_XOR_SPEEDS_1_9` in `tools/decode_capture.py` and `ADDR_XOR_1_9[]` in the
sketches. Capturing an OFF and a MAX as well completes that address.

## Code table

| Speed | Payload | Checksum | Status |
|---|---|---|---|
| OFF | `0x00AC` | `0xAC` | observed |
| 1 | `0x0143` | `0x43` | observed |
| 2 | `0x0286` | `0x86` | observed (2026-08-01) — **confirmed a prediction** |
| 3 | `0x03C5` | `0xC5` | observed |
| 4 | `0x043D` | `0x3D` | observed |
| 5 | `0x057E` | `0x7E` | observed (2026-08-01) — **confirmed a prediction** |
| 6 | `0x06BB` | `0xBB` | observed |
| 7 | `0x07F8` | `0xF8` | observed |
| 8 | `0x087A` | `0x7A` | observed |
| 9 | `0x0939` | `0x39` | observed (2026-08-01) — **confirmed a prediction** |
| MAX (10) | `0x0A82` | `0x82` | observed |

### The prediction was tested and held — every code is now observed

Speeds 2, 5 and 9 were originally derived rather than measured: the GF(2⁸) rule
was fitted to only the six speeds that had been seen on air (1, 3, 4, 6, 7, 8),
and it then *predicted* the other three. Later captures recorded all three
directly — speeds 5 and 9 in `captures/live_2026-08-01_addr0_b.txt`, speed 2 in
`captures/live_2026-08-01_freqest.txt` — and every one matched its predicted value exactly:
`0x0286`, `0x057E`, `0x0939`.

That is a genuine out-of-sample test, not a self-consistency check: a rule
uniquely determined by six data points went on to predict three more correctly.
The table is now fully verified by observation, with no derived entries left.

The remote's carrier was also measured at **−3.2 to −4.8 kHz** from 433.92 MHz
(CC1101 `FREQEST`), so the nominal frequency is correct to within a few kHz.

Across 17 captures every decoded frame satisfies its checksum, with one
exception: a single `0x06BA` in the noisiest 2026-07-22 capture whose seven
siblings decode as `0x06BB`. That is a one-bit error being correctly flagged.

Regenerate with `tools/decode_capture.py --table`.

## How the decode was validated

The decoded values are coherent across button presses rather than merely
self-consistent: the capture contains consecutive presses that land on
consecutive speeds — `0x00AC`→`0x0143` (OFF→1), `0x03C5`→`0x043D` (3→4),
`0x07F8`→`0x087A` (7→8) — each with 8 repetitions, exactly as a
step-through-the-cycle remote should behave. A wrong bit order or frame
alignment would not produce that pattern.

Final confirmation is end-to-end: `fan_remote.ino` synthesises these frames from
the table above and drives the real fan to any commanded speed.

One frame in the noisiest capture decodes as `0x06BA` where its seven siblings
decode as `0x06BB`. The checksum flags it as corrupt, which is the expected
behaviour for a single bit error, not a protocol inconsistency.

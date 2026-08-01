# AC Infinity Fan Remote — RF Protocol Specification

Complete specification of the 433 MHz radio protocol used by the single-button
AC Infinity fan remote (CTR63A-style: one button, ten speed LEDs).

**Status: fully reverse engineered and verified.** Every value below was
recovered from captured transmissions and confirmed by synthesising the signal
and successfully controlling a real fan.

Reverse engineered 2026-07/08 from raw captures using an ESP32 + CC1101. As far
as searches at the time could establish, this protocol had not been published
anywhere previously.

---

## 1. Summary

| Property | Value |
|---|---|
| Carrier frequency | 433.92 MHz (measured −3 to −5 kHz offset) |
| Modulation | **2-FSK** (frequency-shift keying) |
| Frequency deviation | ~35 kHz |
| Bit rate | 4.8 kBaud |
| Line coding | PWM — fixed-width mark, variable-width space |
| Frame length | 16 bits, MSB first |
| Repetitions | 8 per button press |
| Encryption / rolling code | None. Codes are static and directly replayable |

> **The single most important detail is that this is 2-FSK, not OOK.**
> It is easy to assume on-off keying, because the demodulated bit stream looks
> exactly like a PWM envelope. Transmitting that same bit pattern as OOK puts
> energy on the right frequency — enough to jam the real remote — but the fan
> will never decode a single frame. See §6.

---

## 2. Frame structure

Each button press transmits **8 identical frames** back to back. Each frame is
preceded by a long sync mark and a gap:

```
( [sync 6667 µs] [gap 3333 µs] [16 data bits] ) × 8
```

A data bit is a fixed-width mark followed by a space whose width encodes the
value:

```
bit 0:  [mark 531.8 µs][space  518.0 µs]     total ≈ 1050 µs
bit 1:  [mark 531.8 µs][space 1066.7 µs]     total ≈ 1599 µs
```

### Element timings

| Element | Duration | Tolerance observed |
|---|---|---|
| Mark (every bit) | 531.8 µs | ±5 µs |
| Space — bit `0` | 518.0 µs | ±5 µs |
| Space — bit `1` | 1066.7 µs | ±5 µs |
| Sync mark | 6667 µs | — |
| Gap after sync | 3333 µs | — |
| Gap after final repetition | ~12917 µs | — |

Timings were recovered statistically from 2112 mark samples across 132 decoded
frames, which averages out the 208 µs quantisation of the capture hardware.

A full button press occupies roughly **233 ms** on air.

---

## 3. Payload format

16 bits, transmitted MSB first:

```
 15  14  13  12 | 11  10   9   8 | 7  6  5  4  3  2  1  0
+---------------+---------------+-------------------------+
|   address     |     speed     |        checksum         |
|   (4 bits)    |   (4 bits)    |        (8 bits)         |
+---------------+---------------+-------------------------+
```

| Field | Bits | Description |
|---|---|---|
| Address | 15–12 | Set by the remote's DIP switches |
| Speed | 11–8 | `0` = OFF, `1`–`9` = speed, `10` (`0xA`) = MAX |
| Checksum | 7–0 | See §4 |

### Notes on the fields

**Address** is set by the remote's DIP switches. Addresses 0, 4, 6 and 7 have
been captured directly. See §4.1 — the address changes the checksum, and the
rule for doing so is only partly understood.

**Speed is absolute, not incremental.** Although the remote has only one button,
it does not transmit "increment". The remote tracks state internally (that is
what its ten LEDs display) and transmits the resulting absolute speed. Captures
show the speed nibble stepping `0,1,2,…,10` in lockstep with successive presses.
This means a controller can command any speed directly without stepping through
intermediate values.

---

## 4. Checksum algorithm

The checksum is **not** a CRC. It is a multiplication in the finite field
GF(2⁸).

For speeds 1–9:

```
checksum(speed) = 0x43 ⊗ speed
```

where `⊗` is carry-less multiplication modulo the reduction polynomial `0x131`
(x⁸ + x⁵ + x⁴ + 1).

Speeds 0 and 10 do not follow the rule and are fixed exceptions:

```
checksum(0)  = 0xAC
checksum(10) = 0x82
```

### Reference implementation

```python
def gf_mul(a, b, poly=0x131):
    """Carry-less multiply in GF(2^8)."""
    result = 0
    while b:
        if b & 1:
            result ^= a
        b >>= 1
        a <<= 1
        if a & 0x100:
            a ^= poly
    return result

def checksum(speed):
    if speed == 0:  return 0xAC
    if speed == 10: return 0x82
    return gf_mul(0x43, speed)

def payload(speed, addr=0):
    return (addr << 12) | (speed << 8) | checksum(speed)
```

```c
uint8_t gf_mul(uint8_t a, uint8_t b) {
    uint16_t wide = a; uint8_t result = 0;
    while (b) {
        if (b & 1) result ^= (uint8_t)wide;
        b >>= 1; wide <<= 1;
        if (wide & 0x100) wide ^= 0x131;
    }
    return result;
}

uint8_t checksum(uint8_t speed) {
    if (speed == 0)  return 0xAC;
    if (speed == 10) return 0x82;
    return gf_mul(0x43, speed);
}

uint16_t payload(uint8_t speed) {
    return ((uint16_t)speed << 8) | checksum(speed);
}
```

### How this was established

The rule was fitted against **only the six speeds observed at the time**
(1, 3, 4, 6, 7, 8). Across the entire search space of 255 multipliers × 256
reduction polynomials, exactly one solution fits: `k = 0x43`, `poly = 0x131`.

That solution then *predicted* speeds 2, 5 and 9 — values that appeared in no
capture. All three were subsequently observed on air and matched the predictions
exactly. A rule uniquely determined by six points that goes on to predict three
more is strong evidence of the real algorithm rather than a curve fit.

### 4.1 Address effect on the checksum

The address is set by the remote's DIP switches. All observed values lie in 0–7,
so the field is effectively 3 bits despite occupying a nibble.

A non-zero address XORs a constant into the checksum:

```
checksum(addr, speed) = checksum(0, speed) XOR ADDR_XOR[addr][speed_class]
```

**Speeds 1–9** follow a carry-less multiply of the address by `0x31`:

| addr | measured | `0x31 ⊗ addr` | |
|---|---|---|---|
| 0 | `0x00` | `0x00` | ✓ |
| 1 | `0x31` | `0x31` | ✓ |
| 2 | `0x62` | `0x62` | ✓ |
| 3 | not captured | `0x53` | predicted |
| 4 | `0xC4` | `0xC4` | ✓ |
| 5 | not captured | `0xF5` | predicted |
| 6 | `0xA6` | `0xA6` | ✓ |
| 7 | `0x2E` | `0x97` | **exception** |

Address 6 (`= 4 XOR 2`) combines linearly from its constituent bits, which is
good evidence the construction is real. For addr ≤ 7 the product never exceeds
8 bits, so no reduction polynomial is involved.

**Address 7 is a genuine exception** — measured at `0x2E` across five speeds in
two independent sessions. Implementations should special-case it.

**Speed 0 follows no known rule.** Measured constants: addr 1 `0x9D`, 2 `0xCE`,
4 `0x68`, 6 `0x0A`, 7 `0x82`. Speed 0 already breaks the checksum rule at
address 0, so it appears to be special-cased in firmware. Use a lookup.

**Speed 10** matches the speeds-1–9 constant at addresses 2 and 4, but not at 7
(`0x13`).

Practical guidance: use `0x31 ⊗ addr` for speeds 1–9 with address 7
special-cased, and a measured lookup for speeds 0 and 10.

---

## 5. Complete code table

| Speed | Payload | Checksum |
|---|---|---|
| OFF | `0x00AC` | `0xAC` |
| 1 | `0x0143` | `0x43` |
| 2 | `0x0286` | `0x86` |
| 3 | `0x03C5` | `0xC5` |
| 4 | `0x043D` | `0x3D` |
| 5 | `0x057E` | `0x7E` |
| 6 | `0x06BB` | `0xBB` |
| 7 | `0x07F8` | `0xF8` |
| 8 | `0x087A` | `0x7A` |
| 9 | `0x0939` | `0x39` |
| MAX (10) | `0x0A82` | `0x82` |

All eleven codes have been directly observed on air at address 0. Address 4 has
also been captured across all eleven speeds; addresses 6 and 7 partially.

---

## 6. Implementation guidance

### Why modulation is the critical detail

The demodulated bit stream looks exactly like a PWM on-off envelope, which
strongly suggests OOK. It is not. The failure mode is deceptive:

- An **OOK transmission** of the correct bit pattern deposits energy at
  433.92 MHz. It will **jam** the genuine remote — proving the RF chain works —
  while the fan decodes nothing at all.
- Every register, timing measurement and waveform comparison will look correct,
  because the *data* is correct. Only the carrier is wrong.

Diagnostic that settles it quickly:

- Received with a **2-FSK demodulator**: clean, decodable data.
- Received with an **amplitude (OOK) detector**: solid carrier — an entire
  packet of `0xFF`. This is the signature of constant-envelope FSK, and is
  easily mistaken for AGC saturation.

### Why bit rate matters

With OOK, the bit rate is only the resolution at which the envelope is sampled.
With FSK, the receiver recovers its clock at a specific rate. The waveform must
be transmitted at **4.8 kBaud**; the identical waveform at 19.2 kBaud does not
work.

### Building the waveform

Construct the waveform in **microseconds**, then sample it at the transmit bit
period. Do not express element widths as whole numbers of bits: at 4.8 kBaud a
531.8 µs mark is 2.55 bit periods, so rounding shifts every edge by up to 100 µs
and the error accumulates across a frame.

Sampling a microsecond-accurate waveform naturally produces alternating 2- and
3-sample runs whose average width is correct — which is exactly what genuine
recordings of the remote contain.

### CC1101 configuration

```
Frequency        433.92 MHz
Modulation       2-FSK
Deviation        35 kHz
Data rate        4.8 kBaud
Sync word        disabled (no preamble, no sync)
CRC              disabled
Data whitening   disabled
Packet length    fixed
Output power     10 dBm
```

Clock the whole 8-frame burst out of the FIFO as a single fixed-length packet
(~140 bytes at 4.8 kBaud). Splitting it across packets inserts several
milliseconds of dead air at each boundary, because every transmission costs an
IDLE → flush → fill FIFO → PLL calibrate → TX → IDLE round trip.

---

## 7. Security note

The protocol has **no rolling code, no encryption and no authentication**. The
16-bit payload is static per speed, so any transmission can be recorded and
replayed indefinitely. Anyone within radio range can capture a press and
reproduce it.

This is typical of inexpensive 433 MHz appliance remotes and is noted here for
completeness, not as a criticism — the threat model for a fan controller is
modest. It does mean the fan should not be relied upon for anything where
unauthorised control would matter.

---

## 8. Verification

Everything above was validated end to end:

1. **Decode** — captures decode to the code table, with consecutive button
   presses landing on consecutive speeds (`OFF`→`1`, `3`→`4`, `7`→`8`), each
   repeated 8 times. Wrong bit order or framing would not produce this.
2. **Checksum** — every decoded frame across 17 captures satisfies the checksum,
   with a single exception: one frame in the noisiest capture decoding as
   `0x06BA` where its seven siblings read `0x06BB`. The checksum correctly flags
   it as a one-bit error.
3. **Prediction** — three codes were predicted before being observed, then all
   three were observed matching exactly.
4. **Synthesis** — frames generated from this specification drive a real fan to
   any commanded speed.

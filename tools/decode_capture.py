#!/usr/bin/env python3
"""
Decoder for AC Infinity CTR63A remote captures.

Handles both capture formats:

  1. Pulse train (from rx_capture.ino):
         PULSES: +535 -515 +533 -1067 ...
     Signed microseconds; '+' = mark, '-' = space.

  2. Byte dump (rx_packet.ino, 2-FSK packet mode -- the working capture path):
         RSSI=-48 LEN=255 DATA=FF FF FF F8 00 07 39 ...
     Each bit is one demodulator sample at the configured data rate, so the
     byte stream is the signal oversampled ~2.5x.

Protocol (see protocol.md): 2-FSK at 433.92 MHz, 4.8 kBaud, PWM encoded,
16 bits MSB first, 8 repetitions per press. Mark ~532us; space ~518us = 0,
~1067us = 1.
"""

import argparse
import collections
import re
import sys

# --- Protocol constants ------------------------------------------------------

FRAME_BITS = 16

MARK_US = 535
SPACE_0_US = 515
SPACE_1_US = 1067

# Midpoint between the two space widths; anything longer is a '1'.
SPACE_THRESHOLD_US = (SPACE_0_US + SPACE_1_US) // 2

# A pulse must be within this factor of nominal to count as that symbol.
TOLERANCE = 0.45

# Legacy byte dumps were captured with the CC1101 data rate at 4.8 kBaud.
DEFAULT_SAMPLE_US = 1000.0 / 4.8

# Checksum: GF(2^8) multiply by 0x43, reduction polynomial 0x131, for speeds
# 1-9. Speeds 0 and 10 do not follow the rule and are stored explicitly.
CHECKSUM_MULTIPLIER = 0x43
CHECKSUM_POLY = 0x131
CHECKSUM_EXCEPTIONS = {0: 0xAC, 10: 0x82}

SPEED_OFF = 0
SPEED_MAX = 10


def gf_mul(a, b, poly=CHECKSUM_POLY):
    """Carry-less multiply in GF(2^8) with the given reduction polynomial."""
    result = 0
    while b:
        if b & 1:
            result ^= a
        b >>= 1
        a <<= 1
        if a & 0x100:
            a ^= poly
    return result


# --- Address handling --------------------------------------------------------
#
# Measured 2026-08-01 by changing the remote's DIP switches and capturing.
#
# Established: for a given address, ONE XOR constant applied to the addr=0
# checksum covers speeds 1-9. Speed 0 uses a different constant, and speed 10
# sometimes does too (it does at addr 7, but not at addr 4).
#
# Nearly established: for speeds 1-9 the constant is a carry-less (GF(2))
# multiply of the address by 0x31 -- confirmed for addresses 0, 1, 2, 4 and 6,
# including 6 = 4 XOR 2 combining linearly.
#
#     XOR(addr) = 0x31 (x) addr        holds for 0, 1, 2, 4, 6
#
# Address 7 is the sole exception: measured 0x2E where the rule gives 0x97.
# (Curiously 0x31 (x) 15 = 0x2E, but the frames genuinely carry 0111 and 16 bit
# pairs, so this is not a truncated address.) Addresses 3, 5 and 8 would show
# whether bit 0 simply fails to combine linearly with the higher bits, or
# something else is going on.
#
# The speed-0 constants are not linear at all and follow no rule found so far.
#
# Rather than guess, measured values are used where available.

ADDR_XOR_SPEEDS_1_9 = {0: 0x00, 1: 0x31, 2: 0x62, 4: 0xC4, 6: 0xA6, 7: 0x2E}  # measured
# Addresses 3 and 5 have never been captured. These come from the rule
# XOR(addr) = 0x31 (x) addr, which fits all five measured addresses except 7 --
# so treat them as likely but unconfirmed, and note that address 7 proves the
# rule is not universal.
ADDR_XOR_PREDICTED  = {3: 0x53, 5: 0xF5}
ADDR_XOR_SPEED_0    = {0: 0x00, 1: 0x9D, 2: 0xCE, 4: 0x68, 6: 0x0A, 7: 0x82}   # measured
ADDR_XOR_SPEED_10   = {0: 0x00, 2: 0x62, 4: 0xC4, 7: 0x13}   # measured


def checksum(speed, addr=0):
    """8-bit checksum for a speed (0-10) at an address (0-15).

    Addresses 0, 4, 6 and 7 are measured. Others fall back to the speeds-1-9
    constant if known, and raise otherwise rather than inventing a value.
    """
    base = (CHECKSUM_EXCEPTIONS[speed] if speed in CHECKSUM_EXCEPTIONS
            else gf_mul(CHECKSUM_MULTIPLIER, speed))
    if addr == 0:
        return base

    if speed == 0:
        table = ADDR_XOR_SPEED_0
    elif speed == 10:
        table = ADDR_XOR_SPEED_10
    else:
        table = ADDR_XOR_SPEEDS_1_9

    if addr in table:
        return base ^ table[addr]
    # Fall back to the speeds-1-9 constant, which is at least measured for this
    # address even if this particular speed was never captured.
    if addr in ADDR_XOR_SPEEDS_1_9:
        return base ^ ADDR_XOR_SPEEDS_1_9[addr]
    if addr in ADDR_XOR_PREDICTED and 1 <= speed <= 9:
        return base ^ ADDR_XOR_PREDICTED[addr]
    raise ValueError(
        f"address {addr} speed {speed} has never been captured and no rule "
        f"covers it. Measured addresses: {sorted(ADDR_XOR_SPEEDS_1_9)}.")


def build_payload(speed, addr=0):
    """16-bit payload: [addr:4][speed:4][checksum:8]."""
    return (addr << 12) | (speed << 8) | checksum(speed, addr)


def speed_name(speed):
    if speed == SPEED_OFF:
        return "OFF"
    if speed == SPEED_MAX:
        return "MAX"
    return str(speed)


def describe(payload):
    """Render a payload as a human label, flagging checksum mismatches."""
    addr = (payload >> 12) & 0x0F
    speed = (payload >> 8) & 0x0F
    csum = payload & 0xFF
    if speed > SPEED_MAX:
        return f"0x{payload:04X} (invalid speed {speed})"
    expected = checksum(speed, addr)
    label = f"0x{payload:04X} speed={speed_name(speed)}"
    if addr:
        label += f" addr={addr}"
    if csum != expected:
        label += f" BAD-CHECKSUM(got 0x{csum:02X}, want 0x{expected:02X})"
    return label


# --- Capture parsing ---------------------------------------------------------

# Pulses are (level, duration_us) where level is 1 for mark, 0 for space.

BYTE_DUMP_RE = re.compile(r"DATA=([0-9A-Fa-f]{2}(?:\s+[0-9A-Fa-f]{2})*)")
RSSI_RE = re.compile(r"RSSI=(-?\d+)")
PULSE_TRAIN_RE = re.compile(r"(?:PULSES:)?\s*((?:[+-]\d+\s*){8,})")


def pulses_from_byte_dump(hex_text, sample_us=DEFAULT_SAMPLE_US):
    """Expand an oversampled byte dump into pulses."""
    bits = "".join(f"{int(b, 16):08b}" for b in hex_text.split())
    pulses = []
    for run in re.findall(r"0+|1+", bits):
        pulses.append((int(run[0]), len(run) * sample_us))
    return pulses


def pulses_from_pulse_train(text):
    """Parse a signed-microsecond pulse train."""
    pulses = []
    for token in re.findall(r"[+-]\d+", text):
        value = int(token)
        pulses.append((1 if value > 0 else 0, abs(value)))
    return pulses


def parse_line(line, sample_us=DEFAULT_SAMPLE_US):
    """Return (pulses, rssi) for a capture line, or (None, None) if not one."""
    rssi_match = RSSI_RE.search(line)
    rssi = int(rssi_match.group(1)) if rssi_match else None

    byte_match = BYTE_DUMP_RE.search(line)
    if byte_match:
        return pulses_from_byte_dump(byte_match.group(1), sample_us), rssi

    train_match = PULSE_TRAIN_RE.search(line)
    if train_match:
        return pulses_from_pulse_train(train_match.group(1)), rssi

    return None, None


# --- Decoding ----------------------------------------------------------------


def _near(duration, nominal):
    return abs(duration - nominal) <= nominal * TOLERANCE


def decode_pulses(pulses):
    """Extract 16-bit frames from a pulse list.

    Walks mark/space pairs. A pair is a bit; anything else (sync bursts,
    inter-frame gaps, noise) terminates the current frame.
    """
    frames = []
    bits = []

    def flush():
        if len(bits) == FRAME_BITS:
            frames.append(int("".join(bits), 2))
        bits.clear()

    i = 0
    while i < len(pulses) - 1:
        mark_level, mark_us = pulses[i]
        space_level, space_us = pulses[i + 1]

        is_bit = (
            mark_level == 1
            and space_level == 0
            and _near(mark_us, MARK_US)
            and (_near(space_us, SPACE_0_US) or _near(space_us, SPACE_1_US))
        )

        if is_bit:
            bits.append("1" if space_us > SPACE_THRESHOLD_US else "0")
            if len(bits) == FRAME_BITS:
                flush()
            i += 2
        else:
            flush()
            i += 1

    flush()
    return frames


def structure_summary(pulses):
    """Summarise the non-bit elements -- sync bursts and inter-frame gaps."""
    long_marks = []
    long_spaces = []
    for level, duration in pulses:
        if duration < MARK_US * (1 + TOLERANCE):
            continue
        if level == 1:
            long_marks.append(duration)
        elif duration > SPACE_1_US * (1 + TOLERANCE):
            long_spaces.append(duration)
    return long_marks, long_spaces


# --- Reporting ---------------------------------------------------------------


def decode_file(path, sample_us=DEFAULT_SAMPLE_US, verbose=False):
    totals = collections.Counter()
    line_count = 0

    with open(path, errors="replace") as handle:
        for line in handle:
            pulses, rssi = parse_line(line, sample_us)
            if not pulses:
                continue
            frames = decode_pulses(pulses)
            if not frames:
                continue

            line_count += 1
            counts = collections.Counter(frames)
            totals.update(frames)

            rssi_text = f"RSSI={rssi:4d} " if rssi is not None else ""
            summary = ", ".join(
                f"{describe(payload)} x{n}" for payload, n in counts.most_common()
            )
            print(f"  {rssi_text}{len(frames):3d} frames: {summary}")

            if verbose:
                marks, spaces = structure_summary(pulses)
                if marks:
                    print(f"        long marks:  {[round(m) for m in marks]}")
                if spaces:
                    print(f"        long spaces: {[round(s) for s in spaces]}")

    return totals, line_count


def selftest():
    """Verify the checksum rule reproduces every code observed on the air."""
    observed = {
        0: 0x00AC, 1: 0x0143, 3: 0x03C5, 4: 0x043D,
        6: 0x06BB, 7: 0x07F8, 8: 0x087A, 10: 0x0A82,
    }
    failures = []
    for speed, expected in observed.items():
        got = build_payload(speed)
        status = "ok" if got == expected else "FAIL"
        if got != expected:
            failures.append(speed)
        print(f"  speed {speed_name(speed):>3}: 0x{got:04X} (expected 0x{expected:04X}) {status}")

    print("\n  Predicted (never captured, derived from the GF(2^8) rule):")
    for speed in (2, 5, 9):
        print(f"  speed {speed_name(speed):>3}: 0x{build_payload(speed):04X}")

    return not failures


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("files", nargs="*", help="capture files to decode")
    parser.add_argument("--sample-us", type=float, default=DEFAULT_SAMPLE_US,
                        help="sample period for legacy byte dumps (default: %(default).1f)")
    parser.add_argument("--table", action="store_true", help="print the full code table and exit")
    parser.add_argument("--addr", type=int, default=0, choices=range(16), metavar="0-15",
                        help="address nibble for --table (only 0 is verified)")
    parser.add_argument("--selftest", action="store_true", help="verify checksum rule vs captured codes")
    parser.add_argument("-v", "--verbose", action="store_true", help="also report sync/gap structure")
    args = parser.parse_args()

    if args.table:
        if args.addr:
            print(f"address {args.addr} -- UNVERIFIED: no capture contains a "
                  f"non-zero address.\nThe address term is conjecture; see protocol.md.\n")
        print("speed  payload  checksum")
        for speed in range(SPEED_MAX + 1):
            print(f"{speed_name(speed):>5}   0x{build_payload(speed, args.addr):04X}"
                  f"   0x{checksum(speed, args.addr):02X}")
        return 0

    if args.selftest:
        print("Checksum rule vs codes decoded from real captures:")
        return 0 if selftest() else 1

    if not args.files:
        parser.error("no capture files given (use --table or --selftest for offline checks)")

    grand_total = collections.Counter()
    for path in args.files:
        print(f"\n{path}")
        totals, lines = decode_file(path, args.sample_us, args.verbose)
        if not lines:
            print("  no decodable frames")
        else:
            summary = ", ".join(
                f"{describe(payload)} x{n}" for payload, n in sorted(totals.items())
            )
            print(f"  -- {lines} capture(s): {summary}")
        grand_total.update(totals)

    if len(args.files) > 1:
        print("\nAll files:")
        for payload, n in sorted(grand_total.items()):
            print(f"  {describe(payload)} x{n}")

    return 0


if __name__ == "__main__":
    sys.exit(main())

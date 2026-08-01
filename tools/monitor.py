#!/usr/bin/env python3
"""
Serial logger for the ESP32 sketches.

Do NOT reset the board after flashing. arduino-cli already resets it on upload,
and pulsing EN again on this board drops it into a "try 0x4008059c" loop -- the
"boot loop" that earlier sessions misdiagnosed as a flashing or wiring problem.
--reset is therefore off by default; leave it off unless you know you need it.

This also never asserts DTR: DTR drives GPIO0 here, so leaving it asserted holds
the chip in the ROM bootloader.

Usage:
    tools/monitor.py                          # watch, ctrl-C to stop
    tools/monitor.py -t 60 -o captures/x.txt  # log 60s to a file
    tools/monitor.py -s 5                     # send "5" then watch
"""

import argparse
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial not installed: pip install pyserial")

DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUD = 115200


def open_port(port, baud, reset):
    ser = serial.Serial(port, baud, timeout=0.1)

    # Park both lines de-asserted: EN released, GPIO0 high (normal boot).
    ser.dtr = False
    ser.rts = False
    time.sleep(0.05)

    if reset:
        ser.rts = True      # EN low -> in reset
        time.sleep(0.1)
        ser.rts = False     # release -> normal boot, since DTR stayed low
        time.sleep(0.3)

    ser.reset_input_buffer()
    return ser


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("-p", "--port", default=DEFAULT_PORT)
    parser.add_argument("-b", "--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("-t", "--timeout", type=float, help="stop after N seconds")
    parser.add_argument("-o", "--output", help="also write output to this file")
    # NOTE: a value starting with '-' must use the '--send=...' form; argparse
    # otherwise treats it as a flag and exits. Prefer --gain for the common case.
    parser.add_argument("-s", "--send", help="send this string once after connecting "
                                             "(use --send=X form if X starts with '-')")
    parser.add_argument("--gain", type=int, choices=range(4),
                        help="set rx_capture's MAX_DVGA_GAIN (0=most sensitive, "
                             "3=quietest). Opening the port resets the board, so "
                             "gain always starts at 3.")
    parser.add_argument("--reset", action="store_true",
                        help="pulse EN before listening (normally unnecessary and "
                             "can trigger a boot loop -- see module docstring)")
    args = parser.parse_args()

    ser = open_port(args.port, args.baud, args.reset)
    sink = open(args.output, "w") if args.output else None

    if args.gain is not None:
        # rx_capture boots at MAX_DVGA_GAIN=3; '+' steps towards more sensitive.
        time.sleep(2.5)                      # let the sketch finish booting
        ser.write(b"+" * (3 - args.gain))
        ser.flush()

    if args.send:
        ser.write(args.send.encode())
        ser.flush()

    start = time.time()
    try:
        while True:
            if args.timeout and time.time() - start >= args.timeout:
                break
            chunk = ser.read(ser.in_waiting or 1)
            if not chunk:
                continue
            text = chunk.decode("utf-8", errors="replace")
            sys.stdout.write(text)
            sys.stdout.flush()
            if sink:
                sink.write(text)
                sink.flush()
    except KeyboardInterrupt:
        pass
    finally:
        if sink:
            sink.close()
        ser.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())

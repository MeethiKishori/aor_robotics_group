import argparse
import os
import sys
import time


THIS_DIR = os.path.dirname(os.path.abspath(__file__))
PYTHON_DIR = os.path.dirname(THIS_DIR)
if PYTHON_DIR not in sys.path:
    sys.path.insert(0, PYTHON_DIR)

from actuators.usb_alert import UsbAlertOutput


def parse_hex_payload(value):
    cleaned = value.replace(" ", "").replace(":", "")
    if cleaned.startswith("0x"):
        cleaned = cleaned[2:]
    if len(cleaned) % 2 != 0:
        raise ValueError(f"Hex payload must contain an even number of digits: {value}")
    return bytes.fromhex(cleaned)


def main():
    parser = argparse.ArgumentParser(description="Probe USB signal tower commands over /dev/ttyUSB0")
    parser.add_argument("commands", nargs="+",
                        help="Commands to send. In text mode these are plain strings. In hex mode use bytes like AA55 or 0xAA55.")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Serial device path")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    parser.add_argument("--hex", action="store_true", help="Interpret commands as hexadecimal byte strings")
    parser.add_argument("--newline", action="store_true", help="Append newline in text mode")
    parser.add_argument("--repeat", type=int, default=1, help="How many times to send each command")
    parser.add_argument("--delay", type=float, default=1.0, help="Delay in seconds between commands")
    args = parser.parse_args()

    writer = UsbAlertOutput(port=args.port, baudrate=args.baud)

    try:
        for _ in range(max(1, args.repeat)):
            for command in args.commands:
                if args.hex:
                    payload = parse_hex_payload(command)
                    shown = payload.hex(" ")
                else:
                    payload = command + ("\n" if args.newline else "")
                    shown = repr(payload)

                print(f"Sending {shown} to {args.port}")
                writer.send_payload(payload)
                time.sleep(max(0.0, args.delay))
    finally:
        writer.close()


if __name__ == "__main__":
    main()
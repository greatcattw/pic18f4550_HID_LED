#!/usr/bin/env python3
import sys
import os

def main():
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} <hidraw_device:/dev/hidrawX> <led_byte:hex or dec>", file=sys.stderr)
        sys.exit(1)

    hidraw_device = sys.argv[1]

    try:
        # 支援十進位與十六進位，例如：
        # 255, 0xff, 0xAA
        led_byte = int(sys.argv[2], 0)
    except ValueError:
        print("Invalid led_byte value. Please use decimal or hex, e.g. 255 or 0xff", file=sys.stderr)
        sys.exit(1)

    if led_byte < 0 or led_byte > 255:
        print("led_byte must be between 0 and 255", file=sys.stderr)
        sys.exit(1)

    # 準備寫入的 2 bytes
    buf = bytes([led_byte, 0x00])

    try:
        # 開啟 HID 裝置，只寫入
        fd = os.open(hidraw_device, os.O_WRONLY)

        try:
            written = os.write(fd, buf)

            if written != len(buf):
                print(f"Warning: only wrote {written} bytes", file=sys.stderr)

            print(f"Successfully wrote to the HID device {hidraw_device}: {led_byte:02X}")

        finally:
            os.close(fd)

    except PermissionError:
        print(f"Permission denied: {hidraw_device}", file=sys.stderr)
        print("Try running with sudo, or adjust udev permissions.", file=sys.stderr)
        sys.exit(1)

    except FileNotFoundError:
        print(f"Device not found: {hidraw_device}", file=sys.stderr)
        sys.exit(1)

    except OSError as e:
        print(f"Failed to write to HID device: {e}", file=sys.stderr)
        sys.exit(1)

if __name__ == "__main__":
    main()

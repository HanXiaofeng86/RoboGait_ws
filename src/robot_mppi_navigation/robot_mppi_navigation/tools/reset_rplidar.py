"""Reset RPLidar via SLAMTEC protocol command 0xA5 0x40."""

import sys
import time

import serial


def reset_lidar(port: str = '/dev/rplidar', baud: int = 1000000) -> None:
    ser = serial.Serial(port, baud, timeout=1)
    try:
        ser.write(b'\xA5\x40')
        ser.flush()
        time.sleep(2.5)
        ser.reset_input_buffer()
    finally:
        ser.close()


def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    port = argv[0] if argv else '/dev/rplidar'
    try:
        reset_lidar(port)
    except serial.SerialException as exc:
        print(f'Failed to open {port}: {exc}', file=sys.stderr)
        return 1
    print(f'Lidar reset sent on {port}. Wait 2–3s, then restart the driver.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

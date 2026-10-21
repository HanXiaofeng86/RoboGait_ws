"""Reset RPLidar via SLAMTEC protocol command 0xA5 0x40.

Some units stay at health=2 after a single RESET; send twice with a longer settle.
"""

import sys
import time

import serial


def reset_lidar(
    port: str = '/dev/rplidar',
    baud: int = 1000000,
    rounds: int = 2,
    settle_s: float = 3.0,
) -> None:
    ser = serial.Serial(port, baud, timeout=1)
    try:
        for i in range(rounds):
            ser.write(b'\xA5\x40')
            ser.flush()
            time.sleep(settle_s)
            ser.reset_input_buffer()
            if i + 1 < rounds:
                print(f'Lidar RESET {i + 1}/{rounds} on {port}, settling…')
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
    print(
        f'Lidar reset done on {port} (2× 0xA5 0x40). '
        'If health is still 2, unplug/replug lidar USB power and retry.'
    )
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

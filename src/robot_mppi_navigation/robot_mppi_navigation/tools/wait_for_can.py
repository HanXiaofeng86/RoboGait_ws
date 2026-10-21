"""Wait until can0 is UP and at least one frame arrives.

The chassis driver FATAL-exits if it binds the socket before the bus has
data (common right after Ctrl+C). Exit 0 even on timeout so launch continues.
"""

from __future__ import annotations

import subprocess
import sys
import time


def _can_up() -> bool:
    result = subprocess.run(
        ['ip', '-br', 'link', 'show', 'can0'],
        capture_output=True, text=True, check=False)
    return result.returncode == 0 and 'UP' in result.stdout


def _try_start_can() -> None:
    subprocess.run(
        ['systemctl', 'start', 'can.service'],
        capture_output=True, check=False)


def _has_frame(timeout_ms: int = 800) -> bool:
    result = subprocess.run(
        ['candump', '-n', '1', '-T', str(timeout_ms), 'can0'],
        capture_output=True, check=False)
    return result.returncode == 0


def main(argv=None) -> int:
    del argv
    if not _can_up():
        print('wait_for_can: can0 not UP, trying can.service…')
        _try_start_can()
        time.sleep(0.4)

    deadline = time.time() + 8.0
    while time.time() < deadline:
        if _can_up() and _has_frame(700):
            print('wait_for_can: can0 UP and receiving frames')
            time.sleep(0.8)
            return 0
        if not _can_up():
            _try_start_can()
        time.sleep(0.25)

    print(
        'wait_for_can: timeout (can0 DOWN or no frames). '
        'If the driver FATAL-exits, run: sudo systemctl start can.service',
        file=sys.stderr)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

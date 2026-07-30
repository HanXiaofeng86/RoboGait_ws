"""Stop conditions for the safety gate."""

from dataclasses import dataclass
from typing import List, Optional


@dataclass
class SafetyState:
    last_cmd_time: Optional[float] = None
    last_odom_time: Optional[float] = None
    estop_active: bool = False
    cmd_timeout: float = 0.5
    odom_timeout: float = 0.5

    def stop_reasons(self, now: float) -> List[str]:
        reasons: List[str] = []
        if self.estop_active:
            reasons.append('emergency stop active')
        if self.last_cmd_time is None:
            reasons.append('no command received yet')
        elif now - self.last_cmd_time > self.cmd_timeout:
            reasons.append(
                f'command stale ({now - self.last_cmd_time:.2f}s > '
                f'{self.cmd_timeout:.2f}s)')
        if self.last_odom_time is None:
            reasons.append('no odometry received yet')
        elif now - self.last_odom_time > self.odom_timeout:
            reasons.append(
                f'odometry stale ({now - self.last_odom_time:.2f}s > '
                f'{self.odom_timeout:.2f}s)')
        return reasons

    def is_safe(self, now: float) -> bool:
        return not self.stop_reasons(now)

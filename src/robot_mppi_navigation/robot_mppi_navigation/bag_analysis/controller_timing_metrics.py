"""Controller timing metrics from command message arrival times.

At controller_frequency 20 Hz the expected period is 0.05 s. A gap much
larger than the period indicates a missed control deadline.
"""

import numpy as np

EXPECTED_PERIOD = 0.05  # s (20 Hz)


def timing(cmd_times, expected_period: float = EXPECTED_PERIOD,
           miss_factor: float = 1.5):
    if len(cmd_times) < 2:
        return {'mean_period': float('nan'), 'max_period': float('nan'),
                'missed_deadlines': 0, 'samples': len(cmd_times)}
    periods = np.diff(cmd_times)
    return {
        'mean_period': float(np.mean(periods)),
        'max_period': float(np.max(periods)),
        'achieved_frequency': float(1.0 / np.mean(periods)),
        'missed_deadlines': int(np.sum(periods > miss_factor * expected_period)),
        'samples': len(cmd_times),
    }

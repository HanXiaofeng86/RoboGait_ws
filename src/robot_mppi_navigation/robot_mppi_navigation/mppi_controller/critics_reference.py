"""Reference documentation for the 8 MPPI critics (design doc section 8).

Each entry: mathematical purpose, activation condition, configured weight
(mirrors nav2_pure_mppi_params.yaml), expected effect, failure mode.
Do not judge critic importance from its weight alone; raw cost scales differ.
"""

CRITICS = {
    'ConstraintCritic': {
        'purpose': 'Penalizes sampled velocities outside the feasible set '
                   '(vx/wz limits); keeps trajectories dynamically valid.',
        'activation': 'Always active.',
        'weight': 4.0,
        'expected_effect': 'Commands stay within vx_min/vx_max, wz_max.',
        'failure_mode': 'Weight too low: infeasible commands win and get '
                        'clipped, causing tracking error.',
    },
    'ObstaclesCritic': {
        'purpose': 'Penalizes trajectories entering high-cost costmap cells; '
                   'collision cost is effectively a hard constraint.',
        'activation': 'Always active; repulsion only near obstacles.',
        'weight': 'critical_weight 20.0, repulsion_weight 1.5',
        'expected_effect': 'Collision-free candidates; smooth clearance.',
        'failure_mode': 'Repulsion too high: robot stalls or oscillates in '
                        'narrow passages.',
    },
    'GoalCritic': {
        'purpose': 'Rewards reducing Euclidean distance to the goal pose.',
        'activation': 'Within threshold_to_consider (1.4 m) of the goal.',
        'weight': 5.0,
        'expected_effect': 'Committed final approach to the goal.',
        'failure_mode': 'Too high: cuts corners off the path near the goal.',
    },
    'GoalAngleCritic': {
        'purpose': 'Rewards aligning heading with the goal yaw.',
        'activation': 'Within threshold_to_consider (0.5 m) of the goal.',
        'weight': 3.0,
        'expected_effect': 'Correct final heading at the goal.',
        'failure_mode': 'Too high: early spinning instead of positioning.',
    },
    'PathAlignCritic': {
        'purpose': 'Penalizes lateral offset between trajectory and path.',
        'activation': 'Deactivates near goal (threshold 0.5 m) or when path '
                      'is blocked above max_path_occupancy_ratio.',
        'weight': 8.0,
        'expected_effect': 'Low lateral path error on straights and curves.',
        'failure_mode': 'Too high: hugs the path into obstacles, fights the '
                        'obstacle critic.',
    },
    'PathFollowCritic': {
        'purpose': 'Rewards progress toward a point ahead on the path; the '
                   'main forward-driving term.',
        'activation': 'Deactivates near goal (threshold 1.4 m).',
        'weight': 10.0,
        'expected_effect': 'Steady forward progress along the path.',
        'failure_mode': 'Too high: overshoots corners due to speed pressure.',
    },
    'PathAngleCritic': {
        'purpose': 'Penalizes heading error relative to path direction; '
                   'prepares turns before corners.',
        'activation': 'When heading error to furthest point > threshold.',
        'weight': 1.0,
        'expected_effect': 'Earlier turn-in on 90-degree corners.',
        'failure_mode': 'Too high: wavy heading on straight segments.',
    },
    'PreferForwardCritic': {
        'purpose': 'Penalizes reverse motion; with vx_min=0 acts as a soft '
                   'regularizer for forward progress.',
        'activation': 'Beyond threshold_to_consider (0.5 m) from goal.',
        'weight': 5.0,
        'expected_effect': 'Forward-only tracking behaviour.',
        'failure_mode': 'Too high with tight goals: refuses small backing '
                        'adjustments (irrelevant while vx_min=0).',
    },
}

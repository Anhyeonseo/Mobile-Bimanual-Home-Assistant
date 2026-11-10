"""L2 units for the installed single-turn encoders and Mini 1 lift."""
COUNTS_PER_TURN = 4096
LIFT_METERS_PER_TURN = 0.084


def observed_delta(raw, previous):
    """Signed difference between consecutive measured angles; never a motion plan."""
    half = COUNTS_PER_TURN // 2
    return (raw - previous + half) % COUNTS_PER_TURN - half


def lift_rise_m(relative_counts, meters_per_turn=LIFT_METERS_PER_TURN):
    """Up is negative on the installed ID10 axis. Requires a known bottom."""
    return -relative_counts * meters_per_turn / COUNTS_PER_TURN


def lift_target_counts(height_m):
    return -round(height_m / LIFT_METERS_PER_TURN * COUNTS_PER_TURN)

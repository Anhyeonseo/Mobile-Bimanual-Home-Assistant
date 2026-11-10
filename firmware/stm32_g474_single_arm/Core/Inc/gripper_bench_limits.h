#ifndef GRIPPER_BENCH_LIMITS_H
#define GRIPPER_BENCH_LIMITS_H
/* Left ID6 hand-measured range 1936..3182, raw decreases towards closed.
 * Commissioning caps in servo units, not contact-force calibration. */
enum { GRIP_ID = 6, GRIP_LOW = 1950, GRIP_HIGH = 3172, GRIP_SPEED = 75,
       GRIP_CAP = 80, GRIP_STEP = 6, GRIP_LEASE_MS = 300, GRIP_TRIAL_MS = 24000,
       GRIP_HOLD_MIN_MS = 500, GRIP_HOLD_MAX_MS = 3000 };
#endif

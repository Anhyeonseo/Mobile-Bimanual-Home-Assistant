#ifndef TAUGHT_POSE_GUARD_H
#define TAUGHT_POSE_GUARD_H
#include <stdbool.h>
#include <stdint.h>
/* Each finite segment must finish at the current or an adjacent taught pose.
 * Called for every 5 ms output BEFORE dispatch. Fault never advances state. */
void TaughtPoseGuard_Reset(void);
bool TaughtPoseGuard_Check(const int32_t q[12],bool terminal);
#endif

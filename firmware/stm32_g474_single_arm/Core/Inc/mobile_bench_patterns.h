#ifndef MOBILE_BENCH_PATTERNS_H
#define MOBILE_BENCH_PATTERNS_H
#include <stdbool.h>
/* Operator: 7 left, 8 right, 9 front; +raw clockwise from outside.
 * Nominal 120-degree candidate ratios; no calibrated m/s claim. */
static inline bool MobileBench_Pattern(char motion, int speeds[3]) {
    switch (motion) {
    case 'F': speeds[0]=-200; speeds[1]=200; speeds[2]=0; break;
    case 'B': speeds[0]=200; speeds[1]=-200; speeds[2]=0; break;
    case 'L': speeds[0]=-100; speeds[1]=-100; speeds[2]=200; break;
    case 'R': speeds[0]=100; speeds[1]=100; speeds[2]=-200; break;
    case 'A': speeds[0]=200; speeds[1]=200; speeds[2]=200; break;
    case 'D': speeds[0]=-200; speeds[1]=-200; speeds[2]=-200; break;
    default: return false;
    }
    return true;
}
#endif

# Local stream framing changes

Based on ROBOTIS-GIT/hls_lfcd_lds_driver tag `2.1.1`, commit
`fa4608d80b6005e0ca63197f79ad6d09381f84b3`. Original BSD notices and LICENSE retained.
Source: https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver/tree/2.1.1

The upstream fixed 2520-byte read consumes the next rotation's sector zero when
one sector is absent. Its subsequent synchronization skips that whole rotation.
A real 2026-11-12 capture reproduces this twice, producing ~0.398-second gaps.

This overlay reuses the upstream ROS executable, topic, parameters, angular
ordering and sensor start/pause commands. Only framing and transport polling are
changed. It parses checksum-verified 42-byte sectors and preserves a next-rotation
boundary for that next scan. Up to two missing sectors are NaN, never zero-range
clearance, infinity, interpolation, or data copied from another scan. Larger
loss, invalid rotation speed and scans older than 0.30 seconds are rejected.
No timer republishes old scans. More than one rotation of kernel backlog is
flushed rather than stamped fresh. Polling waits at most 100 ms, serial access
is exclusive, and shutdown does not call tcdrain.

Framing/cadence verification is not a calibration of exposure timestamps or
moving-scan deskew. Upstream positive time_increment and publication timestamp
semantics are retained for this stationary commissioning stage. Mounting TF,
range accuracy, moving acquisition timing and localization remain separate gates.
The existing navigation health thresholds must not be relaxed to hide gaps.

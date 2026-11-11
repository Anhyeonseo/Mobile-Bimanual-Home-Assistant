# Local stream framing changes

Based on ROBOTIS-GIT/hls_lfcd_lds_driver tag `2.1.1`, commit
`fa4608d80b6005e0ca63197f79ad6d09381f84b3`. Original BSD notices and LICENSE retained.
Source: https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver/tree/2.1.1

The upstream fixed 2520-byte read consumes the next rotation's sector zero when
one sector is absent. Its subsequent synchronization skips that whole rotation.
A real 2026-11-12 capture reproduces this twice, producing ~0.398-second gaps.

This overlay reuses the upstream ROS executable, topic, parameters, angular
ordering and sensor start/pause commands. It parses checksum-verified 42-byte
sectors and preserves a next-rotation
boundary for that next scan. Up to two missing sectors are NaN, never zero-range
clearance, infinity, interpolation, or data copied from another scan. Larger
loss, invalid rotation speed and scans older than 0.30 seconds are rejected.
No timer republishes old scans. More than one rotation of kernel backlog is
flushed rather than stamped fresh. Polling waits at most 100 ms, serial access
is exclusive, and shutdown does not call tcdrain.

The second preparation change fixes the reversed ray-time order. The increasing
angle array is retained, so `time_increment` is negative: index zero was acquired
last. The header uses an explicit **host serial estimate**, subtracting packet
serialization time (42 bytes, 8N1, 230400 baud) from the last sector's receipt.
An absent final sector is extrapolated using measured RPM and its sector index,
not the next revolution's timestamp. Conversion to ROS time preserves age rather
than stamping publication as acquisition. A future or stale estimate is rejected.

This estimate assumes a packet is emitted after its six rays; device processing
and USB delay are not measured. It is not hardware synchronization, precise
exposure timing, motion compensation, or physical moving-mapping qualification.
Negative ray-time offsets follow the existing reversed storage order; changing
angle order instead would change the mapping consumer's angular contract.
Contract: https://github.com/ros2/common_interfaces/blob/jazzy/sensor_msgs/msg/LaserScan.msg
Projection reference: https://github.com/ros-perception/laser_geometry/blob/ros2/src/laser_geometry.cpp
SLAM reference: https://github.com/SteveMacenski/slam_toolbox/blob/jazzy/src/laser_utils.cpp
Mounting accuracy, range accuracy, moving timing and localization remain separate gates.
The existing navigation health thresholds must not be relaxed to hide gaps.

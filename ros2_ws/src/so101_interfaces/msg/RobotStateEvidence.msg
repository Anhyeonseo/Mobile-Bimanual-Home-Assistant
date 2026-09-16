# Independently measured, synchronized oldest source time; never command echoes.
std_msgs/Header header
string profile_sha256
uint32 sequence
uint32 boot_id
float64[3] xy_yaw
float64[3] base_velocity
float64[12] joints_rad
int32 lift_um
float64 lift_velocity
bool healthy
bool localized
bool lift_homed
bool lift_supported
bool surface_valid
bool collision_scene_ready
bool arm_clear
# Independent reachability/collision validation for the named observed target.
bool reachable
bool collision_checked
string checked_capture_id
string checked_scene_revision

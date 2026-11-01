# Independently measured, synchronized oldest source time; never command echoes.
std_msgs/Header header
string profile_sha256
uint32 sequence
uint32 boot_id
float64[3] xy_yaw
float64[3] base_velocity
# Named installed joints: six for a single arm, twelve for both arms.
# Empty names are retained only for the legacy twelve-joint profile. Receivers
# enforce the configured exact count/order; missing joints are never zero-filled.
string[<=12] joint_names
float64[<=12] joints_rad
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

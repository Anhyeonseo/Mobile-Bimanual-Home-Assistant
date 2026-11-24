# Source the robot's existing Jazzy installation for this process and its children.
pick_source_ros() {
    local runtime="${PICK_JETSON_HOME:-$HOME}/robot/shared/runtime/s4-jazzy"
    if [[ -f "$runtime/env.sh" ]]; then
        source "$runtime/env.sh" || return
        if [[ -f "$runtime/install/setup.bash" ]]; then
            source "$runtime/install/setup.bash" || return
        fi
        printf 'ROS setup: %s\n' "$runtime/env.sh" >&2
    elif [[ "${ROS_DISTRO:-}" != jazzy && -f /opt/ros/jazzy/setup.bash ]]; then
        source /opt/ros/jazzy/setup.bash || return
        printf 'ROS setup: %s\n' /opt/ros/jazzy/setup.bash >&2
    fi
}

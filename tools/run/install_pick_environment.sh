#!/usr/bin/env bash
# Run on the target if it lacks a combined Jazzy + perception Python.
set -eo pipefail
pick_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$pick_root"
source /etc/os-release
if [[ "$ID" != ubuntu || "$VERSION_ID" != 24.04 || ! -f SHA256SUMS ]]; then
    echo 'Run inside the prepared kit on Ubuntu 24.04.' >&2
    exit 2
fi
if ! /usr/bin/python3 -c 'import sys; sys.exit(sys.version_info[:2] != (3,12))'; then
    echo 'ROS Jazzy on Noble needs the system Python 3.12.' >&2
    exit 2
fi
sudo apt-get update
if ! apt-cache show ros-jazzy-ros-base >/dev/null 2>&1; then
    sudo apt-get install -y software-properties-common curl ca-certificates
    sudo add-apt-repository -y universe
    pick_setup_dir="$(mktemp -d)"
    trap 'rm -rf -- "$pick_setup_dir"' EXIT
    pick_release="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest |
        /usr/bin/python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])')"
    curl -fsSL "https://github.com/ros-infrastructure/ros-apt-source/releases/download/$pick_release/ros2-apt-source_${pick_release}.noble_all.deb" \
        -o "$pick_setup_dir/ros2-apt-source.deb"
    sudo dpkg -i "$pick_setup_dir/ros2-apt-source.deb"
    sudo apt-get update
fi
sudo apt-get install -y python3-venv python3-scipy python3-opencv python3-serial python3-yaml \
    ros-jazzy-ros-base ros-jazzy-xacro ros-jazzy-moveit-ros-move-group \
    ros-jazzy-moveit-kinematics ros-jazzy-moveit-planners-ompl ros-jazzy-robot-state-publisher
/usr/bin/python3 -m venv --system-site-packages "$pick_root/.venv"
pick_python="$pick_root/.venv/bin/python"
"$pick_python" -m pip install --index-url https://download.pytorch.org/whl/cpu 'torch==2.10.0+cpu' 'torchvision==0.25.0+cpu'
"$pick_python" -m pip install 'numpy>=1.26,<2' 'Pillow>=10,<13' 'transformers==4.57.6' 'pyrealsense2==2.58.4.10922'
echo 'Environment installed. Run bash tools/run/pick_kit.sh preflight.'

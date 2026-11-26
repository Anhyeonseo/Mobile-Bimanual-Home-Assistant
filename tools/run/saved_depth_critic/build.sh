#!/usr/bin/env bash
# Build the isolated plugin prefix using the same native Nav2 runtime as the station.
set -euo pipefail
src="$(cd "$(dirname "$0")" && pwd)"
prefix="${1:?supply output prefix}"
ros="${2:?supply ROS installation prefix}"
mkdir -p "$prefix/lib" "$prefix/share/ament_index/resource_index/"{dwb_core__pluginlib__plugin,packages} "$prefix/share/cabinet_saved_depth"
incs=(-I"$ros/include")
for d in "$ros"/include/*; do [[ -d "$d" ]] && incs+=("-I$d"); done
sysroot="$(realpath "$ros/../../..")"
for d in "$sysroot/usr/include" "$sysroot/usr/include/eigen3" /usr/include/eigen3; do [[ -d "$d" ]] && incs+=("-I$d"); done
g++ -std=c++17 -O2 -shared -fPIC "${incs[@]}" "$src/critic.cpp" -L"$ros/lib" -lclass_loader -lrclcpp -lrclcpp_lifecycle -o "$prefix/lib/libcabinet_saved_depth.so"
printf '%s\n' '<library path="cabinet_saved_depth"><class type="cabinet::SavedDepthCritic" base_class_type="dwb_core::TrajectoryCritic"/></library>' > "$prefix/share/cabinet_saved_depth/critic.xml"
printf '%s\n' 'share/cabinet_saved_depth/critic.xml' > "$prefix/share/ament_index/resource_index/dwb_core__pluginlib__plugin/cabinet_saved_depth"
cp "$src/package.xml" "$prefix/share/cabinet_saved_depth/package.xml"
touch "$prefix/share/ament_index/resource_index/packages/cabinet_saved_depth"

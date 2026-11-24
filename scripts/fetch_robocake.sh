#!/usr/bin/env bash
set -euo pipefail

src_dir=${1:?Usage: bash fetch_robocake.sh /path/to/catkin_ws/src}
checkout="$src_dir/robocake_beginner"

fetch_revision() {
    local url=$1 directory=$2 revision=$3
    if [[ ! -e "$directory" ]]; then
        git clone --no-checkout "$url" "$directory"
        git -C "$directory" checkout --detach "$revision"
    elif [[ ! -d "$directory/.git" ]] ||
         [[ "$(git -C "$directory" rev-parse HEAD)" != "$revision" ]]; then
        printf 'Expected checkout at %s, revision %s\n' "$directory" "$revision" >&2
        return 1
    fi
}

mkdir -p "$src_dir"
fetch_revision https://github.com/robocake/beginner.git "$checkout" ef4893ae410cc611604126bd3c798c527565b61d
fetch_revision https://github.com/robocake/maps.git "$src_dir/robocake_maps" 0bbc1962717d47aee2ebc031ab78c3a2d685e6bb

# The upstream targets were written for Gazebo 7 and ROS Kinetic.
# Use Gazebo 11's C++ toolchain and link the sensor plugin directly to ROS.
cmake="$checkout/robocake_gazebo/CMakeLists.txt"
plugin="$checkout/robocake_gazebo/src/reflectance_sensor_plugin.cpp"
if grep -q -- '-std=c++11' "$cmake"; then
    sed -i -e 's/-std=c++11/-std=c++17/' \
        -e '/^[[:space:]]*CameraPlugin$/a\  ${catkin_LIBRARIES}' "$cmake"
fi
if ! grep -q '^#include <numeric>' "$plugin"; then
    sed -i '1i#include <numeric>' "$plugin"
fi

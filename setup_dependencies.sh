#!/bin/bash
# Clone pinned third-party dependencies into the expected workspace paths.
set -e

clone_pin () {  # url path pin
    if [ ! -d "$2/.git" ]; then
        git clone "$1" "$2"
    fi
    git -C "$2" fetch --tags origin
    git -C "$2" checkout "$3"
}

clone_pin https://github.com/NVlabs/curobo.git                     spot-ros2_ws/src/curobo            v0.7.7
clone_pin https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox.git isaac-ros_ws/src/isaac_ros_nvblox  7908a183acf84f4f1ab3fda7b6d6caf3eefc1f78
clone_pin https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git isaac-ros_ws/src/isaac_ros_common  fcf4d9e17f8f0a7f47f1d22d6a18421ce3768c01
clone_pin https://github.com/bdaiinstitute/spot_ros2.git           spot-ros2_ws/src/spot_ros2         spot-sdk-4.0.0
clone_pin https://github.com/stereolabs/zed-ros2-wrapper.git       zed_ws/src/zed-ros2-wrapper        e9f54907fbf41ee9ce5d54f3bb694af93dad8bb3
clone_pin https://github.com/stereolabs/zed-ros2-interfaces.git    zed_ws/src/zed-ros2-interfaces     cfffb8854a28b1143731ff7b75ba18379335e290

echo "Dependencies ready."

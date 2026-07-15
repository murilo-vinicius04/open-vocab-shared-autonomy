from setuptools import find_packages, setup
from glob import glob
import os

package_name = "spot_operation_ros2"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yml")),
        (
            os.path.join("share", package_name, "config/spheres"),
            glob("config/spheres/*.yml"),
        ),
    ],
    install_requires=[
        "setuptools",
        "opencv-python",
        "ultralytics",
        "numpy",
        "open3d",
    ],
    zip_safe=True,
    maintainer="Anonymous",
    maintainer_email="anonymous@example.com",
    description="ROS2 package for coordinator, planning, and control node operations on Spot robot using cuRobo MPC and SAM2 tracking.",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "gripper_controller = spot_operation_ros2.gripper_controller:main",
            "curobo_mpc_node = spot_operation_ros2.curobo_mpc_node:main",
            "sam2_tracker_node = spot_operation_ros2.sam2_tracker_node:main",
            "vlm_relocalize_node = spot_operation_ros2.vlm_relocalize_node:main",
            "coordinator_node = spot_operation_ros2.coordinator_node:main",
            "tf_projection_node = spot_operation_ros2.tf_projection_node:main",
            "control_mode_switcher = spot_operation_ros2.control_mode_switcher:main",
            "fake_wrist_target = spot_operation_ros2.fake_wrist_target:main",
        ],
    },
)

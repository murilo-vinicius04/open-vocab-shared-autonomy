from setuptools import setup
import os
from glob import glob

package_name = "arm_pose_estimator"

setup(
    name=package_name,
    version="0.1.0",
    # list the Python packages that exist under the arm_pose_estimator/ folder
    packages=[package_name],
    # now without package_dir!
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "models"), glob("models/*")),
    ],
    install_requires=[
        "setuptools",
        "rclpy",
        "opencv-python",
        "opencv-contrib-python",
        "numpy",
        "cv_bridge",
        "message_filters",
        "mediapipe",
        "tf2_ros",
        "geometry_msgs",
        "sensor_msgs",
    ],
    zip_safe=True,
    maintainer="Anonymous",
    maintainer_email="anonymous@example.com",
    description="Arm pose estimator for ROS 2",
    license="TODO",
    entry_points={
        "console_scripts": [
            "hand_pose_estimator = arm_pose_estimator.hand_pose_estimator:main",
            "hand_orientation_estimator = arm_pose_estimator.hand_orientation_estimator:main",
            "wrist_detector = arm_pose_estimator.wrist_detector:main",
        ],
    },
)

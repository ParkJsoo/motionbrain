import os
from glob import glob

from setuptools import find_packages, setup

package_name = "motionbrain_ros_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jeongsoo Park",
    maintainer_email="ParkJsoo@users.noreply.github.com",
    description="ROS2 bridge for MotionBrain HTTP status, events, camera detection, and light commands.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "motionbrain_joint_state_node = motionbrain_ros_bridge.motionbrain_joint_state_node:main",
            "motionbrain_kinematics_node = motionbrain_ros_bridge.motionbrain_kinematics_node:main",
            "motionbrain_status_node = motionbrain_ros_bridge.motionbrain_status_node:main",
            "motionbrain_fake_endpoint = motionbrain_ros_bridge.fake_motionbrain_endpoint:main",
        ],
    },
)

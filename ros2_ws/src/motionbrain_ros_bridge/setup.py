from setuptools import find_packages, setup

package_name = "motionbrain_ros_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jeongsu Park",
    maintainer_email="jeongsoopark@example.com",
    description="ROS2 bridge for MotionBrain HTTP status, events, camera detection, and light commands.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "motionbrain_status_node = motionbrain_ros_bridge.motionbrain_status_node:main",
        ],
    },
)

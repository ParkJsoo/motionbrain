from setuptools import find_packages, setup

package_name = "motionbrain_mission"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", ["config/mission_home_wifi.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jeongsu Park",
    maintainer_email="jeongsoopark@example.com",
    description="Lightweight mission supervisor for MotionBrain ROS2 demos.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "motionbrain_mission_supervisor = motionbrain_mission.mission_supervisor_node:main",
        ],
    },
)

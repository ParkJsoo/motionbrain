FROM ros:jazzy-ros-base
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-gz ros-jazzy-rviz2 ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher-gui dbus-x11 libgl1-mesa-dri \
    libxcb-cursor0 novnc openbox websockify x11vnc xauth xvfb \
    && rm -rf /var/lib/apt/lists/*
COPY ros2_ws/src/motionbrain_description/urdf/ /opt/motionbrain/description/urdf/
COPY ros2_ws/src/motionbrain_description/rviz/ /opt/motionbrain/description/rviz/
COPY tools/visualization/desktop.sh /usr/local/bin/motionbrain-visual-desktop
ENTRYPOINT ["bash", "/usr/local/bin/motionbrain-visual-desktop"]

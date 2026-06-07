FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    dbus-x11 \
    libgl1-mesa-dri \
    libxcb-cursor0 \
    libxcb-xinerama0 \
    novnc \
    openbox \
    python3-colcon-common-extensions \
    python3-numpy \
    python3-opencv \
    scrot \
    websockify \
    x11vnc \
    xauth \
    xvfb \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/motionbrain/ros2_ws
COPY --from=ros2src . /opt/motionbrain/ros2_ws/src

RUN . /opt/ros/jazzy/setup.sh \
    && colcon build --symlink-install

COPY start_rviz_novnc.sh /usr/local/bin/start_rviz_novnc.sh
COPY motionbrain_http_mirror.py /usr/local/bin/motionbrain_http_mirror.py
RUN chmod +x /usr/local/bin/start_rviz_novnc.sh /usr/local/bin/motionbrain_http_mirror.py

EXPOSE 5900 6080

CMD ["/usr/local/bin/start_rviz_novnc.sh"]

# MotionBrain RViz noVNC

This directory contains the local Docker/noVNC RViz helper used for Mac-side
demo capture when Pi-local RViz is unavailable.

It starts:

- `Xvfb`, `openbox`, `x11vnc`, and noVNC on `localhost:6080`
- RViz with the MotionBrain RobotModel/TF config
- a read-only HTTP mirror that polls the Pi dashboard and republishes:
  - `/motionbrain/status`
  - `/motionbrain/status_typed`
  - `/camera/detection`
  - `/camera/detection_typed`

No actuator command is published by these tools.

## Build

```bash
docker build \
  -f tools/rviz_novnc/Dockerfile \
  --build-context ros2src=ros2_ws/src \
  -t motionbrain-rviz:jazzy-novnc \
  tools/rviz_novnc
```

## Run

```bash
docker run --rm -d \
  --name motionbrain-rviz \
  -p 5900:5900 \
  -p 6080:6080 \
  -e MOTIONBRAIN_DASHBOARD_URL=http://motionbrain-pi.local:8765 \
  motionbrain-rviz:jazzy-novnc
```

Open:

```text
http://localhost:6080/vnc.html?autoconnect=true&resize=scale
```

If `.local` or router DNS fails from Docker, the mirror scans
`MOTIONBRAIN_DASHBOARD_DISCOVERY_CIDRS` on port `8765` and switches to the
first endpoint whose `/api/config` looks like the MotionBrain dashboard.

## Refresh The Mirror In A Running Container

```bash
tools/rviz_novnc/restart_http_mirror.sh
```

Useful checks:

```bash
docker exec motionbrain-rviz bash -lc 'set +u; source /opt/ros/jazzy/setup.bash; source /opt/motionbrain/ros2_ws/install/setup.bash; ros2 param get /motionbrain_http_mirror dashboard_url'
docker exec motionbrain-rviz bash -lc 'DISPLAY=:1 scrot -z /tmp/rviz.png'
docker cp motionbrain-rviz:/tmp/rviz.png .codex/tmp/rviz.png
```

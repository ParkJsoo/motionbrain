# 2026-06-16 Raspberry Pi System Health Evidence

[README](../../README.en.md) | [Robotics system readiness](../../ROBOTICS_SYSTEM_READINESS.en.md)

This note summarizes a read-only health check of the MotionBrain Raspberry Pi
host. It covers SSH reachability, systemd services, dashboard/perception HTTP
health, and ROS2 bridge topic/service/action availability.

No physical actuation command was sent. The ROS2 routine service and action were
called with `action: status` only.

## Environment

| Item | Value |
| --- | --- |
| Capture time | `2026-06-16T23:02:39+09:00` |
| Host | `motionbrain-pi` |
| OS/kernel | Ubuntu 24.04, Linux `6.8.0-1057-raspi`, `aarch64` |
| Clean worktree | `~/develop/arduino/motionbrain` |
| Commit | `588a5bd Generalize robotics portfolio readiness` |

## SSH Reachability

Mac-to-Pi check using `tools/raspi/check_pi_ssh_target.py`:

- alias: `motionbrain-pi`
- configured host: `motionbrain-pi.local`
- user: `motionbrain`
- host key alias: `motionbrain-pi.local`
- TCP/22 reachable at `192.168.219.109` and IPv6
- remote check: `ok`
- SSH service: `active`
- SSH socket: `enabled`, `active`

The router DNS fallback `motionbrain-pi.davolink` was unresolved during this
capture; the primary `.local` path was reachable.

## systemd Units

| Unit | State |
| --- | --- |
| `motionbrain-dashboard.service` | active/running |
| `motionbrain-perception.service` | active/running |
| `motionbrain-ros-bridge.service` | active/running |
| `motionbrain-dashboard-reconcile.timer` | active/waiting |
| `motionbrain-dashboard-reconcile.service` | static; timer-managed oneshot |

## Dashboard And Perception Health

`tools/raspi/check_dashboard_health.sh` passed with service checks enabled:

- perception service active
- dashboard service active
- `http://127.0.0.1:8766/health`
- `http://127.0.0.1:8766/api/detection`
- `http://127.0.0.1:8765/api/config`
- `http://127.0.0.1:8765/api/status`

## ROS2 Bridge Health

`tools/raspi/check_ros_bridge_health.sh` passed with service checks enabled.

Verified topics:

- `/motionbrain/status_typed`
- `/motionbrain/routine`
- `/motionbrain/routine_typed`
- `/motionbrain/lifecycle_typed`
- `/motionbrain/diagnostics`
- `/camera/detection_typed`
- `/joint_states`
- `/motionbrain/end_effector_pose`
- `/motionbrain/kinematics_typed`
- `/motionbrain/control_guard_typed`
- `/motionbrain/mission_state_typed`

Verified service/action:

- `/motionbrain/routine_command`
- `/motionbrain/guarded_routine`

Verified samples:

- status typed sample
- routine diagnostics sample
- routine typed feedback readiness sample
- lifecycle active samples
- diagnostics sample
- routine command status sample
- guarded routine status action sample
- camera detection typed sample
- joint state sample
- end-effector pose sample
- kinematics typed sample
- control guard typed sample
- mission state typed sample

## Correct Claim

```text
The Raspberry Pi host was reachable, the dashboard/perception/ROS2 bridge
services were active, and the ROS2 bridge exposed the expected typed topics,
service, action, and read-only status samples.
```

## Claims To Avoid

- Physical routine execution
- Autonomous motion
- Closed-loop joint convergence
- Production uptime or reliability guarantees

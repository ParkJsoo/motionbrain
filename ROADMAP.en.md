# MotionBrain Roadmap

[Korean roadmap](ROADMAP.md) | [English README](README.en.md) | [Portfolio](PORTFOLIO.en.md)

Last updated: 2026-06-08 KST

This roadmap replaces the old phase documents and the physical-AI MVP plan. The
current baseline is the `demo-ready-20260608` portfolio snapshot plus the latest
`main` documentation.

## Current Baseline

MotionBrain is now a demo-ready embedded robotics portfolio project:

- ESP32 5-axis DC motor controller with a safety state machine.
- STM32 wired teleop/sensor layer with deadman and freshness checks.
- ESP32-CAM video input with `STREAM`, `SNAPSHOT`, and `TRACKED` modes.
- Raspberry Pi dashboard/perception service with constrained `cup`
  known-object detection.
- ROS2 Jazzy bridge with typed status, detection, joint state, kinematics,
  guard, mission, and RViz evidence.
- Final physical teleoperation demo published as README GIF/MP4 assets.
- Stable snapshot tag: `demo-ready-20260608`.

## Roadmap Principles

- Do not expand claims without new physical evidence.
- ESP32 remains the actuator and safety boundary.
- Pi/ROS2/perception may propose or visualize actions, but should not bypass
  the embedded command boundary.
- Keep autonomous grasping out of scope until sensing and feedback improve.
- Prefer one constrained, repeatable workcell scenario over broad object claims.

## Track 1: Portfolio And Applications

Status: active, low engineering risk.

Goal:

- Use the completed demo as a clear job-application artifact.

Next work:

1. Use `demo-ready-20260608` and the tag-pinned README media links in
   applications.
2. Prepare role-specific summaries:
   - embedded firmware / motor control
   - robot system software
   - ROS2 integration
   - dashboard/perception tooling
3. Keep README, portfolio docs, and demo media in sync after any public change.
4. Avoid adding broad AI/autonomy claims without another validated demo.

Done when:

- Resume bullets, interview explanation, README, and portfolio documents tell
  the same story.

## Track 2: Hardware Feedback Upgrade

Status: future hardware work.

Goal:

- Add the feedback needed before attempting more autonomous physical actions.

Candidate upgrades:

- Gripper-mounted range or contact sensing.
- Better camera position or better camera module.
- Base or joint feedback for repeatable pose/trajectory checks.
- Cleaner fixture/workcell for fixed-object experiments.

Next work:

1. Choose one feedback gap to close first.
2. Update `PIN_MAP.md` and wiring docs before modifying hardware.
3. Validate the new signal read-only before tying it to motion.
4. Add safety-state behavior and tests before any physical sequence uses it.

Done when:

- The new feedback signal is visible in status/telemetry and has a repeatable
  failure behavior.

## Track 3: Constrained Perception To Action

Status: design next; physical execution deferred.

Goal:

- Move from `detect -> align -> dry-run plan` to one operator-confirmed,
  low-speed, fixed-workcell sequence.

Allowed scope:

- One known target.
- One fixed workcell position or marker-assisted setup.
- Operator confirmation before motion.
- Low-speed, short-duration command sequence.
- Immediate stop/status verification after each action.

Not allowed yet:

- Arbitrary object recognition.
- Text-prompt object search.
- Continuous visual servoing.
- Autonomous grasping without improved feedback.

Next work:

1. Pick marker-assisted or fixed-known-object mode.
2. Define the exact preconditions for execution.
3. Keep the first implementation as dry-run/log-only.
4. Add one guarded physical sequence only after the dry-run state is stable.

Done when:

- The system can produce and explain a guarded plan without overstating
  autonomy, and any physical execution remains operator-confirmed.

## Track 4: ROS2 And Ops Hardening

Status: optional polish for robotics-system roles.

Goal:

- Make the host-side system easier to demonstrate, debug, and explain.

Next work:

1. Keep systemd service docs and health checks current.
2. Add concise ROS2 evidence captures when a new milestone changes behavior.
3. Consider richer typed messages only when JSON payload growth becomes a
   maintenance problem.
4. Keep RViz/TF/RobotModel evidence aligned with the physical demo story.

Done when:

- A reviewer can understand controller health, perception state, and ROS2
  bridge behavior from one short runbook plus screenshots/log excerpts.

## Retired Roadmaps

The previous phase roadmap, Korean roadmap, and physical-AI MVP plan were
retired because their completed parts are now part of the baseline and their
remaining items are represented by the tracks above.

Removed documents:

- `PHASE3_PLAN.md`
- `로드맵.md`
- `docs/PHYSICAL_AI_OBJECT_DETECTION_PLAN.md`

Use this file as the single public roadmap going forward.

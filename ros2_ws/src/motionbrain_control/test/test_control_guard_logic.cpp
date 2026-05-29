#include "motionbrain_control/control_guard_logic.hpp"

#include <gtest/gtest.h>

namespace
{

motionbrain_control::MotionStatusSnapshot ready_status()
{
  motionbrain_control::MotionStatusSnapshot status;
  status.available = true;
  status.armed = true;
  status.state = "IDLE";
  return status;
}

motionbrain_control::CameraDetectionSnapshot centered_detection()
{
  motionbrain_control::CameraDetectionSnapshot detection;
  detection.available = true;
  detection.detected = true;
  detection.alignment = "CENTER";
  return detection;
}

}  // namespace

TEST(ControlGuardLogicTest, StatusStaleBlocksCommands)
{
  const auto decision = motionbrain_control::evaluate_control_guard(
    ready_status(),
    centered_detection(),
    false,
    true,
    motionbrain_control::ControlGuardConfig{});

  EXPECT_FALSE(decision.ready);
  EXPECT_EQ("status_stale", decision.reason);
}

TEST(ControlGuardLogicTest, FaultedAndMovingStatesBlockCommands)
{
  auto status = ready_status();
  status.faulted = true;

  auto decision = motionbrain_control::evaluate_control_guard(
    status,
    centered_detection(),
    true,
    true,
    motionbrain_control::ControlGuardConfig{});
  EXPECT_FALSE(decision.ready);
  EXPECT_EQ("faulted", decision.reason);

  status.faulted = false;
  status.moving = true;
  decision = motionbrain_control::evaluate_control_guard(
    status,
    centered_detection(),
    true,
    true,
    motionbrain_control::ControlGuardConfig{});
  EXPECT_FALSE(decision.ready);
  EXPECT_EQ("already_moving", decision.reason);
}

TEST(ControlGuardLogicTest, RequireArmedBlocksIdleDisarmedRobot)
{
  auto status = ready_status();
  status.armed = false;

  motionbrain_control::ControlGuardConfig config;
  config.require_armed = true;

  const auto decision = motionbrain_control::evaluate_control_guard(
    status,
    centered_detection(),
    true,
    true,
    config);

  EXPECT_FALSE(decision.ready);
  EXPECT_EQ("not_armed", decision.reason);
}

TEST(ControlGuardLogicTest, OptionalDetectionDoesNotBlockReadyState)
{
  motionbrain_control::CameraDetectionSnapshot detection;

  const auto decision = motionbrain_control::evaluate_control_guard(
    ready_status(),
    detection,
    true,
    false,
    motionbrain_control::ControlGuardConfig{});

  EXPECT_TRUE(decision.ready);
  EXPECT_EQ("ready", decision.reason);
  EXPECT_EQ("none", decision.suggested_action);
}

TEST(ControlGuardLogicTest, RequiredDetectionBlocksStaleOrMissingTarget)
{
  motionbrain_control::ControlGuardConfig config;
  config.require_detection = true;

  auto decision = motionbrain_control::evaluate_control_guard(
    ready_status(),
    centered_detection(),
    true,
    false,
    config);
  EXPECT_FALSE(decision.ready);
  EXPECT_EQ("detection_stale", decision.reason);

  motionbrain_control::CameraDetectionSnapshot detection;
  detection.available = true;
  detection.detected = false;
  decision = motionbrain_control::evaluate_control_guard(
    ready_status(),
    detection,
    true,
    true,
    config);
  EXPECT_FALSE(decision.ready);
  EXPECT_EQ("target_not_detected", decision.reason);
}

TEST(ControlGuardLogicTest, DetectionSuggestionFallsBackFromAlignment)
{
  auto detection = centered_detection();
  detection.alignment = "LEFT";

  auto decision = motionbrain_control::evaluate_control_guard(
    ready_status(),
    detection,
    true,
    true,
    motionbrain_control::ControlGuardConfig{});

  EXPECT_TRUE(decision.ready);
  EXPECT_EQ("base_left", decision.suggested_action);

  detection.command_suggestion = "custom_action";
  decision = motionbrain_control::evaluate_control_guard(
    ready_status(),
    detection,
    true,
    true,
    motionbrain_control::ControlGuardConfig{});

  EXPECT_EQ("custom_action", decision.suggested_action);
}


/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL, BIT
 */

#pragma once

#include <mc_control/CompletionCriteria.h>
#include <mc_control/fsm/State.h>

namespace mc_control
{
namespace fsm
{
struct Controller;
} // namespace fsm
} // namespace mc_control

/**
 * @brief Sets a robot pose relative to another robot
 * object: robot to move
 * robot: robot wrt to which it will be moved
 */
struct UpdateRobotPose : mc_control::fsm::State
{

  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  mc_rtc::Configuration config_;
  /* object name */
  std::string name_;
  /* origin surface */
  std::string surface_;
  /* offset from origin surface */
  sva::PTransformd surfaceOffset_ = sva::PTransformd::Identity();

  /* robot name */
  std::string robotName_;
  /* whether to use the real robot or control robot */
  bool useReal_ = false;
  /* origin surface on robot */
  std::string robotSurface_;
  /* offset wrt to robot origin surface */
  sva::PTransformd robotSurfaceOffset_ = sva::PTransformd::Identity();
};

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

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  /* object name */
  std::string name_;
  /* origin frame */
  std::string frame_;
  /* offset from origin frame */
  sva::PTransformd frameOffset_ = sva::PTransformd::Identity();

  /* robot name */
  std::string robotName_;
  /* whether to use the real robot or control robot */
  bool useReal_ = false;
  /* origin frame on robot */
  std::string robotFrame_;
  /* offset wrt to robot origin frame */
  sva::PTransformd robotFrameOffset_ = sva::PTransformd::Identity();

  /* Whether we should also update additional robots (typically contacts) */
  std::vector<std::string> additionalRobots_;
};

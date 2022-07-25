#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/State.h>
#include <mc_tasks/TransformTask.h>

namespace whycon_plugin
{

/** Move a frame in a given (world or local) direction at a given speed until
 * a force threhsold is met or a distance threhshold is reached */
struct MoveUntilTouch : mc_control::fsm::State
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void start(mc_control::fsm::Controller & ctl) override;
  bool run(mc_control::fsm::Controller & ctl) override;
  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  Eigen::Vector3d direction_ = Eigen::Vector3d{0, 0, 1};
  Eigen::Vector3d worldDirection_ = Eigen::Vector3d{0, 0, 1};
  bool directionIsLocal_ = true;
  double speed_ = 0.01;
  double pressureThreshold_ = 15;
  double distanceThreshold_ = 0.1;
  /** State logic */
  std::shared_ptr<mc_tasks::TransformTask> task_ = nullptr;
  Eigen::Vector3d pressureZero_;
  sva::PTransformd positionZero_;
  unsigned int iter_ = 0;
  bool done_ = false;
  void done();
};

} // namespace whycon_plugin

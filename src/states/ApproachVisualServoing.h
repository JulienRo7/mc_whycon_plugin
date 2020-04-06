#pragma once

#include <mc_control/CompletionCriteria.h>
#include <mc_control/fsm/Controller.h>
#include <mc_whycon_plugin/TaskUpdater.h>

#include <mc_tasks/BSplineTrajectoryTask.h>
#include <mc_tasks/LookAtTask.h>
#include <mc_tasks/PositionBasedVisServoTask.h>

namespace whycon_plugin
{

/** Approach the selected bracket */
struct ApproachVisualServoing : mc_control::fsm::State
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  /** Configuration of the state, can be used to retrieve method specific configuration */
  mc_rtc::Configuration config_;
  /** Marker name */
  std::string marker_;
  /** Estimation of the bracket position obtained from the vision system */
  sva::PTransformd X_0_bracket_;
  /** Spline task used to drive the gripper above
   * XXX: to be repaced with a spline with exact waypoints in the future
   **/
  std::shared_ptr<mc_tasks::BSplineTrajectoryTask> task_;
  mc_control::CompletionCriteria taskCrit_;

  /** Visual servoing task used to adjust the gripper x/y position */
  std::shared_ptr<mc_tasks::PositionBasedVisServoTask> pbvsTask_;
  double stiffness_ = 2;
  bool manualConfirmation_ = true;
  /** Evaluation threshold for the task */
  double evalTh_ = 0.02;
  /** Speed threshold for the task */
  double speedTh_ = 0.02;
  /** Task used to keep the vision system active */
  std::shared_ptr<mc_tasks::LookAtTask> lookAt_;
  /** Task updater for LookAt task */
  std::unique_ptr<whycon_plugin::TaskUpdater> updater_;
  /** Waypoints */
  std::vector<sva::PTransformd> waypoints_;
  /** User enables visual servoing approach */
  bool userEnableVS_ = false;
  /** True if the initial positioning is done */
  bool posDone_ = false;
  /** True if visual servoing is done */
  bool vsDone_ = false;
  std::vector<std::string> category_;

private:
  size_t iter_ = 0;
  size_t waypoint_ = 0;
  void nextWaypoint();
  void configureThresholds(const std::string & phase);
};

} // namespace whycon_plugin

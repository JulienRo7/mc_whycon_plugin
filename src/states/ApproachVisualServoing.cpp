#include "ApproachVisualServoing.h"

#include <mc_rbdyn/rpy_utils.h>
#include <mc_tasks/MetaTaskLoader.h>
#include <mc_whycon_plugin/WhyConSubscriber.h>
#include <mc_whycon_plugin/WhyConUpdater.h>

#include <sch/S_Object/S_Box.h>

namespace whycon_plugin
{

void ApproachVisualServoing::configure(const mc_rtc::Configuration & config)
{
  config_.load(config);
}

sva::PTransformd ApproachVisualServoing::robotMarkerToSurfaceOffset(const mc_control::fsm::Controller & ctl) const
{
  const auto & observer = static_cast<const WhyConSubscriber &>(*subscriber_);
  const auto & robotMarker = observer.lshape(robotMarkerName_);
  auto & robot = ctl.robots().robot(robotMarker.robot);
  auto X_0_robotSurface = robot.surfacePose(robotSurface_);
  auto X_0_robotMarker = robotMarker.surfaceOffset * robot.surfacePose(robotMarker.surface);
  return X_0_robotSurface * X_0_robotMarker.inv();
}

sva::PTransformd ApproachVisualServoing::targetMarkerToSurfaceOffset(const mc_control::fsm::Controller & ctl) const
{
  const auto & observer = static_cast<const WhyConSubscriber &>(*subscriber_);
  const auto & targetMarker = observer.lshape(targetMarkerName_);
  auto & targetRobot = ctl.robots().robot(targetMarker.robot);

  // Visual servoing target:
  // First compute the relative transform between the target marker and gripper
  // marker so that the robot surface is at the target surface (+offset) at the
  // end of the PBVS task convergence
  auto X_0_targetMarker = targetMarker.surfaceOffset * targetRobot.surfacePose(targetMarker.surface);
  auto X_targetMarker_targetSurface_ = targetRobot.surfacePose(targetSurface_) * X_0_targetMarker.inv();
  auto X_targetSurface_target = targetOffset_;
  auto X_targetMarker_target = X_targetSurface_target * X_targetMarker_targetSurface_;
  return X_targetMarker_target;
}

void ApproachVisualServoing::updateLookAt(const mc_control::fsm::Controller & ctl)
{
  const auto & observer = static_cast<const WhyConSubscriber &>(*subscriber_);
  const auto & robotMarker = observer.lshape(robotMarkerName_);
  const auto & targetMarker = observer.lshape(targetMarkerName_);
  auto & robot = ctl.robots().robot(robotMarker.robot);
  auto & targetRobot = ctl.robots().robot(targetMarker.robot);
  lookAt_->target(sva::interpolate(targetMarker.surfaceOffset * targetRobot.surfacePose(targetMarker.surface),
                                   robotMarker.surfaceOffset * robot.surfacePose(robotMarker.surface), 0.5)
                      .translation());
}

void ApproachVisualServoing::setBoundedSpeed(mc_control::fsm::Controller & ctl, double speed)
{
  maxSpeed_ = speed;
  if(constr_)
  {
    constr_->removeBoundedSpeed(ctl.solver(), ctl.robot().surface(robotSurface_).bodyName());
  }
  Eigen::Vector6d spd;
  spd << M_PI * maxSpeed_, M_PI * maxSpeed_, M_PI * maxSpeed_, maxSpeed_, maxSpeed_, maxSpeed_;
  constr_->addBoundedSpeed(ctl.solver(), ctl.robot().surface(robotSurface_).bodyName(),
                           ctl.robot().surface(robotSurface_).X_b_s().translation(), Eigen::MatrixXd::Identity(6, 6),
                           -spd, spd);
  LOG_INFO("[ApproachVisualServoing] Bounded speed set to " << spd.transpose());
}

void ApproachVisualServoing::start(mc_control::fsm::Controller & ctl)
{
  /* Get the selected bracket position from the vision system */
  subscriber_ = ctl.datastore().call<std::shared_ptr<WhyConSubscriber>>("WhyconPlugin::getWhyconSubscriber");
  const auto & observer = static_cast<const WhyConSubscriber &>(*subscriber_);

  category_ = config_("category_", std::vector<std::string>{name()});

  const auto & pbvsConf = config_("visualServoing");
  pbvsConf("offset", targetOffset_);

  const auto & approachConf = config_("approach");
  robotMarkerName_ = static_cast<std::string>(config_("robot")("marker"));
  robotSurface_ = static_cast<std::string>(config_("robot")("surface"));
  targetMarkerName_ = static_cast<std::string>(config_("target")("marker"));
  targetSurface_ = static_cast<std::string>(config_("target")("surface"));

  const auto & targetMarker = observer.lshape(targetMarkerName_);
  const auto & robotMarker = observer.lshape(robotMarkerName_);
  auto & targetRobot = ctl.robots().robot(targetMarker.robot);
  auto & robot = ctl.robots().robot(robotMarker.robot);

  /** Create the VS task, will add later */
  pbvsConf("stiffness", stiffness_);
  pbvsConf("maxStiffness", maxStiffness_);
  pbvsTask_ = std::make_shared<mc_tasks::PositionBasedVisServoTask>(
      robotSurface_, sva::PTransformd::Identity() /* No initial error, will be set by the updater later */,
      ctl.robots(), robot.robotIndex(), stiffness_, pbvsConf("weight", 500.));
  if(pbvsConf.has("joints"))
  {
    pbvsTask_->selectActiveJoints(pbvsConf("joints"));
  }
  pbvsConf("manualConfirmation", manualConfirmation_);
  pbvsConf("eval", evalTh_);
  pbvsConf("speed", speedTh_);

  constr_ = std::make_shared<mc_solver::BoundedSpeedConstr>(ctl.robots(), 0, ctl.solver().dt());
  ctl.solver().addConstraintSet(*constr_);
  pbvsConf("maxSpeed", maxSpeed_);
  prevMaxSpeed_ = maxSpeed_;

  auto targetOffset = targetMarkerToSurfaceOffset(ctl);
  auto robotOffset = robotMarkerToSurfaceOffset(ctl);
  updater_.reset(new WhyConUpdater(observer, robotMarkerName_, targetMarkerName_, targetOffset, robotOffset));

  /* approach */
  bool useMarker = approachConf("useMarker", false);
  auto approachOffset = approachConf("offset", sva::PTransformd::Identity());
  auto X_0_markerSurface = targetRobot.surfacePose(targetMarker.surface);
  auto X_0_targetSurface_ = targetRobot.surfacePose(targetSurface_);
  auto X_markerSurface_targetSurface_ = X_0_targetSurface_ * X_0_markerSurface.inv();
  mc_tasks::BSplineTrajectoryTask::waypoints_t waypoints;
  std::vector<std::pair<double, Eigen::Matrix3d>> oriWp;
  if(useMarker)
  { /* Target relative to the target marker */
    X_0_bracket_ = approachOffset * X_markerSurface_targetSurface_ * observer.X_0_marker(targetMarkerName_);
  }
  else
  { /* Target relative to the target robot's surface */
    X_0_bracket_ = approachOffset * X_markerSurface_targetSurface_ * X_0_markerSurface;
  }
  if(approachConf.has("waypoints"))
  { // Control points offsets defined wrt to the target surface frame
    const auto & controlPoints = approachConf("waypoints");
    waypoints.resize(controlPoints.size());
    for(unsigned int i = 0; i < controlPoints.size(); ++i)
    {
      const Eigen::Vector3d wp = controlPoints[i];
      sva::PTransformd X_offset(wp);
      waypoints[i] = (X_offset * X_0_bracket_).translation();
    }
  }
  if(approachConf.has("oriWaypoints"))
  { // orientation waypoints are defined wrt to the target surface frame
    std::vector<std::pair<double, Eigen::Matrix3d>> oriWaypoints = approachConf("oriWaypoints");
    for(const auto & wp : oriWaypoints)
    {
      const sva::PTransformd offset{wp.second};
      const sva::PTransformd ori = offset * X_0_bracket_;
      oriWp.push_back(std::make_pair(wp.first, ori.rotation()));
    }
  }
  task_ = std::make_shared<mc_tasks::BSplineTrajectoryTask>(
      ctl.solver().robots(), ctl.solver().robots().robot(robotMarker.robot).robotIndex(), robotSurface_,
      approachConf("duration"), approachConf("stiffness"), approachConf("weight"), X_0_bracket_, waypoints, oriWp);
  const auto displaySamples = approachConf("displaySamples", task_->displaySamples());
  task_->displaySamples(displaySamples);
  task_->pause(approachConf("paused", false));
  taskCrit_.configure(*task_, ctl.solver().dt(), approachConf("completion"));

  task_->reset();
  ctl.solver().addTask(task_);

  /** Setup look at task */
  if(config_.has("lookAt"))
  {
    const auto & lookConf = config_("lookAt");
    lookAt_ =
        std::make_shared<mc_tasks::LookAtTask>(lookConf("body"), lookConf("bodyVector"), ctl.robots(),
                                               robot.robotIndex(), lookConf("stiffness", 2.), lookConf("weight", 100.));
    if(lookConf.has("joints"))
    {
      lookAt_->selectActiveJoints(lookConf("joints"));
    }
    // Target the expected pose
    lookAt_->target(X_0_targetSurface_.translation());
    ctl.solver().addTask(lookAt_);
  }
}

void ApproachVisualServoing::teardown(mc_control::fsm::Controller & ctl)
{
  if(task_)
  {
    ctl.solver().removeTask(task_);
    ctl.solver().removeTask(pbvsTask_);
    ctl.gui()->removeElement(category_, "Enable visual servoing");
    ctl.gui()->removeElement(category_, "Status");
    ctl.gui()->removeElement(category_, "Pause");
    ctl.gui()->removeElement(category_, "Resume");
    ctl.gui()->removeElement(category_, "Stiffness");
    ctl.gui()->removeElement(category_, "Max stiffness");
    ctl.gui()->removeElement(category_, "Max speed");
    ctl.gui()->removeElement(category_, "Offset wrt target surface (translation) [m]");
    ctl.gui()->removeElement(category_, "Offset wrt target surface (rotation) [deg]");
  }
  if(lookAt_)
  {
    ctl.solver().removeTask(lookAt_);
  }
  if(constr_)
  {
    ctl.solver().removeConstraintSet(*constr_);
  }
}

void ApproachVisualServoing::updatePBVSTask(const mc_control::fsm::Controller & ctl)
{
  auto & updater = static_cast<WhyConUpdater &>(*updater_);
  updater.envOffset(targetMarkerToSurfaceOffset(ctl));
  updater.surfaceOffset(robotMarkerToSurfaceOffset(ctl));
  updater.update(*pbvsTask_);
}

bool ApproachVisualServoing::run(mc_control::fsm::Controller & ctl)
{
  // if(lookAt_)
  // {
  //   updater_->updateLookAt(*lookAt_);
  // }

  if(!task_)
  {
    output("NoVision");
    return true;
  }

  /* Approach trajectory completed, start visual servoing */
  if(!posDone_)
  {
    bool completed = taskCrit_.completed(*task_) && iter_++ > 10;
    if(completed)
    {
      posDone_ = true;
      iter_ = 0;
      // Look halfway between the expected markers
      updateLookAt(ctl);
      auto enableVisualServoing = [this, &ctl]() {
        userEnableVS_ = true;
        ctl.solver().removeTask(task_);
        ctl.solver().addTask(pbvsTask_);
        updatePBVSTask(ctl);
        // Limit speed of visual servoing
        setBoundedSpeed(ctl, maxSpeed_);
        ctl.gui()->removeElement(category_, "Enable visual servoing");
      };
      if(manualConfirmation_)
      {
        ctl.gui()->addElement(category_, mc_rtc::gui::Button("Enable visual servoing", enableVisualServoing));
      }
      else
      {
        enableVisualServoing();
      }
      ctl.gui()->addElement(
          category_,
          mc_rtc::gui::Label("Status",
                             [this]() {
                               if(vsPaused_)
                               {
                                 return "paused";
                               }
                               else if(!userEnableVS_)
                               {
                                 return "not enabled";
                               }
                               else if(userEnableVS_ && !vsDone_)
                               {
                                 return "active";
                               }
                               else if(userEnableVS_ && vsDone_)
                               {
                                 return "converged";
                               }
                               return "unknown";
                             }),
          mc_rtc::gui::Button("Pause",
                              [this, &ctl]() {
                                userEnableVS_ = false;
                                vsPaused_ = true;
                                pbvsTask_->error(sva::PTransformd::Identity());
                                prevMaxSpeed_ = maxSpeed_;
                                setBoundedSpeed(ctl, 0);
                              }),
          mc_rtc::gui::Button("Resume",
                              [this, &ctl]() {
                                userEnableVS_ = true;
                                vsResume_ = true;
                                vsPaused_ = false;
                                updatePBVSTask(ctl);
                                setBoundedSpeed(ctl, prevMaxSpeed_);
                              }),
          mc_rtc::gui::Label("Stiffness", [this]() { return stiffness_; }),
          mc_rtc::gui::NumberInput("Max stiffness", [this]() { return maxStiffness_; },
                                   [this](double s) { maxStiffness_ = std::max(0., s); }),
          mc_rtc::gui::NumberInput("Max speed", [this]() { return maxSpeed_; },
                                   [this, &ctl](double s) { setBoundedSpeed(ctl, std::max(0., s)); }),
          mc_rtc::gui::ArrayInput("Offset wrt target surface (translation) [m]", {"x", "y", "z"},
                                  [this]() -> const Eigen::Vector3d & { return targetOffset_.translation(); },
                                  [this](const Eigen::Vector3d & t) { targetOffset_.translation() = t; }),
          mc_rtc::gui::ArrayInput("Offset wrt target surface (rotation) [deg]", {"r", "p", "y"},
                                  [this]() -> Eigen::Vector3d {
                                    return mc_rbdyn::rpyFromMat(targetOffset_.rotation()) * 180.
                                           / mc_rtc::constants::PI;
                                  },
                                  [this](const Eigen::Vector3d & rpy) {
                                    targetOffset_.rotation() = mc_rbdyn::rpyToMat(rpy * mc_rtc::constants::PI / 180.);
                                  }));
    }
  }
  else if(!userEnableVS_ || vsPaused_ || vsResume_)
  { // visual servoing disabled, do nothing
    vsResume_ = false;
  }
  else if(!vsDone_)
  {
    // updateLookAt(ctl);
    if(pbvsTask_->eval().tail(3).norm() < evalTh_ && pbvsTask_->speed().tail(3).norm() < speedTh_ && iter_++ > 10)
    {
      vsDone_ = true;
      task_->reset();
      ctl.solver().removeTask(pbvsTask_);
      output("OK");
      return true;
    }
    else
    {
      updatePBVSTask(ctl);
      // If we still haven't converged, double stiffness every 100 iterations
      if(pbvsTask_->speed().tail(3).norm() < speedTh_ && iter_++ > 100)
      {
        double stiffness = std::min(2 * stiffness_, maxStiffness_);
        if(pbvsTask_->stiffness() < stiffness)
        {
          pbvsTask_->stiffness(stiffness);
          iter_ = 0;
          stiffness_ = stiffness;
        }
      }
    }
  }
  else
  {
    output("OK");
    return true;
  }
  return false;
}

} // namespace whycon_plugin

EXPORT_SINGLE_STATE("WhyconPlugin::ApproachVisualServoing", whycon_plugin::ApproachVisualServoing)

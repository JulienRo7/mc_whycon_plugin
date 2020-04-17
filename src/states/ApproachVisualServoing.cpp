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
  auto X_targetSurface_target = targetOffset_ * targetSurfaceOffset_;
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
  const auto & robotMarker = subscriber_->lshape(robotMarkerName_);
  auto & robot = ctl.robots().robot(robotMarker.robot);

  maxSpeed_ = speed;
  if(constr_)
  {
    constr_->removeBoundedSpeed(ctl.solver(), robot.surface(robotSurface_).bodyName());
  }
  Eigen::Vector6d spd;
  spd << M_PI * maxSpeed_, M_PI * maxSpeed_, M_PI * maxSpeed_, maxSpeed_, maxSpeed_, maxSpeed_;
  constr_->addBoundedSpeed(ctl.solver(), robot.surface(robotSurface_).bodyName(),
                           robot.surface(robotSurface_).X_b_s().translation(), Eigen::MatrixXd::Identity(6, 6), -spd,
                           spd);
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
  approachConf("use", useApproach_);
  robotMarkerName_ = static_cast<std::string>(config_("robot")("marker"));
  robotSurface_ = static_cast<std::string>(config_("robot")("surface"));
  targetMarkerName_ = static_cast<std::string>(config_("target")("marker"));
  targetSurface_ = static_cast<std::string>(config_("target")("surface"));
  config_("target")("surfaceOffset", targetSurfaceOffset_);

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
  pbvsConf("use", useVisualServoing_);
  pbvsConf("eval", evalTh_);
  pbvsConf("speed", speedTh_);

  constr_ = std::make_shared<mc_solver::BoundedSpeedConstr>(ctl.robots(), robot.robotIndex(), ctl.solver().dt());
  ctl.solver().addConstraintSet(*constr_);
  pbvsConf("maxSpeed", maxSpeedDesired_);
  maxSpeed_ = maxSpeedDesired_;

  auto targetOffset = targetMarkerToSurfaceOffset(ctl);
  auto robotOffset = robotMarkerToSurfaceOffset(ctl);

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
    X_0_bracket_ =
        approachOffset * targetSurfaceOffset_ * X_markerSurface_targetSurface_ * observer.X_0_marker(targetMarkerName_);
  }
  else
  { /* Target relative to the target robot's surface */
    X_0_bracket_ = approachOffset * targetSurfaceOffset_ * X_markerSurface_targetSurface_ * X_0_markerSurface;
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
  if(useApproach_)
  {
    ctl.solver().addTask(task_);
  }
  else
  {
    setBoundedSpeed(ctl, 0.);
  }

  /** Setup look at task */
  if(config_.has("lookAt"))
  {
    const auto & lookConf = config_("lookAt");
    lookAt_ = std::make_shared<mc_tasks::LookAtTask>(
        lookConf("body"), lookConf("bodyVector"), ctl.robots(),
        ctl.robots().robot(lookConf("robot", ctl.robots().robot().name())).robotIndex(), lookConf("stiffness", 2.),
        lookConf("weight", 100.));
    if(lookConf.has("joints"))
    {
      lookAt_->selectActiveJoints(lookConf("joints"));
    }
    // Target the expected pose
    lookAt_->target(X_0_targetSurface_.translation());
  }
}

void ApproachVisualServoing::teardown(mc_control::fsm::Controller & ctl)
{
  if(task_)
  {
    ctl.solver().removeTask(task_);
    ctl.solver().removeTask(pbvsTask_);
    ctl.gui()->removeCategory(category_);
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

void ApproachVisualServoing::pause(mc_control::fsm::Controller & ctl)
{
  if(vsPaused_) return;
  vsPaused_ = true;
  pbvsTask_->error(sva::PTransformd::Identity());
  setBoundedSpeed(ctl, 0);
}

void ApproachVisualServoing::resume(mc_control::fsm::Controller & ctl)
{
  if(vsResume_) return;
  vsResume_ = true;
  vsPaused_ = false;
  wasVisible_ = false;
  updatePBVSTask(ctl);
  // setBoundedSpeed(ctl, maxSpeedDesired_);
}

void ApproachVisualServoing::enableVisualServoing(mc_control::fsm::Controller & ctl)
{
  userEnableVS_ = true;
  ctl.solver().removeTask(task_);
  ctl.solver().addTask(pbvsTask_);
  updatePBVSTask(ctl);
  // Limit speed of visual servoing
  setBoundedSpeed(ctl, maxSpeedDesired_);
  ctl.gui()->removeElement(category_, "Enable visual servoing");
}

bool ApproachVisualServoing::updatePBVSTask(mc_control::fsm::Controller & ctl)
{
  auto & task = pbvsTask_;
  visible_ = subscriber_->visible(targetMarkerName_) && subscriber_->visible(robotMarkerName_);

  // If the marker becomes not visible, disable task
  if(!visible_ && wasVisible_)
  {
    LOG_WARNING(
        "[ApproachVisualServoing] Disabling visual servoing updates, will re-enable when the markers become visible");
    pbvsTask_->error(sva::PTransformd::Identity());
    setBoundedSpeed(ctl, 0);
    wasVisible_ = false;
    return false;
  }

  // If at last iteration the marker was not visible but now is, re-enable
  if(visible_ && !wasVisible_)
  {
    LOG_INFO("[ApproachVisualServoing] Re-enabling visual servoing");
    setBoundedSpeed(ctl, maxSpeedDesired_);
  }

  static bool once = true;
  auto envOffset = targetMarkerToSurfaceOffset(ctl);
  auto surfaceOffset = robotMarkerToSurfaceOffset(ctl);
  auto X_camera_target = envOffset * subscriber_->X_camera_marker(targetMarkerName_);
  auto X_camera_surface = surfaceOffset * subscriber_->X_camera_marker(robotMarkerName_);
  auto X_t_s = X_camera_surface * X_camera_target.inv();
  if(once)
  {
    std::cout << "X_camera_target:\n"
              << "\ttranslation: " << X_camera_target.translation().transpose() << "\n"
              << "\trotation   : "
              << mc_rbdyn::rpyFromMat(X_camera_target.rotation()).transpose() * 180 / mc_rtc::constants::PI << "\n";
    std::cout << "X_camera_surface:\n"
              << "\ttranslation: " << X_camera_surface.translation().transpose() << "\n"
              << "\trotation   : "
              << mc_rbdyn::rpyFromMat(X_camera_surface.rotation()).transpose() * 180 / mc_rtc::constants::PI << "\n";
    std::cout << "surfaceOffset:\n"
              << "\ttranslation: " << surfaceOffset.translation().transpose() << "\n"
              << "\trotation   : "
              << mc_rbdyn::rpyFromMat(surfaceOffset.rotation()).transpose() * 180 / mc_rtc::constants::PI << "\n";
    std::cout << "envOffset:\n"
              << "\ttranslation: " << envOffset.translation().transpose() << "\n"
              << "\trotation   : "
              << mc_rbdyn::rpyFromMat(envOffset.rotation()).transpose() * 180 / mc_rtc::constants::PI << "\n";
    std::cout << "X_t_s:\n"
              << "\ttranslation: " << X_t_s.translation().transpose() << "\n"
              << "\trotation   : " << mc_rbdyn::rpyFromMat(X_t_s.rotation()).transpose() * 180 / mc_rtc::constants::PI
              << "\n";
    once = false;
  }
  task->error(X_t_s);
  wasVisible_ = visible_;
  return true;
}

bool ApproachVisualServoing::run(mc_control::fsm::Controller & ctl)
{
  if(!task_)
  {
    output("NoVision");
    return true;
  }

  /* Approach trajectory completed, start visual servoing */
  if(!posDone_)
  {
    bool completed = !useApproach_ || (taskCrit_.completed(*task_) && iter_++ > 10);
    if(completed)
    {
      posDone_ = true;
      iter_ = 0;
      // Look halfway between the expected markers
      ctl.solver().addTask(lookAt_);
      LOG_INFO("completed, update lookat");
      updateLookAt(ctl);
      if(useVisualServoing_)
      {
        if(manualConfirmation_)
        {
          ctl.gui()->addElement(category_, mc_rtc::gui::Button("Enable visual servoing",
                                                               [this, &ctl]() { this->enableVisualServoing(ctl); }));
        }
        else
        {
          enableVisualServoing(ctl);
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
          mc_rtc::gui::Label("Marker " + robotMarkerName_,
                             [this]() { return subscriber_->visible(robotMarkerName_) ? "visible" : "not visible"; }),
          mc_rtc::gui::Label("Marker " + targetMarkerName_,
                             [this]() { return subscriber_->visible(targetMarkerName_) ? "visible" : "not visible"; }),
          mc_rtc::gui::Label("Error [m]",
                             [this]() {
                               if(pbvsTask_)
                               {
                                 return pbvsTask_->eval().tail(3).norm();
                               }
                               return 0.;
                             }),
          mc_rtc::gui::Button("Pause", [this, &ctl]() { pause(ctl); }),
          mc_rtc::gui::Button("Resume", [this, &ctl]() { resume(ctl); }),
          mc_rtc::gui::Label("Stiffness", [this]() { return stiffness_; }),
          mc_rtc::gui::NumberInput("Max stiffness", [this]() { return maxStiffness_; },
                                   [this](double s) { maxStiffness_ = std::max(0., s); }),
          mc_rtc::gui::Label("Actual max speed", [this]() { return maxSpeed_; }),
          mc_rtc::gui::NumberInput("Max speed", [this]() { return maxSpeedDesired_; },
                                   [this, &ctl](double s) {
                                     maxSpeedDesired_ = std::max(0., s);
                                     setBoundedSpeed(ctl, maxSpeedDesired_);
                                   }),
          mc_rtc::gui::NumberInput("Convergence Threshold [m]", [this]() { return evalTh_; },
                                   [this, &ctl](double s) { evalTh_ = std::max(0., s); }),
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
  }
  else if(useVisualServoing_ && (!userEnableVS_ || vsPaused_ || vsResume_))
  { // visual servoing disabled, do nothing
    vsResume_ = false;
  }
  else if(useVisualServoing_ && !vsDone_)
  {
    // updateLookAt(ctl);
    if(visible_ && pbvsTask_->eval().tail(3).norm() < evalTh_ && pbvsTask_->speed().tail(3).norm() < speedTh_
       && iter_++ > 10)
    {
      vsDone_ = true;
      task_->reset();
      ctl.solver().removeTask(pbvsTask_);
      // Ensure that the hand stops moving after completion if the state remains
      // active
      setBoundedSpeed(ctl, 0);
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

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

void ApproachVisualServoing::start(mc_control::fsm::Controller & ctl)
{
  /* Get the selected bracket position from the vision system */
  subscriber_ = ctl.datastore().call<std::shared_ptr<WhyConSubscriber>>("WhyconPlugin::getWhyconSubscriber");
  const auto & observer = static_cast<const WhyConSubscriber &>(*subscriber_);

  category_ = config_("category_", std::vector<std::string>{name()});

  const auto & pbvsConf = config_("visualServoing");
  pbvsConf("offset", sva::PTransformd::Identity());

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
    ctl.gui()->removeElement(category_, "Pause");
    ctl.gui()->removeElement(category_, "Resume");
  }
  if(lookAt_)
  {
    ctl.solver().removeTask(lookAt_);
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
      auto enableVisualServoing = [this, &ctl]() {
        userEnableVS_ = true;
        ctl.solver().removeTask(task_);
        ctl.solver().addTask(pbvsTask_);
        updatePBVSTask(ctl);
        ctl.gui()->removeElement(category_, "Enable visual servoing");
      };
      if(manualConfirmation_)
      {
        ctl.gui()->addElement(category_, mc_rtc::gui::Button("Enable visual servoing", enableVisualServoing));
        ctl.gui()->addElement(category_,
                              mc_rtc::gui::Button("Pause",
                                                  [this, &ctl]() {
                                                    userEnableVS_ = false;
                                                    pbvsTask_->error(sva::PTransformd::Identity());
                                                  }),
                              mc_rtc::gui::Button("Resume", [this, &ctl]() {
                                userEnableVS_ = true;
                                vsResume_ = true;
                                updatePBVSTask(ctl);
                              }));
      }
      else
      {
        enableVisualServoing();
      }
    }
  }
  else if(!userEnableVS_ || vsResume_)
  { // visual servoing disabled, do nothing
  }
  else if(!vsDone_)
  {
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

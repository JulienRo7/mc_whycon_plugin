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

void ApproachVisualServoing::start(mc_control::fsm::Controller & ctl)
{
  category_ = config_("category_", std::vector<std::string>{name()});
  ctl.grippers["l_gripper"]->setTargetOpening(1);

  const auto & pbvsConf = config_("visualServoing");
  const auto & approachConf = config_("approach");
  std::string robot_marker = config_("robot")("marker");
  std::string robotSurface = config_("robot")("surface");
  std::string target_marker = config_("target")("marker");
  std::string targetSurface = config_("target")("surface");

  /* Get the selected bracket position from the vision system */
  auto subscriber = ctl.datastore().call<std::shared_ptr<WhyConSubscriber>>("WhyconPlugin::getWhyconSubscriber");
  const auto & observer = static_cast<const WhyConSubscriber &>(*subscriber);

  const auto & targetMarker = observer.lshape(target_marker);
  const auto & robotMarker = observer.lshape(robot_marker);
  auto & targetRobot = ctl.robots().robot(targetMarker.robot);
  auto & robot = ctl.robots().robot(robotMarker.robot);

  // Visual servoing target:
  // First compute the relative transform between the target marker and gripper
  // marker so that the robot surface is at the target surface (+offset) at the
  // end of the PBVS task convergence
  auto X_0_targetMarker = targetMarker.surfaceOffset * targetRobot.surfacePose(targetMarker.surface);
  auto X_targetMarker_targetSurface = targetRobot.surfacePose(targetSurface) * X_0_targetMarker.inv();
  auto X_targetSurface_target = pbvsConf("offset", sva::PTransformd::Identity());
  auto X_targetMarker_target = X_targetSurface_target * X_targetMarker_targetSurface;

  auto X_0_robotSurface = robot.surfacePose(robotSurface);
  auto X_0_robotMarker = robotMarker.surfaceOffset * robot.surfacePose(robotMarker.surface);
  auto X_robotSurface_robotMarker = X_0_robotMarker * X_0_robotSurface.inv();

  /** Final desired offset between the target marker and the gripper marker */
  auto offset = X_robotSurface_robotMarker * X_targetMarker_target;
  LOG_INFO("OFFSET translation: " << offset.translation().transpose());
  LOG_INFO("OFFSET: rotation  : " << mc_rbdyn::rpyFromMat(offset.rotation()).transpose() * 180.
                                         / mc_rtc::constants::PI);
  /** Create the VS task, will add later */
  pbvsConf("stiffness", stiffness_);
  pbvsTask_ = std::make_shared<mc_tasks::PositionBasedVisServoTask>(
      robotSurface, offset, ctl.robots(), robot.robotIndex(), stiffness_, pbvsConf("weight", 500.));
  if(pbvsConf.has("joints"))
  {
    pbvsTask_->selectActiveJoints(pbvsConf("joints"));
  }
  pbvsConf("manualConfirmation", manualConfirmation_);
  pbvsConf("eval", evalTh_);
  pbvsConf("speed", speedTh_);

  if(!observer.visible(target_marker))
  {
    LOG_ERROR("[" << name() << "] " << target_marker << " is not visible")
    return;
  }
  updater_.reset(new WhyConUpdater(observer, robot_marker, target_marker, offset));

  /* approach */
  bool useMarker = approachConf("useMarker", false);
  auto approachOffset = approachConf("offset", sva::PTransformd::Identity());
  auto X_0_markerSurface = targetRobot.surfacePose(targetMarker.surface);
  auto X_0_targetSurface = targetRobot.surfacePose(targetSurface);
  auto X_markerSurface_targetSurface = X_0_targetSurface * X_0_markerSurface.inv();
  mc_tasks::BSplineTrajectoryTask::waypoints_t waypoints;
  std::vector<std::pair<double, Eigen::Matrix3d>> oriWp;
  if(useMarker)
  { /* Target relative to the target marker */
    X_0_bracket_ = approachOffset * X_markerSurface_targetSurface * observer.X_0_marker(target_marker);
  }
  else
  { /* Target relative to the target robot's surface */
    X_0_bracket_ = approachOffset * X_markerSurface_targetSurface * X_0_markerSurface;
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
      ctl.solver().robots(), ctl.solver().robots().robot(robotMarker.robot).robotIndex(), robotSurface,
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
    lookAt_->target(X_0_targetSurface.translation());
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
  }
  if(lookAt_)
  {
    ctl.solver().removeTask(lookAt_);
  }
}

bool ApproachVisualServoing::run(mc_control::fsm::Controller & ctl)
{
  if(lookAt_)
  {
    updater_->updateLookAt(*lookAt_);
  }

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
        updater_->update(*pbvsTask_);
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
    }
  }
  else if(!userEnableVS_)
  {
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
      updater_->update(*pbvsTask_);
      // If we still haven't converged, double stiffness every 100 iterations
      if(pbvsTask_->speed().tail(3).norm() < speedTh_ && iter_++ > 100)
      {
        double stiffness = 2 * stiffness_;
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

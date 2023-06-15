#include "TouchWithAdmittance.h"
#include <mc_tasks/MetaTaskLoader.h>

namespace whycon_plugin
{

sva::PTransformd TouchWithAdmittance::robotMarkerToFrameOffset(const mc_control::fsm::Controller & ctl) const
{
  const auto & observer = static_cast<const WhyConSubscriber &>(*subscriber_);
  const auto & robotMarker = observer.lshape(robotMarkerName_);
  auto & robot = ctl.robot(robotMarker.robot);
  auto X_0_robotFrame = robot.frame(robotFrame_).position();
  auto X_0_robotMarker = robotMarker.frameOffset * robot.frame(robotMarker.frame).position();
  return X_0_robotFrame * X_0_robotMarker.inv();
}

sva::PTransformd TouchWithAdmittance::targetMarkerToFrameOffset(const mc_control::fsm::Controller & ctl) const
{
  const auto & observer = static_cast<const WhyConSubscriber &>(*subscriber_);
  const auto & targetMarker = observer.lshape(targetMarkerName_);
  auto & targetRobot = ctl.robot(targetMarker.robot);

  auto X_0_targetMarker = targetMarker.frameOffset * targetRobot.frame(targetMarker.frame).position();
  auto X_targetMarker_targetFrame_ = targetRobot.frame(targetFrame_).position() * X_0_targetMarker.inv();
  auto X_targetFrame_target = targetOffset_ * targetFrameOffset_;
  auto X_targetMarker_target = X_targetFrame_target * X_targetMarker_targetFrame_;
  return X_targetMarker_target;
}

void TouchWithAdmittance::updateLookAt(const mc_control::fsm::Controller & ctl)
{
  const auto & observer = static_cast<const WhyConSubscriber &>(*subscriber_);
  const auto & robotMarker = observer.lshape(robotMarkerName_);
  const auto & targetMarker = observer.lshape(targetMarkerName_);
  auto & robot = ctl.robot(robotMarker.robot);
  auto & targetRobot = ctl.robot(targetMarker.robot);
  lookAt_->target(sva::interpolate(targetMarker.frameOffset * targetRobot.frame(targetMarker.frame).position(),
                                   robotMarker.frameOffset * robot.frame(robotMarker.frame).position(), 0.5)
                      .translation());
}

// void TouchWithAdmittance::setBoundedSpeed(mc_control::fsm::Controller & ctl, double speed)
// {
//   const auto & robotMarker = subscriber_->lshape(robotMarkerName_);
//   auto & robot = ctl.robot(robotMarker.robot);
//   // XXX (mc_rtc): shouldn't RobotFrame::parent() return a RobotFrame instead of a Frame if the parent was a
//   RobotFrame? const auto & parentFrame =
//   *std::static_pointer_cast<mc_rbdyn::RobotFrame>(robot.frame(robotFrame_).parent());

//   maxSpeed_ = speed;
//   if(constr_)
//   {
//     constr_->removeBoundedSpeed(ctl.solver(), parentFrame);
//   }
//   Eigen::Vector6d spd;
//   spd << M_PI * maxSpeed_, M_PI * maxSpeed_, M_PI * maxSpeed_, maxSpeed_, maxSpeed_, maxSpeed_;
//   constr_->addBoundedSpeed(ctl.solver(), parentFrame, Eigen::MatrixXd::Identity(6, 6), -spd, spd);
//   mc_rtc::log::info("[{}] Bounded speed set to {}", name(), spd.transpose());
// }

void TouchWithAdmittance::start(mc_control::fsm::Controller & ctl)
{
  /* Get the selected bracket position from the vision system */
  subscriber_ = ctl.datastore().call<std::shared_ptr<WhyConSubscriber>>("WhyconPlugin::getWhyconSubscriber");
  const auto & observer = static_cast<const WhyConSubscriber &>(*subscriber_);

  category_ = config_("category_", std::vector<std::string>{name()});

  const auto & pbvsConf = config_("visualServoing");
  pbvsConf("offset", targetOffset_);

  robotMarkerName_ = static_cast<std::string>(config_("robot")("marker"));
  robotFrame_ = static_cast<std::string>(config_("robot")("frame"));
  targetMarkerName_ = static_cast<std::string>(config_("target")("marker"));
  targetFrame_ = static_cast<std::string>(config_("target")("frame"));
  config_("target")("frameOffset", targetFrameOffset_);

  const auto & targetMarker = observer.lshape(targetMarkerName_);
  const auto & robotMarker = observer.lshape(robotMarkerName_);
  auto & targetRobot = ctl.robot(targetMarker.robot);
  auto & robot = ctl.robot(robotMarker.robot);

  /** Create the VS task and  add it to solver  */
  pbvsConf("stiffness", stiffness_);
  pbvsConf("maxStiffness", maxStiffness_);
  pbvsTask_ = std::make_shared<mc_tasks::PositionBasedVisServoTask>(
      ctl.robot().frame(robotFrame_),
      sva::PTransformd::Identity(), /* No initial error, will be set by the updater later */
      stiffness_, pbvsConf("weight", 500.));

  if(pbvsConf.has("joints"))
  {
    pbvsTask_->selectActiveJoints(pbvsConf("joints"));
  }
  // pbvsConf("manualConfirmation", manualConfirmation_);
  // pbvsConf("use", useVisualServoing_);
  // pbvsConf("eval", evalTh_);
  // pbvsConf("speed", speedTh_);
  ctl.solver().addTask(pbvsTask_);

  /** Setup look at task */
  if(config_.has("lookAt"))
  {
    const auto & lookConf = config_("lookAt");
    auto & robot = ctl.robot(lookConf("robot", ctl.robot().name()));
    lookAt_ = std::make_shared<mc_tasks::LookAtTask>(robot.frame(lookConf("body")), lookConf("bodyVector"),
                                                     lookConf("stiffness", 2.), lookConf("weight", 100.));
    if(lookConf.has("joints"))
    {
      lookAt_->selectActiveJoints(lookConf("joints"));
    }
    // Target the expected pose
    auto X_0_targetFrame_ = targetRobot.frame(targetFrame_).position();
    lookAt_->target(X_0_targetFrame_.translation());
    ctl.solver().addTask(lookAt_);
  }

  /** Admittance task */
  // -> the admittance task as it is also a surface transform task should be used to control the xy position of the
  // endeffector
  admittanceTask_ = std::make_shared<mc_tasks::force::AdmittanceTask>(ctl.robot("hrp4").frame("toolchanger_rh"));
  // set the target force
  if(config_.has("admittanceTask"))
  {
    admittanceTask_->load(ctl.solver(), config_("admittanceTask"));
  }
  else
  {
    auto wrench = admittanceTask_->targetWrench();
    wrench.force() = {-20.0, 0.0, 0.0};
    wrench.couple() = {0.0, 0.0, 0.0};
    admittanceTask_->targetWrench(wrench);
    wrench.force() = {1e-3, 0.0, 0.0};
    wrench.couple() = {0.0, 0.0, 0.0};
    admittanceTask_->admittance(wrench);
  }
  // set the gains
  sva::MotionVecd stiffness, damping;
  stiffness.angular() = {0.0, 0.0, 0.0}; // because the vision deals with it
  stiffness.linear() = {1.0, 100.0, 100.0}; // because admittance on x and postion control on y and z
  damping.angular() = {0.0, 0.0, 0.0}; // because the vision deals with it
  damping.linear() = {300.0, 1.0, 1.0}; // because admittance on x and postion control on y and z
  admittanceTask_->stiffness(stiffness);
  admittanceTask_->damping(damping);

  ctl.solver().addTask(admittanceTask_);
  // log the initial position
  initialPose_ = admittanceTask_->surfacePose();
  p_ = 0.001;
  p2pi_ = p_ / (2 * mc_rtc::constants::PI);
  p24pi2_ = std::pow(p2pi_, 2);
  v_ = 0.010;
  R_ = 0.005;
  r_ = 0.0;
  theta_ = 0.0;
  thetap_ = v_ / std::sqrt(std::pow(r_, 2) + p24pi2_);
  rp_ = p2pi_ * thetap_;

  ctl.logger().addLogEntry("WhyConPlugin_TouchWithAdmittance_endEffectorFramesRelativePose_",
                           [this]() -> sva::PTransformd { return this->endEffectorFramesRelativePose_; });
  ctl.logger().addLogEntry("WhyConPlugin_TouchWithAdmittance_endEffectorFramesDistance_",
                           [this]() -> double { return this->endEffectorFramesRelativePose_.translation().norm(); });
}

void TouchWithAdmittance::teardown(mc_control::fsm::Controller & ctl)
{
  // if(task_)
  // {
  //   ctl.solver().removeTask(task_);
  //   ctl.solver().removeTask(pbvsTask_);
  //   ctl.gui()->removeCategory(category_);
  // }
  if(pbvsTask_)
  {
    ctl.solver().removeTask(pbvsTask_);
  }
  if(lookAt_)
  {
    ctl.solver().removeTask(lookAt_);
  }
  // if(constr_)
  // {
  //   ctl.solver().removeConstraintSet(*constr_);
  // }

  if(admittanceTask_)
  {
    ctl.solver().removeTask(admittanceTask_);
  }

  ctl.logger().removeLogEntry("WhyConPlugin_TouchWithAdmittance_endEffectorFramesRelativePose_");
  ctl.logger().removeLogEntry("WhyConPlugin_TouchWithAdmittance_endEffectorFramesDistance_");
}

bool TouchWithAdmittance::updatePBVSTask(mc_control::fsm::Controller & ctl)
{
  auto & task = pbvsTask_;
  visible_ = subscriber_->visible(targetMarkerName_) && subscriber_->visible(robotMarkerName_);

  // If the marker becomes not visible, disable task
  if(!visible_)
  {
    if(wasVisible_)
    {
      mc_rtc::log::warning("[{}] Disabling visual servoing updates, will re-enable when the markers become visible",
                           name());
      pbvsTask_->error(sva::PTransformd::Identity());
      wasVisible_ = false;
    }
    return false;
  }

  // If at last iteration the marker was not visible but now is, re-enable
  if(visible_ && !wasVisible_)
  {
    mc_rtc::log::info("[{}] Re-enabling visual servoing", name());
  }

  static bool once = true;
  auto envOffset = targetMarkerToFrameOffset(ctl);
  auto frameOffset = robotMarkerToFrameOffset(ctl);
  auto X_camera_target = envOffset * subscriber_->X_camera_marker(targetMarkerName_);
  auto X_camera_frame = frameOffset * subscriber_->X_camera_marker(robotMarkerName_);
  auto X_t_s = X_camera_frame * X_camera_target.inv();
  X_t_s.translation() = Eigen::Vector3d::Zero();
  // --> set the translation part of X_t_s to 0
  if(once)
  {
    std::cout << "X_camera_target:\n"
              << "\ttranslation: " << X_camera_target.translation().transpose() << "\n"
              << "\trotation   : "
              << mc_rbdyn::rpyFromMat(X_camera_target.rotation()).transpose() * 180 / mc_rtc::constants::PI << "\n";
    std::cout << "X_camera_frame:\n"
              << "\ttranslation: " << X_camera_frame.translation().transpose() << "\n"
              << "\trotation   : "
              << mc_rbdyn::rpyFromMat(X_camera_frame.rotation()).transpose() * 180 / mc_rtc::constants::PI << "\n";
    std::cout << "frameOffset:\n"
              << "\ttranslation: " << frameOffset.translation().transpose() << "\n"
              << "\trotation   : "
              << mc_rbdyn::rpyFromMat(frameOffset.rotation()).transpose() * 180 / mc_rtc::constants::PI << "\n";
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

bool TouchWithAdmittance::run(mc_control::fsm::Controller & ctl)
{
  // update xy position according the desired trajectory

  if(r_ >= R_) //  or endEffectorFramesDistance_ <= 1e-3
  {
    auto admittance = admittanceTask_->admittance();
    admittance.force() = {1e-3, 2e-4, 0.0};
    admittanceTask_->admittance(admittance);

    sva::MotionVecd stiffness, damping;
    stiffness.angular() = {0.0, 0.0, 0.0};
    stiffness.linear() = {1.0, 1.0, 0.0};
    damping.angular() = {0.0, 0.0, 0.0};
    damping.linear() = {300.0, 300.0, 0.0};
    admittanceTask_->stiffness(stiffness);
    admittanceTask_->damping(damping);
    done_ = true;
  }
  else
  {
    const double dt = ctl.solver().dt();

    r_ += rp_ * dt;
    theta_ += thetap_ * dt;

    thetap_ = v_ / std::sqrt(std::pow(r_, 2) + p24pi2_);
    rp_ = p2pi_ * thetap_;

    auto currentPose = admittanceTask_->targetPose();
    currentPose.translation().y() = initialPose_.translation().y() + r_ * std::cos(theta_);
    currentPose.translation().z() = initialPose_.translation().z() + r_ * std::sin(theta_);
    admittanceTask_->targetPose(currentPose);

    sva::MotionVecd refVelocity = sva::MotionVecd::Zero();
    refVelocity.linear().y() = rp_ * std::cos(theta_) + r_ * thetap_ * std::sin(theta_);
    refVelocity.linear().z() = rp_ * std::sin(theta_) - r_ * thetap_ * std::cos(theta_);
    admittanceTask_->refVelB(refVelocity);
  }

  // once the trajectory has completed turn back to admittance control of the y and z directions -> there seems to be an
  // issue with the admittance task after ward

  // update orientation error based on visual servoing
  updatePBVSTask(ctl);

  // Look halfway between the expected markers
  if(lookAt_)
  {
    updateLookAt(ctl);
  }

  // check if the toolchanger is engaged
  auto X_rM_rF = robotMarkerToFrameOffset(ctl);
  auto X_tM_tF = targetOffset_.inv() * targetMarkerToFrameOffset(ctl);
  auto X_camera_target = X_tM_tF * subscriber_->X_camera_marker(targetMarkerName_);
  auto X_camera_frame = X_rM_rF * subscriber_->X_camera_marker(robotMarkerName_);
  endEffectorFramesRelativePose_ = X_camera_frame * X_camera_target.inv();
  endEffectorFramesDistance_ = endEffectorFramesRelativePose_.translation().norm();

  if(done_)
  {
    output("OK");
    return true;
  }
  return false;
}

} // namespace whycon_plugin

EXPORT_SINGLE_STATE("WhyconPlugin::TouchWithAdmittance", whycon_plugin::TouchWithAdmittance)

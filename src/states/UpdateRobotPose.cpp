#include "UpdateRobotPose.h"
#include <mc_control/fsm/Controller.h>
#include <mc_rbdyn/rpy_utils.h>
#include <mc_rtc/ConfigurationHelpers.h>

void UpdateRobotPose::start(mc_control::fsm::Controller & ctl)
{
  if(config_.has("object"))
  {
    const auto & objConf = config_("object");
    name_ = objConf("name", std::string{});
    frame_ = objConf("frame", std::string{});
    objConf("offset", frameOffset_);
  }

  if(config_.has("robot"))
  {
    const auto & robotConf = config_("robot");
    robotName_ = robotConf("name", ctl.robot().name());
    robotFrame_ = static_cast<std::string>(robotConf("frame"));
    robotConf("offset", robotFrameOffset_);
  }

  additionalRobots_ = mc_rtc::fromVectorOrElement(config_, "additionalRobots", std::vector<std::string>{});

  auto & r = [&ctl, this]() -> mc_rbdyn::Robot & {
    return useReal_ ? ctl.realRobots().robot(robotName_) : ctl.robot(robotName_);
  }();
  sva::PTransformd X_0_robotFrameWithOffset = robotFrameOffset_ * r.frame(robotFrame_).position();

  auto & o = ctl.robot(name_);
  // Transform from object frame to its frame
  sva::PTransformd X_object_frame = o.frame(frame_).position() * o.posW().inv();
  // Transform from object frame to its frame with a user-specified offset
  sva::PTransformd X_object_frameOffset = frameOffset_ * X_object_frame;
  sva::PTransformd X_0_object = X_object_frameOffset.inv() * X_0_robotFrameWithOffset;
  auto X_0_prevObject = o.posW();
  o.posW(X_0_object);
  mc_rtc::log::info("[{}] Updated object robot ", name_);

  if(additionalRobots_.size())
  {
    for(const auto & updateRobotName : additionalRobots_)
    {
      auto & updateRobot = ctl.robot(updateRobotName);
      auto X_object_robot = updateRobot.posW() * X_0_prevObject.inv();
      updateRobot.posW(X_object_robot * X_0_object);
      mc_rtc::log::info("[{}] Updated additional robot {}", name(), updateRobotName);
    }
  }
}

bool UpdateRobotPose::run(mc_control::fsm::Controller & ctl)
{
  output("OK");
  return true;
}

void UpdateRobotPose::teardown(mc_control::fsm::Controller & ctl) {}

EXPORT_SINGLE_STATE("WhyconPlugin::UpdateRobotPose", UpdateRobotPose)

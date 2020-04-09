#include "MoveUntilTouch.h"
#include <mc_tasks/MetaTaskLoader.h>

namespace whycon_plugin
{

void MoveUntilTouch::configure(const mc_rtc::Configuration & config)
{
  config_.load(config);
}

void MoveUntilTouch::start(mc_control::fsm::Controller & ctl)
{
  config_("direction", direction_);
  config_("directionIsLocal", directionIsLocal_);
  config_("speed", speed_);
  config_("distanceThreshold", distanceThreshold_);
  config_("pressureThreshold", pressureThreshold_);
  direction_.normalize();

  task_ = mc_tasks::MetaTaskLoader::load<mc_tasks::SurfaceTransformTask>(ctl.solver(), config_("task"));
  ctl.solver().addTask(task_);
  pressureZero_ = ctl.robot().surfaceWrench(task_->surface()).force();
  auto relative = config_("relative", std::string("robot"));
  if(relative == "robot")
  {
    positionZero_ = ctl.robot().surfacePose(task_->surface());
    worldDirection_ = positionZero_.rotation().inverse() * direction_ ;
  }
  else if(relative == "target")
  {
    positionZero_ = task_->target();
    worldDirection_ = task_->target().rotation().inverse() * direction_;
  }
  else if(relative == "world")
  {
    positionZero_ = task_->target();
    worldDirection_ = direction_;
  }
  else
  {
    LOG_ERROR_AND_THROW(std::runtime_error,
                        "[" << name() << "] relative property only supports [robot, surface, world]");
  }

  iter_ = 0;
  LOG_INFO("[" << name() << "] Pressure threshold: " << pressureThreshold_)

  if(ctl.config()("simulation", false))
  {
    if(config_.has("simulation"))
    {
      config_("simulation")("distanceThreshold", distanceThreshold_);
    }
  }
}

void MoveUntilTouch::teardown(mc_control::fsm::Controller & ctl)
{
  ctl.solver().removeTask(task_);
}

bool MoveUntilTouch::run(mc_control::fsm::Controller & ctl)
{
  if(done_)
  {
    return true;
  }

  Eigen::Vector3d pressure = ctl.robot().surfaceWrench(task_->surface()).force();
  if((pressure - pressureZero_).norm() > pressureThreshold_)
  {
    iter_++;
    if(iter_ >= 5)
    {
      LOG_INFO("[" << name() << "] Pressure threhsold detected\n")
      done();
      output("OK");
      return true;
    }
  }
  else
  {
    iter_ = 0;
  }
  // Distance projected along direction
  sva::PTransformd X_target_surface = ctl.robot().surfacePose(task_->surface()) * positionZero_.inv();
  double distance = X_target_surface.translation().dot(direction_);
  if(distance > distanceThreshold_)
  {
    LOG_INFO("[" << name() << "] Distance threshold detected\n")
    done();
    output("OK");
    return true;
  }

  Eigen::Vector3d dx = speed_ * ctl.solver().dt() * worldDirection_;
  sva::PTransformd delta{dx};
  task_->target(task_->target() * delta);
  return false;
}

void MoveUntilTouch::done()
{
  done_ = true;
  task_->reset();
}

} // namespace whycon_plugin

EXPORT_SINGLE_STATE("WhyconPlugin::MoveUntilTouch", whycon_plugin::MoveUntilTouch)

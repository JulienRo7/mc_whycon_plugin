#include <mc_rbdyn/rpy_utils.h>
#include <mc_tasks/PositionBasedVisServoTask.h>
#include <mc_whycon_plugin/WhyConUpdater.h>

namespace whycon_plugin
{

WhyConUpdater::WhyConUpdater(const WhyConSubscriber & subscriber,
                             const std::string & frame,
                             const std::string & env,
                             const sva::PTransformd & envOffset,
                             const sva::PTransformd & frameOffset)
: subscriber_(subscriber), frame_(frame), env_(env), envOffset_(envOffset), frameOffset_(frameOffset)
{
}

bool WhyConUpdater::update(mc_tasks::MetaTask & task_)
{
  auto & task = static_cast<mc_tasks::PositionBasedVisServoTask &>(task_);
  bool visible = true;
  if(!subscriber_.visible(frame_))
  {
    visible = false;
    mc_rtc::log::error("[WhyConUpdater] Cannot see {} marker", frame_);
  }
  if(!subscriber_.visible(env_))
  {
    visible = false;
    mc_rtc::log::error("[WhyConUpdater] Cannot see {} marker", env_);
  }
  if(!visible)
  {
    task.error(sva::PTransformd::Identity());
    return false;
  }
  static bool once = true;
  auto X_camera_target = envOffset_ * subscriber_.X_camera_marker(env_);
  auto X_camera_frame = frameOffset_ * subscriber_.X_camera_marker(frame_);
  auto X_t_s = X_camera_frame * X_camera_target.inv();
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
              << "\ttranslation: " << frameOffset_.translation().transpose() << "\n"
              << "\trotation   : "
              << mc_rbdyn::rpyFromMat(frameOffset_.rotation()).transpose() * 180 / mc_rtc::constants::PI << "\n";
    std::cout << "envOffset:\n"
              << "\ttranslation: " << envOffset_.translation().transpose() << "\n"
              << "\trotation   : "
              << mc_rbdyn::rpyFromMat(envOffset_.rotation()).transpose() * 180 / mc_rtc::constants::PI << "\n";
    std::cout << "X_t_s:\n"
              << "\ttranslation: " << X_t_s.translation().transpose() << "\n"
              << "\trotation   : " << mc_rbdyn::rpyFromMat(X_t_s.rotation()).transpose() * 180 / mc_rtc::constants::PI
              << "\n";
    once = false;
  }
  task.error(X_t_s);
  return true;
}

bool WhyConUpdater::updateLookAt(mc_tasks::LookAtTask & task)
{
  if(subscriber_.visible(env_))
  {
    task.target(sva::interpolate(subscriber_.X_0_marker(frame_), subscriber_.X_0_marker(env_), 0.5).translation());
    return true;
  }
  return false;
}

} // namespace whycon_plugin

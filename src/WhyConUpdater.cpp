#include <mc_tasks/PositionBasedVisServoTask.h>
#include <mc_whycon_plugin/WhyConUpdater.h>

namespace whycon_plugin
{

WhyConUpdater::WhyConUpdater(const WhyConSubscriber & subscriber,
                             const std::string & surface,
                             const std::string & env,
                             const sva::PTransformd & offset)
: subscriber_(subscriber), surface_(surface), env_(env), offset_(offset)
{
}

bool WhyConUpdater::update(mc_tasks::MetaTask & task_)
{
  auto & task = static_cast<mc_tasks::PositionBasedVisServoTask &>(task_);
  bool visible = true;
  if(!subscriber_.visible(surface_))
  {
    visible = false;
    LOG_ERROR("[WhyConUpdater] Cannot see " << surface_ << " marker")
  }
  if(!subscriber_.visible(env_))
  {
    visible = false;
    LOG_ERROR("[WhyConUpdater] Cannot see " << env_ << " marker")
  }
  if(!visible)
  {
    return false;
  }
  static bool once = true;
  auto X_camera_target = offset_ * subscriber_.X_camera_marker(env_);
  auto X_camera_surface = subscriber_.X_camera_marker(surface_);
  auto X_t_s = X_camera_surface * X_camera_target.inv();
  if(once)
  {
    std::cout << "X_camera_target " << X_camera_target.translation().transpose() << "\n"
              << X_camera_target.rotation() << "\n";
    std::cout << "X_camera_surface " << X_camera_surface.translation().transpose() << "\n"
              << X_camera_surface.rotation() << "\n";
    std::cout << "X_t_s " << X_t_s.translation().transpose() << "\n" << X_t_s.rotation() << "\n";
    once = false;
  }
  task.error(X_t_s);
  return true;
}

bool WhyConUpdater::updateLookAt(mc_tasks::LookAtTask & task)
{
  if(subscriber_.visible(env_))
  {
    task.target(sva::interpolate(subscriber_.X_0_marker(surface_), subscriber_.X_0_marker(env_), 0.5).translation());
  }
}

} // namespace whycon_plugin

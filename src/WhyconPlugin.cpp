#include <mc_rbdyn/rpy_utils.h>
#include <mc_whycon_plugin/WhyConSubscriber.h>
#include <mc_whycon_plugin/WhyConUpdater.h>
#include <mc_whycon_plugin/WhyconPlugin.h>

namespace whycon_plugin
{

WhyconPlugin::WhyconPlugin() : nh_(mc_rtc::ROSBridge::get_node_handle())
{
  if(!nh_)
  {
    mc_rtc::log::error_and_throw("[WhyconPlugin] ROS is not available");
  }
}

WhyconPlugin::~WhyconPlugin()
{
  if(spinner_.joinable())
  {
    spinner_.join();
  }
}

void WhyconPlugin::init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config)
{
  auto & ctl = controller.controller();
  whyconSubscriber_ = std::make_shared<WhyConSubscriber>(ctl, config);

  // Add a callback to the datastore to create a task updater
  // Is this useful? Wouldn't a lambda that gets the subscriber be more useful?
  // Or even a lambda that creates a full state?
  ctl.datastore().make_call("WhyconPlugin::addTaskUpdater", [this](const std::string & name,
                                                                   const std::string & surface, const std::string & env,
                                                                   const sva::PTransformd & offset) {
    taskUpdaters_[name] = std::unique_ptr<WhyConUpdater>(new WhyConUpdater(*whyconSubscriber_, surface, env, offset));
  });
  ctl.datastore().make_call("WhyconPlugin::removeTaskUpdater",
                            [this](const std::string & name) { taskUpdaters_.erase(name); });
  ctl.datastore().make_call("WhyconPlugin::updateTask", [this](const std::string & name, mc_tasks::MetaTask & task) {
    taskUpdaters_.at(name)->update(task);
  });
  ctl.datastore().make_call(
      "WhyconPlugin::updateLookAtTask",
      [this](const std::string & name, mc_tasks::LookAtTask & task) { taskUpdaters_.at(name)->updateLookAt(task); });

  ctl.datastore().make_call("WhyconPlugin::getWhyconSubscriber", [this]() { return whyconSubscriber_; });

  if(!config.has("camera"))
  {
    mc_rtc::log::error_and_throw("[WhyconPlugin] No entry named camera in configuration");
  }

  if(!config("camera").has("surface"))
  {
    mc_rtc::log::error_and_throw("[WhyconPlugin] No entry camera/surface in configuration");
  }
  if(config("camera").has("surface"))
  {
    config("camera")("surface", cameraSurface_);
    if(!ctl.robot().hasSurface(cameraSurface_))
    {
      mc_rtc::log::error_and_throw("[WhyconPlugin] No surface named {} for the camera", cameraSurface_);
    }
    cameraOffset_ = config("camera")("offset", sva::PTransformd::Identity());
  }

  ctl.gui()->addElement({"Plugins", "WhyCon"},
                        mc_rtc::gui::ArrayInput(
                            "Camera offset RPY [deg]", {"x", "y", "z"},
                            [this]() -> Eigen::Vector3d {
                              return mc_rbdyn::rpyFromMat(cameraOffset_.rotation()) * 180 / mc_rtc::constants::PI;
                            },
                            [this](const Eigen::Vector3d & offset) {
                              cameraOffset_.rotation() = mc_rbdyn::rpyToMat(offset * mc_rtc::constants::PI / 180);
                            }),
                        mc_rtc::gui::ArrayInput(
                            "Camera offset translation [m]", {"x", "y", "z"},
                            [this]() -> const Eigen::Vector3d & { return cameraOffset_.translation(); },
                            [this](const Eigen::Vector3d & offset) { cameraOffset_.translation() = offset; }));

  spinner_ = std::thread([]() {
    ros::Rate rt(30);
    while(ros::ok())
    {
      ros::spinOnce();
      rt.sleep();
    }
  });

  initialized_ = true;
  mc_rtc::log::success("[Plugin::WhyconPlugin] initialized");
}

void WhyconPlugin::reset(mc_control::MCGlobalController & controller) {}

void WhyconPlugin::before(mc_control::MCGlobalController & controller)
{
  if(!initialized_) return;
  // Get Camera position
  auto & ctl = controller.controller();
  auto X_0_camera = cameraOffset_ * ctl.robot().surfacePose("TopCameraRGB");
  whyconSubscriber_->cameraPose(X_0_camera);
  whyconSubscriber_->tick(controller.controller().timeStep);
}

} // namespace whycon_plugin

EXPORT_MC_RTC_PLUGIN("WhyconPlugin", whycon_plugin::WhyconPlugin)

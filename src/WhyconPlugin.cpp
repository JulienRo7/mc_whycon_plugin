#include "WhyconPlugin.h"
#include "WhyConSubscriber.h"
#include "WhyConUpdater.h"
#include <mc_rbdyn/rpy_utils.h>


namespace whycon_plugin
{

WhyconPlugin::WhyconPlugin()
    : nh_(mc_rtc::ROSBridge::get_node_handle())
{  if(!nh_)
  {
    LOG_ERROR_AND_THROW(std::runtime_error, "[WhyconPlugin] ROS is not available")
  }
}

void WhyconPlugin::init(mc_control::MCGlobalController &controller, const mc_rtc::Configuration &config)
{
  auto & ctl = controller.controller();
  whyconSubscriber_ = std::make_shared<WhyConSubscriber>(ctl, config);

  // Add a callback to the datastore to create a task updater
  // Is this useful? Wouldn't a lambda that gets the subscriber be more useful?
  // Or even a lambda that creates a full state?
  ctl.datastore().make_call("WhyconPlugin::addTaskUpdater",
                                      [this](const std::string & name, const std::string & surface, const std::string & env, const sva::PTransformd & offset)
                                      {
                                        taskUpdaters_[name] = std::make_unique<WhyConUpdater>(*whyconSubscriber_, surface, env, offset);
                                      });
  ctl.datastore().make_call("WhyconPlugin::removeTaskUpdater",
                                      [this](const std::string & name)
                                      {
                                        taskUpdaters_.erase(name);
                                      });
  ctl.datastore().make_call("WhyconPlugin::updateTask",
                                      [this](const std::string & name, mc_tasks::MetaTask & task)
                                      {
                                        taskUpdaters_.at(name)->update(task);
                                      });
  ctl.datastore().make_call("WhyconPlugin::updateLookAtTask",
                                      [this](const std::string & name, mc_tasks::LookAtTask & task)
                                      {
                                        taskUpdaters_.at(name)->updateLookAt(task);
                                      });

  if(config("camera").has("surface"))
  {
    config("camera")("surface", cameraSurface_);
    cameraOffset_ = config("camera")("offset", sva::PTransformd::Identity());
  }

  ctl.gui()->addElement({"Plugins", "WhyCon"},
                        mc_rtc::gui::Label("Status",
                              []()
                              {
                                return "running";
                              }),
                        mc_rtc::gui::ArrayInput("Camera offset RPY [deg]", {"x", "y", "z"},
                          [this]() -> Eigen::Vector3d {
                            return mc_rbdyn::rpyFromMat(cameraOffset_.rotation()) * 180 / mc_rtc::constants::PI;
                        },
                        [this](const Eigen::Vector3d & offset) {
                          cameraOffset_.rotation() = mc_rbdyn::rpyToMat(offset * mc_rtc::constants::PI / 180);
                        }),
                             mc_rtc::gui::ArrayInput("Camera offset translation [m]", {"x", "y", "z"},
                        [this]() -> const Eigen::Vector3d & { return cameraOffset_.translation(); },
                        [this](const Eigen::Vector3d & offset) { cameraOffset_.translation() = offset; }));

  LOG_SUCCESS("[Plugin::WhyconPlugin] initialized");
}

void WhyconPlugin::reset(mc_control::MCGlobalController &controller)
{
}

void WhyconPlugin::before(mc_control::MCGlobalController &controller)
{
  // Get Camera position
  auto & ctl = controller.controller();
  auto X_0_camera = cameraOffset_ * ctl.robot().surfacePose("TopCameraRGB");
  whyconSubscriber_->cameraPose(X_0_camera);
  whyconSubscriber_->tick(controller.controller().timeStep);
}

} // namespace whycon_plugin

EXPORT_MC_RTC_PLUGIN("WhyconPlugin", whycon_plugin::WhyconPlugin)

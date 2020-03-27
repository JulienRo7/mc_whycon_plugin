#include "WhyconPlugin.h"
#include "WhyConSubscriber.h"


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
  LOG_SUCCESS("[Plugin::WhyconPlugin] initialized");
}

void WhyconPlugin::reset(mc_control::MCGlobalController &controller)
{
}

void WhyconPlugin::before(mc_control::MCGlobalController &controller)
{
  // Get Camera position
  auto & ctl = controller.controller();
  auto X_0_camera = ctl.robot().surfacePose("TopCameraRGB");

  whyconSubscriber_->cameraPose(X_0_camera);
  whyconSubscriber_->tick(controller.controller().timeStep);
}

} // namespace whycon_plugin

EXPORT_MC_RTC_PLUGIN("WhyconPlugin", whycon_plugin::WhyconPlugin)

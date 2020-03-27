#include "WhyconPlugin.h"


namespace mc_plugin
{

WhyconPlugin::WhyconPlugin()
    : nh_(mc_rtc::ROSBridge::get_node_handle())
{
}

void WhyconPlugin::init(mc_control::MCGlobalController &controller, const mc_rtc::Configuration &config)
{
}

void WhyconPlugin::reset(mc_control::MCGlobalController &controller)
{
}

} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("WhyconPlugin", mc_plugin::WhyconPlugin)

/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL, BIT
 */

#pragma once

#include <mc_rtc/ros.h>
#include <mc_rtc/DataStore.h>
#include <mc_control/GlobalPlugin.h>
#include <mc_control/GlobalPluginMacros.h>

namespace whycon_plugin
{

struct WhyConSubscriber;

struct WhyconPlugin : public mc_control::GlobalPlugin
{
    WhyconPlugin();

    void init(mc_control::MCGlobalController &controller, const mc_rtc::Configuration &config) override;

    void reset(mc_control::MCGlobalController &controller) override;

    void before(mc_control::MCGlobalController &) override;

    void after(mc_control::MCGlobalController &controller) override {};

private:
    std::shared_ptr<ros::NodeHandle> nh_;
    std::shared_ptr<WhyConSubscriber> whyconSubscriber_;
};

} // namespace whycon_plugin

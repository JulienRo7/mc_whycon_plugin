/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL, BIT
 */

#pragma once

#include <mc_control/GlobalPlugin.h>
#include <mc_control/GlobalPluginMacros.h>
#include <mc_rtc/DataStore.h>
#include <mc_rtc/ros.h>

namespace whycon_plugin
{

struct WhyConSubscriber;
struct WhyConUpdater;

struct WhyconPlugin : public mc_control::GlobalPlugin
{
  WhyconPlugin();

  void init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config) override;

  void reset(mc_control::MCGlobalController & controller) override;

  void before(mc_control::MCGlobalController &) override;

  void after(mc_control::MCGlobalController & controller) override{};

private:
  std::shared_ptr<ros::NodeHandle> nh_;
  std::shared_ptr<WhyConSubscriber> whyconSubscriber_;
  std::map<std::string, std::unique_ptr<WhyConUpdater>> taskUpdaters_;

  std::string cameraSurface_;
  sva::PTransformd cameraOffset_;

  /* temporary hack. for now in mc_openrtm before() is called as soon as we do connectComponent, but
init() is only called when starting the component */
  bool initialized_ = false;
};

} // namespace whycon_plugin

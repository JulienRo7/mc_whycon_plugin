#pragma once

#include <mc_control/mc_controller.h>
#include "LShape.h"
#include "VisionSubscriber.h"

#include <mc_rtc/Configuration.h>
#include <mc_rtc/ros.h>
#include <ros/ros.h>

#include <mutex>
#include <thread>

namespace whycon_plugin
{

/** Subscribe to WhyCon data to provide up-to-date information on the markers */
struct WhyConSubscriber : public VisionSubscriber
{
  WhyConSubscriber(mc_control::MCController & controller,
                   const mc_rtc::Configuration & config);

  ~WhyConSubscriber();

  WhyConSubscriber(const WhyConSubscriber &) = delete;
  WhyConSubscriber(WhyConSubscriber &&) = delete;
  WhyConSubscriber & operator=(const WhyConSubscriber &) = delete;
  WhyConSubscriber & operator=(WhyConSubscriber &&) = delete;

  /** set camera pose, to be called before tick() */
  void cameraPose(const sva::PTransformd & pose)
  {
    X_0_camera = pose;
  }

  void tick(double dt) override;

  /** Check whether a marker is visible or not */
  bool visible(const std::string & marker) const;

  /** Returns the camera position of a given marker */
  const sva::PTransformd & X_camera_marker(const std::string & marker) const;

  /** Returns the world position of a given marker */
  const sva::PTransformd & X_0_marker(const std::string & marker) const;
private:
  bool running_ = true;
  std::shared_ptr<ros::NodeHandle> nh_;
  mc_control::MCController & ctl_;
  std::thread updateThread_;
  mutable std::mutex updateMutex_;
  std::unordered_map<std::string, LShape> lshapes_;
  void newMarker(const std::string & name);
  ros::Subscriber sub_;
  /* Store the world position of the camera */
  sva::PTransformd X_0_camera = sva::PTransformd::Identity();
};

} /* whycon_plugin */

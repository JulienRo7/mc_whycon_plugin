#pragma once

#include <sn_walking/Controller.h>
#include <sn_walking/vision/VisionSubscriber.h>

#include <mc_rtc/Configuration.h>
#include <mc_rtc/ros.h>
#include <ros/ros.h>

#include <mutex>
#include <thread>

namespace sn_walking
{

namespace vision
{

/** Represent an object detected by the VISP detector */
struct VISPObject
{
  VISPObject(const std::string & name,
             const std::string & robot,
             const std::string & tf,
             const sva::PTransformd & offset)
  : name(name), robot(robot), tf(tf), offset(offset)
  {
  }

  VISPObject() : name(""), robot(""), tf("") {}

  /** Store object's name */
  std::string name;
  /** Store the robot linked to the object */
  std::string robot;
  /** Store the tf used in simulation to obtain a tf */
  std::string tf;
  /** Store world position */
  sva::PTransformd posWorld;
  /** Store simulation offset */
  sva::PTransformd offset;
  /** Only care about the rotation about Z for the reference simulation frame */
  bool yawOnly = false;

  /** True if the object is visible */
  bool visible = false;
  /** Position of the l-shape in the camera frame */
  sva::PTransformd pos = sva::PTransformd::Identity();
  /** Position of the l-shape in the world frame (estimated) */
  sva::PTransformd posW = sva::PTransformd::Identity();
  /** Tick every iteration to update the visibility */
  void tick(double dt)
  {
    lastUpdate_ += dt;
    visible = lastUpdate_ < 0.5;
  }
  /** Called to update the position of the object from the vision system */
  void update(const sva::PTransformd & in, const sva::PTransformd & X_0_camera)
  {
    visible = true;
    pos = in;
    posW = offset * pos * X_0_camera;
    lastUpdate_ = 0;
  }

private:
  double lastUpdate_ = 1;
};

/** Subscribe to VISP data to provide up-to-date information */
struct VISPSubscriber : public VisionSubscriber
{
  VISPSubscriber(Controller & controller, const mc_rtc::Configuration & config);

  ~VISPSubscriber();

  VISPSubscriber(const VISPSubscriber &) = delete;
  VISPSubscriber(VISPSubscriber &&) = delete;
  VISPSubscriber & operator=(const VISPSubscriber &) = delete;
  VISPSubscriber & operator=(VISPSubscriber &&) = delete;

  void tick(double dt) override;

  /** Check whether object is visible or not */
  bool visible(const std::string & name) const;

  const sva::PTransformd & X_camera_object(const std::string & object) const;

  const sva::PTransformd & X_0_object(const std::string & object) const;

  const VISPObject & object(const std::string & object) const
  {
    return objects_.at(object);
  }

private:
  bool running_ = true;
  std::shared_ptr<ros::NodeHandle> nh_;
  Controller & ctl_;
  std::unordered_map<std::string, VISPObject> objects_;
  std::thread updateThread_; // vector?
  mutable std::mutex updateMutex_;
  void newObject(const std::string & name);
  std::vector<ros::Subscriber> subs_;
};

} // namespace vision

} // namespace sn_walking

#include "WhyConSubscriber.h"

// ROS stuff
#include <ros/ros.h>
#include <whycon_lshape/WhyConLShapeMsg.h>
#include <mc_rbdyn/rpy_utils.h>
#include <tuple>

namespace whycon_plugin
{

WhyConSubscriber::WhyConSubscriber(mc_control::MCController & ctl,
                                   const mc_rtc::Configuration & config)
: nh_(mc_rtc::ROSBridge::get_node_handle()),
  ctl_(ctl)
{
  if(!nh_)
  {
    LOG_ERROR_AND_THROW(std::runtime_error, "[WhyConSubscriber] ROS is not available")
  }
  bool simulation = config("simulation");
  auto methodConf = config("whycon");
  if(simulation)
  {
    auto markers = methodConf("markers");
    std::unordered_map<std::string, std::function<void(const mc_control::MCController&, LShape&)>> markerUpdates_;
    for(auto k : markers.keys())
    {
      std::string robotName = markers(k)("robot", ctl.robot().name());
      std::string relative = markers(k)("relative", std::string(""));
      std::string relative_body = markers(k)("relative_body", std::string(""));
      sva::PTransformd pos = markers(k)("pos", sva::PTransformd::Identity());
      if(relative.size())
      {
        markerUpdates_[k] = [this,robotName,pos,relative](const mc_control::MCController & ctl, LShape & shape)
        {
          auto & robot = ctl.robots().robot(robotName);
          auto X_camera_0 = X_0_camera.inv();
          auto X_relative_marker = pos;
          auto X_0_marker = X_relative_marker * robot.surfacePose(relative);
          shape.update(X_0_marker * X_camera_0, X_0_camera);
        };
      }
      else if(relative_body.size())
      {
        markerUpdates_[k] = [this,robotName,pos,relative_body](const mc_control::MCController & ctl, LShape & shape)
        {
          auto & robot = ctl.robots().robot(robotName);
          auto X_camera_0 = X_0_camera.inv();
          auto X_relative_marker = pos;
          auto X_0_body = robot.bodyPosW(relative_body);
          Eigen::Vector3d rpy = mc_rbdyn::rpyFromMat(X_0_body.rotation());
          X_0_body.rotation() = mc_rbdyn::rpyToMat(0, 0, rpy.z());
          auto X_0_marker = X_relative_marker * X_0_body;
          shape.update(X_0_marker * X_camera_0, X_0_camera);
        };
      }
      else
      {
        markerUpdates_[k] = [this,pos](const mc_control::MCController & ctl, LShape & shape)
        {
          auto X_camera_0 = X_0_camera.inv();
          auto X_0_marker = pos;
          shape.update(X_0_marker * X_camera_0, X_0_camera);
        };
      }
      newMarker(k);
    }
    updateThread_ = std::thread([this,markerUpdates_]()
    {
      ros::Rate rt(30);
      while(ros::ok() && running_)
      {
        for(auto & m : markerUpdates_)
        {
          std::lock_guard<std::mutex> lock(updateMutex_);
          m.second(ctl_, lshapes_[m.first]);
        }
        rt.sleep();
      }
    });
  }
  else
  {
    boost::function<void(const whycon_lshape::WhyConLShapeMsg&)> callback_ =
       [this](const whycon_lshape::WhyConLShapeMsg & msg)
       {
        for(const auto & s : msg.shapes)
        {
          Eigen::Vector3d pos { s.pose.position.x, s.pose.position.y, s.pose.position.z };
          Eigen::Quaterniond q { s.pose.orientation.w, s.pose.orientation.x, s.pose.orientation.y, s.pose.orientation.z };
          const auto & name = s.name;
          std::lock_guard<std::mutex> lock(updateMutex_);
          if(!lshapes_.count(name))
          {
            lshapes_[name].update({q,pos}, X_0_camera);
            newMarker(name);
            // Add marker to the datastore
            ctl_.datastore().make<std::tuple<sva::PTransformd,sva::PTransformd, double>>("WhyconPlugin::Marker::" + name, lshapes_.at(name).posW, lshapes_.at(name).pos, lshapes_.at(name).lastUpdate());
          }
          else
          {
            lshapes_[name].update({q, pos}, X_0_camera);
            // Update datastore marker
            ctl_.datastore().assign("WhyconPlugin::Marker::" + name, std::tuple<sva::PTransformd,sva::PTransformd, double>{lshapes_.at(name).posW, lshapes_.at(name).pos, lshapes_.at(name).lastUpdate()});
          }
        }
       };
    sub_ = nh_->subscribe<whycon_lshape::WhyConLShapeMsg>(methodConf("topic"), 1000, callback_);
  }
}

WhyConSubscriber::~WhyConSubscriber()
{
  if(updateThread_.joinable())
  {
    running_ = false;
    updateThread_.join();
  }
}

void WhyConSubscriber::tick(double dt)
{
  std::lock_guard<std::mutex> lock(updateMutex_);
  for(auto & lshape : lshapes_)
  {
    lshape.second.tick(dt);
  }
}

bool WhyConSubscriber::visible(const std::string & marker) const
{
  std::lock_guard<std::mutex> lock(updateMutex_);
  return lshapes_.count(marker) && lshapes_.at(marker).visible;
}

const sva::PTransformd & WhyConSubscriber::X_camera_marker(const std::string & marker) const
{
  std::lock_guard<std::mutex> lock(updateMutex_);
  return lshapes_.at(marker).pos;
}

const sva::PTransformd & WhyConSubscriber::X_0_marker(const std::string & marker) const
{
  std::lock_guard<std::mutex> lock(updateMutex_);
  return lshapes_.at(marker).posW;
}

void WhyConSubscriber::newMarker(const std::string & name)
{
  LOG_INFO("[WhyConSubscriber] New marker: " << name)
  ctl_.logger().addLogEntry("WhyConMarkers_" + name, [this,name]() -> const sva::PTransformd & { return lshapes_.at(name).pos; });
  ctl_.logger().addLogEntry("WhyConMarkers_" + name + "_World", [this,name]() -> const sva::PTransformd & { return lshapes_.at(name).posW; });
  auto gui = ctl_.gui();
  if(!gui) { return; }
  gui->addElement({"Plugins", "WhyCon", "Markers"},
                  mc_rtc::gui::Transform(name,
                                         [this,name]() { return lshapes_.at(name).posW; }
                                         ));
}

} /* whycon_plugin */

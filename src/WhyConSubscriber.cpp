#include <mc_whycon_plugin/WhyConSubscriber.h>

// ROS stuff
#include <mc_rbdyn/rpy_utils.h>
#include <ros/ros.h>
#include <tuple>
#include <whycon_lshape/WhyConLShapeMsg.h>

namespace whycon_plugin
{

WhyConSubscriber::WhyConSubscriber(mc_control::MCController & ctl, const mc_rtc::Configuration & config)
: nh_(mc_rtc::ROSBridge::get_node_handle()), ctl_(ctl)
{
  if(!nh_)
  {
    mc_rtc::log::error_and_throw("[WhyConSubscriber] ROS is not available");
  }
  ctl.config()("simulation", simulation_);
  auto methodConf = config("whycon");

  auto markers = methodConf("markers");
  std::unordered_map<std::string, std::function<void(const mc_control::MCController &, LShape &)>> markerUpdates_;
  for(auto k : markers.keys())
  {
    std::string robotName = markers(k)("robot", ctl.robot().name());
    std::string relative = markers(k)("relative", std::string(""));
    sva::PTransformd pos = markers(k)("pos", sva::PTransformd::Identity());
    lshapes_[k].robot = robotName;
    lshapes_[k].frame = relative;
    lshapes_[k].frameOffset = pos;
  }
  readLshapes_ = lshapes_;

  if(simulation_)
  {
    for(auto k : markers.keys())
    {
      markerUpdates_[k] = [this, k](const mc_control::MCController & ctl, LShape & shape)
      {
        auto & robot = ctl.robot(shape.robot);
        auto X_camera_0 = X_0_camera.inv();
        auto X_relative_marker = shape.frameOffset;
        auto X_0_marker = X_relative_marker * robot.frame(shape.frame).position();
        shape.update(X_0_marker * X_camera_0, X_0_camera);
      };
    }
    // Simulate marker update
    updateThread_ = std::thread(
        [this, markerUpdates_]()
        {
          ros::Rate rt(30);
          while(ros::ok() && running_)
          {
            for(auto & m : markerUpdates_)
            {
              std::lock_guard<std::mutex> lock(updateMutex_);
              m.second(ctl_, readLshapes_[m.first]);
            }
            rt.sleep();
          }
        });
  }
  else
  {
    boost::function<void(const whycon_lshape::WhyConLShapeMsg &)> callback_ =
        [this](const whycon_lshape::WhyConLShapeMsg & msg)
    {
      for(const auto & s : msg.shapes)
      {
        const auto & name = s.name;
        std::lock_guard<std::mutex> lock(updateMutex_);
        if(readLshapes_.count(name))
        { // supported marker
          Eigen::Vector3d pos{s.pose.position.x, s.pose.position.y, s.pose.position.z};
          Eigen::Quaterniond q{s.pose.orientation.w, s.pose.orientation.x, s.pose.orientation.y, s.pose.orientation.z};
          readLshapes_[name].update({q, pos}, X_0_camera);
        }
      }
    };
    methodConf("topic", topic_);
    try
    {
      sub_ = nh_->subscribe<whycon_lshape::WhyConLShapeMsg>(topic_, 1000, callback_);
    }
    catch(...)
    {
      mc_rtc::log::warning("[WhyconPluginPlugin] Could not connect to topic {} (invalid name)", topic_);
      connected_ = false;
    }
  }

  ctl_.gui()->addElement({"Plugins", "WhyCon"},
                         mc_rtc::gui::Label("Status",
                                            [this]()
                                            {
                                              if(simulation_)
                                              {
                                                return "simulation";
                                              }
                                              else
                                              {
                                                return connected_ ? "connected" : "disconnected";
                                              }
                                            }),
                         mc_rtc::gui::Label("Topic", [this]() { return topic_; }));
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
  if(sub_.getNumPublishers() > 0)
  {
    if(!connected_)
    {
      mc_rtc::log::success("[WhyconPluginPlugin] Connected to topic \"{}\"", topic_);
      connected_ = true;
    }
  }
  else
  {
    if(connected_)
    {
      mc_rtc::log::warning("[WhyconPluginPlugin] All publishers disconnected from topic \"{}\"", topic_);
      connected_ = false;
    }
  }
  {
    std::lock_guard<std::mutex> lock(updateMutex_);
    for(auto & [name, lshape] : readLshapes_)
    {
      lshape.tick(dt);
    }
    lshapes_ = readLshapes_;
  }
  for(auto & [name, lshape] : lshapes_)
  {
    if(!ctl_.datastore().has("WhyconPlugin::Marker::" + name))
    {
      newMarker(name);
    }
    else
    {
      ctl_.datastore().assign(
          "WhyconPlugin::Marker::" + name,
          std::pair<sva::PTransformd, double>(lshapes_.at(name).posW, lshapes_.at(name).lastUpdate()));
    }

    // auto & markerFrame = ctl_.robot(lshape.robot).frame("WhyconMarker_" + name);
    // const auto & parentFrame = ctl_.robot(lshape.robot).frame(lshape.frame);
    // markerFrame.X_p_f(lshape.posW * parentFrame.position().inv());
  }
}

bool WhyConSubscriber::visible(const std::string & marker) const
{
  return lshapes_.count(marker) && lshapes_.at(marker).visible;
}

const sva::PTransformd & WhyConSubscriber::X_camera_marker(const std::string & marker) const
{
  return lshapes_.at(marker).pos;
}

const sva::PTransformd & WhyConSubscriber::X_0_marker(const std::string & marker) const
{
  return lshapes_.at(marker).posW;
}

void WhyConSubscriber::newMarker(const std::string & name)
{
  mc_rtc::log::info("[WhyConSubscriber] New marker: {}", name);
  ctl_.logger().addLogEntry("WhyConMarkers_" + name,
                            [this, name]() -> const sva::PTransformd & { return lshapes_.at(name).pos; });
  ctl_.logger().addLogEntry("WhyConMarkers_" + name + "_World",
                            [this, name]() -> const sva::PTransformd & { return lshapes_.at(name).posW; });
  ctl_.datastore().make<std::pair<sva::PTransformd, double>>("WhyconPlugin::Marker::" + name, lshapes_.at(name).posW,
                                                             lshapes_.at(name).lastUpdate());
  auto gui = ctl_.gui();
  if(!gui)
  {
    return;
  }
  gui->addElement({"Plugins", "WhyCon", "Markers"},
                  mc_rtc::gui::Transform(name, [this, name]() { return lshapes_.at(name).posW; }));
}

} // namespace whycon_plugin

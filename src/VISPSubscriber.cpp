#include <sn_walking/vision/VISPSubscriber.h>

// To load dynamically a robot
#include <mc_rbdyn/RobotLoader.h>

// ROS stuff
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <comanoid_bracket_grasping/PoseStampedStatus.h>

namespace sn_walking
{

namespace vision
{

VISPSubscriber::VISPSubscriber(Controller & ctl, const mc_rtc::Configuration & config)
: nh_(mc_rtc::ROSBridge::get_node_handle()), ctl_(ctl)
{
  if(!nh_)
  {
    LOG_ERROR_AND_THROW(std::runtime_error, "[VISPSubscriber] ROS is not available")
  }
  bool simulation = config("simulation");
  auto methodConf = config("visp");
  auto objects = methodConf("objects");

  for(auto k : objects.keys())
  {
    std::string robot = objects(k)("robot");
    std::string tf = objects(k)("tf");
    sva::PTransformd offset = objects(k)("offset", sva::PTransformd::Identity());
    if(objects(k).has("load") && simulation)
    {
      Eigen::Vector3d trans = objects(k)("load")("translation");
      Eigen::Vector3d rot = objects(k)("load")("rotation");
      sva::PTransformd base(mc_rbdyn::rpyToMat(rot.x(), rot.y(), rot.z()), trans);
      auto object =
          mc_rbdyn::RobotLoader::get_robot_module(std::string("env"), std::string(mc_rtc::MC_ENV_DESCRIPTION_PATH), k);
      ctl.robots().load(*object, &base);
    }
    objects_[k] = {k, robot, tf, offset};
    objects(k)("yawOnly", objects_[k].yawOnly);
    objects_[k].posWorld = sva::PTransformd::Identity();
    objects(k)("posWorld", objects_[k].posWorld);
    newObject(k);
  }

  if(simulation)
  {
    updateThread_ = std::thread([this]() {
      ros::Rate rt(30);
      while(ros::ok() && running_)
      {
        auto X_camera_0 = ctl_.cameraPose().inv();

        for(auto & o : objects_)
        {
          std::lock_guard<std::mutex> lock(updateMutex_);
          sva::PTransformd X_0_object;
          if(o.second.robot == "world")
          {
            X_0_object = o.second.posWorld;
          }
          else if(ctl_.robots().robot(o.second.robot).hasSurface(o.second.tf))
          {
            X_0_object =
                ctl_.robots().robot(o.second.robot).surface(o.second.tf).X_0_s(ctl_.robots().robot(o.second.robot));
          }
          else
          {
            X_0_object = ctl_.robots().robot(o.second.robot).bodyPosW(o.second.tf);
            if(o.second.yawOnly)
            {
              auto rpy = mc_rbdyn::rpyFromMat(X_0_object.rotation());
              X_0_object.rotation() = mc_rbdyn::rpyToMat(0, 0, rpy.z());
            }
          }
          o.second.update(X_0_object * X_camera_0, X_camera_0.inv());
        }
        rt.sleep();
      }
    });
  }
  else
  {
    for(auto & o : objects_)
    {
      using msg_t = geometry_msgs::PoseStamped;
      boost::function<void(const msg_t &)> callback_ = [this, &o](const msg_t & msg) {
        auto X_0_camera = ctl_.cameraPose();
        sva::PTransformd X_camera_object = sva::PTransformd::Identity();
        auto & pos = msg.pose.position;
        auto & rot = msg.pose.orientation;
        X_camera_object.translation() = Eigen::Vector3d(pos.x, pos.y, pos.z);
        X_camera_object.rotation() =
            Eigen::Matrix3d(Eigen::Quaterniond(rot.w, rot.x, rot.y, rot.z).inverse().normalized());
        o.second.update(X_camera_object, X_0_camera);
      };
      subs_.push_back(nh_->subscribe<msg_t>(o.second.name + "/pose_hand", 10, callback_));
      using visp_msg_t = comanoid_bracket_grasping::PoseStampedStatus;
      boost::function<void(const visp_msg_t &)> visp_callback_ = [&o](const visp_msg_t & msg) {
        o.second.visible = msg.status;
      };
    }
  }
}

VISPSubscriber::~VISPSubscriber()
{
  if(updateThread_.joinable())
  {
    running_ = false;
    updateThread_.join();
  }
}

void VISPSubscriber::tick(double dt)
{
  std::lock_guard<std::mutex> lock(updateMutex_);
  for(auto & o : objects_)
  {
    o.second.tick(dt);
  }
}

bool VISPSubscriber::visible(const std::string & object) const
{
  std::lock_guard<std::mutex> lock(updateMutex_);
  return objects_.count(object) && objects_.at(object).visible;
}

const sva::PTransformd & VISPSubscriber::X_camera_object(const std::string & object) const
{
  std::lock_guard<std::mutex> lock(updateMutex_);
  return objects_.at(object).pos;
}

const sva::PTransformd & VISPSubscriber::X_0_object(const std::string & object) const
{
  std::lock_guard<std::mutex> lock(updateMutex_);
  return objects_.at(object).posW;
}

void VISPSubscriber::newObject(const std::string & name)
{
  LOG_INFO("[VISPSubscriber] New object: " << name)
  ctl_.logger().addLogEntry("VISPObjects_" + name,
                            [this, name]() -> const sva::PTransformd & { return objects_.at(name).pos; });
  ctl_.logger().addLogEntry("VISPObjects_" + name + "_World",
                            [this, name]() -> const sva::PTransformd & { return objects_.at(name).posW; });
  auto gui = ctl_.gui();
  if(!gui)
  {
    return;
  }
  gui->addElement({"VISP", "Objects"}, mc_rtc::gui::Transform(name, [this, name]() { return objects_.at(name).posW; },
                                                              [this, name](const sva::PTransformd & pt) {
                                                                auto & o = objects_.at(name);
                                                                if(o.robot == "world")
                                                                {
                                                                  o.posWorld = pt;
                                                                }
                                                              }));
}

} // namespace vision

} // namespace sn_walking

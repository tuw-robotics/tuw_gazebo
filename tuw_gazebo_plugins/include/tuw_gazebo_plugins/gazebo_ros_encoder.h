#ifndef GAZEBO_ROS_ENCODER
#define GAZEBO_ROS_ENCODER

#include <string>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

#include <tuw_gazebo_plugins/PubQueue.h>
#include <tuw_gazebo_plugins/gazebo_ros_utils.h>
#include <tuw_gazebo_plugins/noise_sim_model.h>

#include <tuw_vehicle_msgs/Wheelspeeds.h>

namespace gazebo {

class GazeboRosEncoder : public ModelPlugin {
 public:
  GazeboRosEncoder();
  ~GazeboRosEncoder();
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) override;

 private:
  void Update();

  physics::ModelPtr parent_;
  physics::JointPtr joints_[4];

  event::ConnectionPtr update_connection_;

  common::Time last_update_time_;
  double update_period_;

  GazeboRosPtr gazebo_ros_;
  PubMultiQueue pub_multi_queue_;
  PubQueue<tuw_vehicle_msgs::Wheelspeeds>::Ptr pub_queue_;
  ros::Publisher pub_;
  std::string topic_name_;

  tuw::NoiseSimModel noise_[4];
};
}

#endif  // GAZEBO_ROS_ENCODER

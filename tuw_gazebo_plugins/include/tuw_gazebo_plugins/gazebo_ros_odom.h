#ifndef ODOM_PLUGIN_H
#define ODOM_PLUGIN_H

#include <gazebo/common/common.hh>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tuw_gazebo_plugins/gazebo_ros_utils.h>

namespace gazebo {

class GazeboRosOdom : public ModelPlugin {
public:
  GazeboRosOdom();
  ~GazeboRosOdom();
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) override;
  void Reset() override;
  void Init() override;

protected:
  void UpdateChild();
  void FiniChild();

private:
  void publishOdometry(void);

  GazeboRosPtr gazebo_ros_;
  physics::ModelPtr parent_;
  event::ConnectionPtr update_connection_;

  ros::Publisher pubOdom_;
  tf::TransformBroadcaster tfBroadcaster_;

  std::string baseLinkFrame_;
  std::string odomFrame_;
  std::string odomTopic_;

  // ros::CallbackQueue queue_;
  boost::mutex lock;
  common::Time lastUpdateTime_;

  nav_msgs::Odometry odom_;
};
}

#endif
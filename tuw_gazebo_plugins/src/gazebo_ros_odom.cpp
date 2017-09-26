#include "tuw_gazebo_plugins/gazebo_ros_odom.h"
#include <ros/advertise_options.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/TransformStamped.h>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

namespace gazebo {

GazeboRosOdom::GazeboRosOdom() { pubOdom_.shutdown(); }

GazeboRosOdom::~GazeboRosOdom() {
  this->update_connection_.reset();
}

void GazeboRosOdom::Load(physics::ModelPtr parent, sdf::ElementPtr sdf) {
  this->parent_ = parent;
  gazebo_ros_ = GazeboRosPtr(new GazeboRos(parent, sdf, "Odom"));
  gazebo_ros_->isInitialized();

  int updateRate;

  gazebo_ros_->getParameter<std::string>(odomTopic_, "odomTopic", "odom_gt");
  gazebo_ros_->getParameter<std::string>(odomFrame_, "odomFrame", "odom_gt");
  gazebo_ros_->getParameter<int>(updateRate, "updateRate", 50);

  updatePeriod_ = 1. / (double)updateRate;

  lastUpdateTime_ = parent_->GetWorld()->SimTime();

  pubOdom_ = gazebo_ros_->node()->advertise<nav_msgs::Odometry>(odomTopic_, 1);
  ROS_INFO("%s: Advertising odom on %s", gazebo_ros_->info(),
           odomTopic_.c_str());

  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboRosOdom::UpdateChild, this));
}

void GazeboRosOdom::Init() { gazebo::ModelPlugin::Init(); }

void GazeboRosOdom::Reset() {
  gazebo::ModelPlugin::Reset();
  lastUpdateTime_ = parent_->GetWorld()->SimTime();
}

void GazeboRosOdom::UpdateChild() {
  common::Time current_time = parent_->GetWorld()->SimTime();

  if ((current_time - lastUpdateTime_).Double() > updatePeriod_) {
    publishOdometry();
    lastUpdateTime_ = current_time;
  }
}

void GazeboRosOdom::publishOdometry() {

  ros::Time current_time = ros::Time::now();
  std::string mapFrame = "map";

  ignition::math::Pose3d pose = parent_->WorldPose();

  double px = pose.Pos().X(), py = pose.Pos().Y(), pz = pose.Pos().Z();
  double qx = pose.Rot().X(), qy = pose.Rot().Y(), qz = pose.Rot().Z(), qw = pose.Rot().W();

  geometry_msgs::TransformStamped tfStamped;
  tfStamped.header.stamp = current_time;
  tfStamped.header.frame_id = "map";
  tfStamped.child_frame_id = odomFrame_;
  tfStamped.transform.translation.x = px;
  tfStamped.transform.translation.y = py;
  tfStamped.transform.translation.z = pz;
  tfStamped.transform.rotation.x = qx;
  tfStamped.transform.rotation.y = qy;
  tfStamped.transform.rotation.z = qz;
  tfStamped.transform.rotation.w = qw;

  odom_.pose.pose.position.x = px;
  odom_.pose.pose.position.y = py;
  odom_.pose.pose.position.z = pz;
  odom_.pose.pose.orientation.x = qx;
  odom_.pose.pose.orientation.y = qy;
  odom_.pose.pose.orientation.z = qz;
  odom_.pose.pose.orientation.w = qw;

  // get velocity in /odom frame
  ignition::math::Vector3d linear;
  linear = parent_->WorldLinearVel();
  odom_.twist.twist.angular.z = parent_->WorldAngularVel().Z();

  // convert velocity to child_frame_id (aka base_footprint)
  float yaw = pose.Rot().Yaw();
  odom_.twist.twist.linear.x =
      (cosf(yaw) * linear.X() + sinf(yaw) * linear.Y());
  odom_.twist.twist.linear.y =
      (cosf(yaw) * linear.Y() - sinf(yaw) * linear.X());

  // set covariance
  odom_.pose.covariance[0] = 0.00001;
  odom_.pose.covariance[7] = 0.00001;
  odom_.pose.covariance[14] = 1000000000000.0;
  odom_.pose.covariance[21] = 1000000000000.0;
  odom_.pose.covariance[28] = 1000000000000.0;
  odom_.pose.covariance[35] = 0.001;

  // set header
  odom_.header.stamp = current_time;
  odom_.header.frame_id = mapFrame;
  odom_.child_frame_id = odomFrame_;

  tfBroadcaster_.sendTransform(tfStamped);
  pubOdom_.publish(odom_);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosOdom)
}

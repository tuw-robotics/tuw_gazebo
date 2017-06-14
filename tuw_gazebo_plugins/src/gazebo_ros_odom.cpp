#include "tuw_gazebo_plugins/gazebo_ros_odom.h"
#include <ros/advertise_options.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

namespace gazebo {

GazeboRosOdom::GazeboRosOdom() { pubOdom_.shutdown(); }

GazeboRosOdom::~GazeboRosOdom() {
  event::Events::DisconnectWorldUpdateBegin(this->update_connection_);
}

void GazeboRosOdom::Load(physics::ModelPtr parent, sdf::ElementPtr sdf) {
  this->parent_ = parent;
  gazebo_ros_ = GazeboRosPtr(new GazeboRos(parent, sdf, "Odom"));
  gazebo_ros_->isInitialized();

  gazebo_ros_->getParameter<std::string>(odomTopic_, "odomTopic", "odom_gt");
  gazebo_ros_->getParameter<std::string>(odomFrame_, "odomFrame", "odom_gt");
  gazebo_ros_->getParameter<std::string>(baseLinkFrame_, "baseLinkFrame",
                                         "base_link_gt");

  lastUpdateTime_ = parent_->GetWorld()->SimTime();

  pubOdom_ = gazebo_ros_->node()->advertise<nav_msgs::Odometry>(odomTopic_, 1);
  ROS_INFO("%s: Advertising odom on %s", gazebo_ros_->info(),
           odomTopic_.c_str());

  // this->callback_queue_thread_ =
  //     boost::thread(boost::bind(&GazeboRosOdom::QueueThread, this));
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
  common::Time dt_sec = (current_time - lastUpdateTime_);
  lastUpdateTime_ = current_time;
  publishOdometry();
}

void GazeboRosOdom::publishOdometry() {
  ros::Time current_time = ros::Time::now();
  std::string mapFrame = "map";

  tf::Quaternion qt;
  tf::Vector3 vt;

  // getting data form gazebo world
  ignition::math::Pose3d pose = parent_->WorldPose();
  qt = tf::Quaternion(pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(),
                      pose.Rot().W());
  vt = tf::Vector3(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());

  odom_.pose.pose.position.x = vt.x();
  odom_.pose.pose.position.y = vt.y();
  odom_.pose.pose.position.z = vt.z();

  odom_.pose.pose.orientation.x = qt.x();
  odom_.pose.pose.orientation.y = qt.y();
  odom_.pose.pose.orientation.z = qt.z();
  odom_.pose.pose.orientation.w = qt.w();

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

  tf::Transform base_footprint_to_odom(qt, vt);
  tfBroadcaster_.sendTransform(tf::StampedTransform(
      base_footprint_to_odom, current_time, mapFrame, odomFrame_));

  tf::Transform zeroTf;
  zeroTf.setOrigin(tf::Vector3(0, 0, 0));
  zeroTf.setRotation(tf::Quaternion(0, 0, 0, 1));
  tfBroadcaster_.sendTransform(
      tf::StampedTransform(zeroTf, current_time, odomFrame_, baseLinkFrame_));

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

  pubOdom_.publish(odom_);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosOdom)
}

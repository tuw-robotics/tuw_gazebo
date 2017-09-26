#ifndef GAZEBO_ROS_MAP_H
#define GAZEBO_ROS_MAP_H

#include <string>
#include <vector>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <ros/ros.h>

#include <tuw_gazebo_plugins/PubQueue.h>
#include <tuw_gazebo_plugins/gazebo_ros_utils.h>
#include <tuw_gazebo_plugins/map_common.h>

#include <tuw_object_msgs/ObjectDetection.h>
#include <tuw_object_msgs/ObjectWithCovarianceArray.h>

namespace gazebo {

typedef struct {

  std::string nodeHandleName;

  std::string topicName;

  std::string frameId;

  double updateRate;

  double objectCovariancePos;

  DetectionConfig detectionConfig;
} GazeboRosMapOptions;

class GazeboRosMap : public ModelPlugin {

public:
  GazeboRosMap();
  ~GazeboRosMap();
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

private:
  void ParseOpt();
  void Update();

  physics::ModelPtr parent_;
  GazeboRosPtr gazebo_ros_;

  event::ConnectionPtr update_connection_;
  common::Time last_update_time_;
  double update_period_;

  std::unique_ptr<ros::NodeHandle> rosNode_;
  PubMultiQueue pub_multi_queue_;
  PubQueue<tuw_object_msgs::ObjectWithCovarianceArray>::Ptr pub_queue_;
  ros::Publisher pub_;

  std::vector<physics::ModelPtr> cones_;
  GazeboRosMapOptions options_;
};
}

#endif // GAZEBO_ROS_CONES_GROUNDTRUTH_PUBLISHER

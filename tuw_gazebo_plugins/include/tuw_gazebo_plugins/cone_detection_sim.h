#ifndef cone_detection_sim
#define cone_detection_sim

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
#include <tuw_gazebo_plugins/noise_sim_model.h>

#include <tuw_object_msgs/Object.h>
#include <tuw_object_msgs/ObjectDetection.h>
#include <tuw_object_msgs/ObjectWithCovariance.h>

#include <dynamic_reconfigure/server.h>
#include <tuw_gazebo_plugins/ConeDetectionSimConfig.h>

namespace gazebo {

typedef struct {

  std::string nodeHandleName;

  std::string topicName;

  std::string frameId;

  DetectionConfig detectionConfig;

} ConeDetectionSimOptions;

class ConeDetectionSim : public ModelPlugin {

public:
  ConeDetectionSim();
  ~ConeDetectionSim();
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

  void callbackConfig(tuw_gazebo_plugins::ConeDetectionSimConfig &_config,
                      uint32_t _level);
  void setVisualDetectionCovariance(tuw_object_msgs::ObjectWithCovariance &owc);

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
  PubQueue<tuw_object_msgs::ObjectDetection>::Ptr pub_queue_;
  ros::Publisher pub_;

  std::shared_ptr<
      dynamic_reconfigure::Server<tuw_gazebo_plugins::ConeDetectionSimConfig>>
      reconfigureServer_;
  dynamic_reconfigure::Server<
      tuw_gazebo_plugins::ConeDetectionSimConfig>::CallbackType reconfigureFnc_;
  std::vector<physics::ModelPtr> cones_;
  ConeDetectionSimOptions options_;
  tuw_gazebo_plugins::ConeDetectionSimConfig config_;

  tuw::NoiseSimModel noiseX_;
  tuw::NoiseSimModel noiseY_;
};
}

#endif // GAZEBO_ROS_CONES_GROUNDTRUTH_PUBLISHER

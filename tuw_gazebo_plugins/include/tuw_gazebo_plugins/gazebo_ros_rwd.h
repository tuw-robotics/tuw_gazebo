#ifndef RWD_PLUGIN_H
#define RWD_PLUGIN_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tuw_gazebo_plugins/HV_battery.h>
#include <tuw_gazebo_plugins/aero_forces.h>
#include <tuw_gazebo_plugins/gazebo_ros_utils.h>
#include <tuw_gazebo_plugins/noise_sim_model.h>
#include <tuw_vehicle_msgs/RWDControl.h>
#include <boost/thread.hpp>
#include <gazebo/common/common.hh>

#define TO_PASCAL(bar) (bar * 100000.0)
#define TO_RADIANS(dg) ((dg)*M_PI / 180.0)
#define TO_RADPS(rpm) (M_PI / 30 * rpm)

namespace gazebo {
class GazeboRosRWD : public ModelPlugin {
 public:
  GazeboRosRWD();
  ~GazeboRosRWD();
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) override;
  void Reset() override;
  void Init() override;

 protected:
  void UpdateChild();
  void FiniChild();

 private:
  void QueueThread();
  void cmdRWDCallback(const tuw_vehicle_msgs::RWDControl::ConstPtr &rwd_msg);
  double GetLeftToeAngle(double steeringAngle);
  double GetRightToeAngle(double steeringAngle);
  double GetMaxVelocity();
  void WheelForces();
  void SetWheelForce(physics::JointPtr &wheel, double brakeTorque,
                     double torque);

  GazeboRosPtr gazebo_ros_;
  physics::ModelPtr parent_;
  event::ConnectionPtr update_connection_;
  physics::JointPtr leftRearJoint_;
  physics::JointPtr rightRearJoint_;
  physics::JointPtr leftFrontJoint_;
  physics::JointPtr rightFrontJoint_;
  physics::JointPtr leftSteeringJoint_;
  physics::JointPtr rightSteeringJoint_;

  ros::Subscriber cmd_rwd_subscriber_;
  ros::Publisher battery_state_publisher_;
  ros::Publisher chassis_state_publisher_;
  ros::Publisher odometry_publisher_;
  tf::TransformBroadcaster transform_broadcaster_;
  boost::mutex lock;

  std::string robot_namespace_;
  std::string command_topic_;
  std::string batteryStateTopic_;
  std::string chassisStateTopic_;
  std::string baseLinkName_;

  ros::CallbackQueue queue_;
  boost::thread callback_queue_thread_;

  bool alive_;

  double update_rate_;
  double update_period_;
  common::Time last_update_time_;
  std::vector<double> steeringAngles_;
  std::vector<double> leftToeAngles_;

  common::PID leftSteerPID_;
  common::PID rightSteerPID_;

  double torqueLeft_;
  double torqueRight_;
  double steering_;
  double brakePressure_;

  physics::LinkPtr baseLink_;

  double brakeBalance_;
  double frontBrakeCoefficient_;
  double rearBrakeCoefficient_;

  HVBattery hvBattery;
  AeroModel aeroModel_;

  double motorPsiM_;
  double gearTransmission_;
  double maxVelocity_;
  double powertrainEfficiency_;

  tuw::NoiseSimModel steeringAngleNoise_;
  common::Time lastRWDTime_;
};
}

#endif

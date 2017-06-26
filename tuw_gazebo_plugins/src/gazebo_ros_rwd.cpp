#include <nav_msgs/Odometry.h>
#include <ros/advertise_options.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <tuw_gazebo_plugins/aero_forces.h>
#include <tuw_gazebo_plugins/gazebo_ros_rwd.h>
#include <tuw_gazebo_plugins/gazebo_ros_utils.h>
#include <tuw_vehicle_msgs/BatteryState.h>
#include <tuw_vehicle_msgs/ChassisState.h>
#include <tuw_vehicle_msgs/RWDControl.h>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

namespace gazebo {
GazeboRosRWD::GazeboRosRWD() {
  odometry_publisher_.shutdown();

  // from the famous excel table
  leftToeAngles_ = {-32.535, -29.422, -26.304, -23.171, -20.015, -16.827,
                    -13.597, -10.312, -6.961,  -3.53,   0,       3.648,
                    7.441,   11.413,  15.608,  20.092,  24.959,  30.368,
                    36.611,  44.391,  57.063};
  steeringAngles_ = {-90, -81, -72, -63, -54, -45, -36, -27, -18, -9, 0,
                     9,   18,  27,  36,  45,  54,  63,  72,  81,  90};

  if (leftToeAngles_.size() != steeringAngles_.size()) {
    ROS_ERROR("leftToeAngles_ and steeringAngles_ size differ");
  }

  torqueLeft_ = 0;
  torqueRight_ = 0;
  steering_ = 0;
  brakePressure_ = 0;
}

GazeboRosRWD::~GazeboRosRWD() {
  event::Events::DisconnectWorldUpdateBegin(this->update_connection_);

  cmd_rwd_subscriber_.shutdown();

  queue_.clear();
  queue_.disable();
  alive_ = false;
  callback_queue_thread_.join();
}

void GazeboRosRWD::Load(physics::ModelPtr parent, sdf::ElementPtr sdf) {
  this->parent_ = parent;
  gazebo_ros_ = GazeboRosPtr(new GazeboRos(parent, sdf, "TUWRRWDrive"));
  gazebo_ros_->isInitialized();

  aeroModel_.LoadParam(sdf->GetElement("aero"));

  gazebo_ros_->getParameter<std::string>(command_topic_, "commandTopic",
                                         "cmd_rwd");
  gazebo_ros_->getParameter<std::string>(batteryStateTopic_,
                                         "batteryStateTopic", "battery_state");
  gazebo_ros_->getParameter<std::string>(chassisStateTopic_,
                                         "chassisStateTopic", "chassis");
  gazebo_ros_->getParameter<std::string>(baseLinkName_, "baseLink",
                                         "base_link");
  gazebo_ros_->getParameter<double>(update_rate_, "updateRate", 100.0);

  double frontBrakeApplicationArea;
  double frontBrakeDiscRadius;
  double rearBrakeApplicationArea;
  double rearBrakeDiscRadius;
  double brakeDiscFriction;

  gazebo_ros_->getParameter<double>(frontBrakeApplicationArea,
                                    "frontBrakeApplicationArea", 0.00202);
  gazebo_ros_->getParameter<double>(frontBrakeDiscRadius,
                                    "frontBrakeDiscRadius", 0.0875);
  gazebo_ros_->getParameter<double>(rearBrakeApplicationArea,
                                    "rearBrakeApplicationArea", 0.00202);
  gazebo_ros_->getParameter<double>(rearBrakeDiscRadius, "rearBrakeDiscRadius",
                                    0.0875);
  gazebo_ros_->getParameter<double>(brakeBalance_, "brakeBalance", 0.33);
  gazebo_ros_->getParameter<double>(brakeDiscFriction, "brakeDiscFriction",
                                    0.7);

  frontBrakeCoefficient_ =
      frontBrakeApplicationArea * brakeDiscFriction * frontBrakeDiscRadius;
  rearBrakeCoefficient_ =
      rearBrakeApplicationArea * brakeDiscFriction * rearBrakeDiscRadius;

  hvBattery.initFromParameters(gazebo_ros_);
  gazebo_ros_->getParameter<double>(motorPsiM_, "motorPsiM", 0.03);
  gazebo_ros_->getParameter<double>(gearTransmission_, "gearTransmission",
                                    11.9654);

  double gearboxEfficiency;
  gazebo_ros_->getParameter<double>(gearboxEfficiency, "gearboxEfficiency",
                                    0.96);
  // inverter calcs /12 instead of /gearTransmission
  powertrainEfficiency_ = (1.0 / 12.0) * gearTransmission_ * gearboxEfficiency;

  leftRearJoint_ = gazebo_ros_->getJoint(parent_, "leftRearJoint",
                                         "rear_left_powertrain_joint");
  rightRearJoint_ = gazebo_ros_->getJoint(parent_, "rightRearJoint",
                                          "rear_right_powertrain_joint");
  leftFrontJoint_ =
      gazebo_ros_->getJoint(parent_, "leftFrontJoint", "front_left_hub_joint");
  rightFrontJoint_ = gazebo_ros_->getJoint(parent_, "rightFrontJoint",
                                           "front_right_hub_joint");
  leftSteeringJoint_ = gazebo_ros_->getJoint(parent_, "leftSteeringJoint",
                                             "left_steering_joint");
  rightSteeringJoint_ = gazebo_ros_->getJoint(parent_, "rightSteeringJoint",
                                              "right_steering_joint");

  bool foundNoiseSdf = steeringAngleNoise_.loadParam(
      gazebo_ros_->Sdf()->GetElement("steering_angle_noise"));

  maxVelocity_ = GetMaxVelocity();
  leftRearJoint_->SetVelocityLimit(0, maxVelocity_);
  rightRearJoint_->SetVelocityLimit(0, maxVelocity_);

  baseLink_ = parent_->GetLink(baseLinkName_);

  leftSteerPID_ =
      common::PID(100.0, 50.0, 1.0, 1000.0, -1000.0, 1000.0, -1000.0);
  rightSteerPID_ =
      common::PID(100.0, 50.0, 1.0, 1000.0, -1000.0, 1000.0, -1000.0);

  if (this->update_rate_ > 0.0) {
    this->update_period_ = 1.0 / this->update_rate_;
  } else {
    this->update_period_ = 0.0;
  }
  last_update_time_ = parent_->GetWorld()->SimTime();

  alive_ = true;

  ROS_INFO("%s: Try to subscribe to %s!", gazebo_ros_->info(),
           command_topic_.c_str());

  ros::SubscribeOptions so =
      ros::SubscribeOptions::create<tuw_vehicle_msgs::RWDControl>(
          command_topic_, 1,
          boost::bind(&GazeboRosRWD::cmdRWDCallback, this, _1), ros::VoidPtr(),
          &queue_);

  cmd_rwd_subscriber_ = gazebo_ros_->node()->subscribe(so);
  battery_state_publisher_ =
      gazebo_ros_->node()->advertise<tuw_vehicle_msgs::BatteryState>(
          batteryStateTopic_.c_str(), 10);
  chassis_state_ =
      gazebo_ros_->node()->advertise<tuw_vehicle_msgs::ChassisState>(
          chassisStateTopic_.c_str(), 10);

  ROS_INFO("%s: Subscribe to %s!", gazebo_ros_->info(), command_topic_.c_str());

  this->callback_queue_thread_ =
      boost::thread(boost::bind(&GazeboRosRWD::QueueThread, this));
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboRosRWD::UpdateChild, this));
}

void GazeboRosRWD::Init() { gazebo::ModelPlugin::Init(); }

void GazeboRosRWD::Reset() {
  gazebo::ModelPlugin::Reset();
  last_update_time_ = parent_->GetWorld()->SimTime();
}

void GazeboRosRWD::UpdateChild() {
  common::Time current_time = parent_->GetWorld()->SimTime();
  common::Time seconds_since_last_update = (current_time - last_update_time_);
  hvBattery.Update(seconds_since_last_update.Double());

  maxVelocity_ = GetMaxVelocity();

  double effortRight = rightSteerPID_.Update(rightSteeringJoint_->Position() -
                                                 GetRightToeAngle(steering_),
                                             seconds_since_last_update);
  double effortLeft = leftSteerPID_.Update(leftSteeringJoint_->Position() -
                                               GetLeftToeAngle(steering_),
                                           seconds_since_last_update);
  leftSteeringJoint_->SetForce(0, effortLeft);
  rightSteeringJoint_->SetForce(0, effortRight);
  WheelForces();
  aeroModel_.AddForces(parent_->RelativeLinearVel().X(), baseLink_);
  tuw::ros_msgs::BatteryState batteryStateMsg;
  batteryStateMsg.header.stamp = ros::Time(current_time.sec, current_time.nsec);
  batteryStateMsg.header.frame_id = gazebo_ros_->getNamespace();
  hvBattery.SetBatteryState(batteryStateMsg);
  battery_state_publisher_.publish(
      (tuw_vehicle_msgs::BatteryState)batteryStateMsg);

  tuw_vehicle_msgs::ChassisState chassisStateMsg;
  chassisStateMsg.header.stamp = ros::Time(current_time.sec, current_time.nsec);
  chassisStateMsg.header.frame_id = "0";
  chassisStateMsg.steeringAngle =
      steeringAngleNoise_.sim(steering_, seconds_since_last_update.Double());
  chassis_state_.publish(chassisStateMsg);

  last_update_time_ = current_time;
}

double GazeboRosRWD::GetMaxVelocity() {
  return TO_RADPS(hvBattery.GetCurrentVoltage() / motorPsiM_) /
         gearTransmission_;
}

void GazeboRosRWD::WheelForces() {
  double secSinceUpdate =
      (parent_->GetWorld()->SimTime() - lastRWDTime_).Double();
  if (secSinceUpdate > 1) {
    // if no rwd received, just stop
    SetWheelForce(leftRearJoint_, 30, 0);
    SetWheelForce(rightRearJoint_, 30, 0);
    SetWheelForce(leftFrontJoint_, 30, 0);
    SetWheelForce(rightFrontJoint_, 30, 0);
    return;
  }

  double pascalPressure = TO_PASCAL(brakePressure_);
  double brakeTorqueFront = pascalPressure * frontBrakeCoefficient_;
  double brakeTorqueRear =
      pascalPressure * brakeBalance_ * rearBrakeCoefficient_;
  double leftTorque = torqueLeft_ * powertrainEfficiency_;
  double rightTorque = torqueRight_ * powertrainEfficiency_;
  double resistanceConstant = 5;
  if (fabs(leftTorque) < 0.1) {
    double vel = leftRearJoint_->GetVelocity(0);
    leftTorque =
        -resistanceConstant * ignition::math::clamp(vel / 1.0, -1.0, 1.0);
  }
  if (fabs(rightTorque) < 0.1) {
    double vel = rightRearJoint_->GetVelocity(0);
    rightTorque =
        -resistanceConstant * ignition::math::clamp(vel / 1.0, -1.0, 1.0);
  }
  SetWheelForce(leftRearJoint_, brakeTorqueRear, leftTorque);
  SetWheelForce(rightRearJoint_, brakeTorqueRear, rightTorque);
  SetWheelForce(leftFrontJoint_, brakeTorqueFront, 0);
  SetWheelForce(rightFrontJoint_, brakeTorqueFront, 0);
}

void GazeboRosRWD::SetWheelForce(physics::JointPtr &wheel, double brakeTorque,
                                 double torque) {
  double vel = wheel->GetVelocity(0);
  if (ignition::math::isnan(vel)) {
    vel = 0;
  }
  double smoothing = 1;
  // weil es keine Bremskraft sondern eine Gegenkraft zur Rotation ist,
  // braucht man das smoothing um ein vorwärts/rückwärts Zirkulieren zu
  // verhindern
  // Gegenkraft bei Rädern mit Drehzahl < smoothing rad/s verringert
  brakeTorque *= -1 * ignition::math::clamp(vel / smoothing, -1.0, 1.0);
  if (vel >= maxVelocity_) {
    torque = 0;
  }
  double outTorque = torque + brakeTorque;
  wheel->SetForce(0, outTorque);
  hvBattery.AddDischargeWithMotor(torque, vel);
}

void GazeboRosRWD::FiniChild() {
  alive_ = false;
  queue_.clear();
  queue_.disable();
  gazebo_ros_->node()->shutdown();
  callback_queue_thread_.join();
}

void GazeboRosRWD::cmdRWDCallback(
    const tuw_vehicle_msgs::RWDControl::ConstPtr &rwd) {
  boost::mutex::scoped_lock scoped_lock(lock);
  torqueLeft_ = rwd->left_torque;
  torqueRight_ = rwd->right_torque;
  steering_ = rwd->steering_angle;
  brakePressure_ = fmax(0, rwd->brake_pressure);
  lastRWDTime_ = parent_->GetWorld()->SimTime();
}

void GazeboRosRWD::QueueThread() {
  static const double timeout = 0.01;

  while (alive_ && gazebo_ros_->node()->ok()) {
    queue_.callAvailable(ros::WallDuration(timeout));
  }
}

double GazeboRosRWD::GetLeftToeAngle(double steeringAngle) {
  int i, upperIndex, lowerIndex;
  bool upperFound = false;
  double upperSteering = 0, lowerSteering = 0, upperToe = 0, lowerToe = 0;
  for (i = 0; i < steeringAngles_.size(); i++) {
    upperSteering = steeringAngles_[i];
    if (upperSteering > steeringAngle) {
      upperFound = TRUE;
      break;
    }
  }

  if (!upperFound) {
    return TO_RADIANS(leftToeAngles_[steeringAngles_.size() - 1]);
  }
  upperIndex = i;
  if (1 == i) {
    return TO_RADIANS(leftToeAngles_[0]);
  }
  lowerIndex = i - 1;
  lowerSteering = steeringAngles_[lowerIndex];
  upperToe = leftToeAngles_[upperIndex];
  lowerToe = leftToeAngles_[lowerIndex];

  return TO_RADIANS(
      (lowerToe +
       ((steeringAngle - lowerSteering) / (upperSteering - lowerSteering)) *
           (upperToe - lowerToe)));
}

double GazeboRosRWD::GetRightToeAngle(double steeringAngle) {
  return -GetLeftToeAngle(-steeringAngle);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosRWD)
}

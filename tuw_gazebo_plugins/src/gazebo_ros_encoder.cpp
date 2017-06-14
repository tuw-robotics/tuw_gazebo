#include <tuw_gazebo_plugins/gazebo_ros_encoder.h>

namespace gazebo {

GazeboRosEncoder::GazeboRosEncoder() {}

GazeboRosEncoder::~GazeboRosEncoder() {
  pub_queue_.reset();
  pub_.shutdown();
}

void GazeboRosEncoder::Load(physics::ModelPtr parent, sdf::ElementPtr sdf) {
  this->parent_ = parent;
  gazebo_ros_ = GazeboRosPtr(new GazeboRos(parent, sdf, "TUWREncoder"));
  gazebo_ros_->isInitialized();

  gazebo_ros_->getParameter<std::string>(topic_name_, "topicName",
                                         "wheelspeeds");
  gazebo_ros_->getParameter<double>(update_period_, "updateRate", 100.0);
  update_period_ = 1. / update_period_;

  joints_[0] = gazebo_ros_->getJoint(parent_, "rightFrontJoint",
                                     "front_right_hub_joint");
  joints_[1] =
      gazebo_ros_->getJoint(parent_, "leftFrontJoint", "front_left_hub_joint");
  joints_[2] = gazebo_ros_->getJoint(parent_, "rightRearJoint",
                                     "rear_right_powertrain_joint");
  joints_[3] = gazebo_ros_->getJoint(parent_, "leftRearJoint",
                                     "rear_left_powertrain_joint");

  bool foundNoiseSdf = false;
  for (size_t i = 0; i < 4; i++) {
    foundNoiseSdf =
        foundNoiseSdf ||
        noise_[i].loadParam(gazebo_ros_->Sdf()->GetElement("encoder_noise"));
  }
  if (!foundNoiseSdf) {
    ROS_WARN(
        "%s: <encoder>: missing <noise type=\"tuw_advanced\"> defaults to "
        "no noise",
        gazebo_ros_->info());
  }

  pub_ = gazebo_ros_->node()->advertise<tuw_vehicle_msgs::Wheelspeeds>(topic_name_, 1);
  pub_multi_queue_.startServiceThread();
  pub_queue_ = pub_multi_queue_.addPub<tuw_vehicle_msgs::Wheelspeeds>();

  update_connection_ = event::Events::ConnectWorldUpdateEnd(
      boost::bind(&GazeboRosEncoder::Update, this));
}

static double radps2rpm(double radps) { return radps / M_PI * 30; }

// y axis (0 = x, 1 = y, 2 = z)
const int wheelRotationAxis = 1;

static double getJointRpm(const physics::JointPtr &joint) {
  double radps = joint->GetVelocity(wheelRotationAxis);
  return radps2rpm(radps);
}

void GazeboRosEncoder::Update() {
  common::Time current_time = parent_->GetWorld()->SimTime();
  double dt = (current_time - last_update_time_).Double();
  if (dt < update_period_) {
    return;
  }
  last_update_time_ = current_time;

  tuw_vehicle_msgs::Wheelspeeds msg;

  msg.header.stamp = ros::Time(current_time.sec, current_time.nsec);
  msg.header.frame_id = "0";

  msg.fr = noise_[0].sim(getJointRpm(joints_[0]), dt);
  msg.fl = noise_[1].sim(getJointRpm(joints_[1]), dt);
  msg.rr = noise_[2].sim(getJointRpm(joints_[2]), dt);
  msg.rl = noise_[3].sim(getJointRpm(joints_[3]), dt);

  pub_.publish(msg);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosEncoder)
}

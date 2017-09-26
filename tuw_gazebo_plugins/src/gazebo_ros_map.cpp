#include <tuw_gazebo_plugins/gazebo_ros_map.h>
#include <cstdlib>
#include <ctime>

namespace gazebo {

const double coneRadius = 0.2;

GazeboRosMap::GazeboRosMap() {}

GazeboRosMap::~GazeboRosMap() {
  pub_queue_.reset();
  pub_.shutdown();
}

void GazeboRosMap::ParseOpt() {
  gazebo_ros_->getParameter<std::string>(options_.nodeHandleName,
                                         "nodeHandleName", "");

  gazebo_ros_->getParameter<std::string>(options_.topicName, "topicName",
                                         "map_gt");

  gazebo_ros_->getParameter<std::string>(options_.frameId, "frameId", "map");

  gazebo_ros_->getParameter<double>(options_.updateRate, "updateRate", 10.0);

  gazebo_ros_->getParameter<double>(options_.detectionConfig.distanceMin,
                                    "distanceMin", 0.0);

  gazebo_ros_->getParameter<double>(options_.detectionConfig.distanceMax,
                                    "distanceMax", 0.0);

  gazebo_ros_->getParameter<double>(options_.detectionConfig.fovHorizontalDeg,
                                    "fovHorizontalDeg", 0.0);

  gazebo_ros_->getParameter<double>(options_.detectionConfig.fovVerticalDeg,
                                    "fovVerticalDeg", 0.0);

  gazebo_ros_->getParameter<double>(options_.objectCovariancePos,
                                    "objectCovariancePos", 0.1);

  update_period_ = 1.0 / options_.updateRate;
}

void GazeboRosMap::Load(physics::ModelPtr parent, sdf::ElementPtr sdf) {
  parent_ = parent;
  gazebo_ros_ = GazeboRosPtr(new GazeboRos(parent, sdf, "TUWRRWDrive"));
  gazebo_ros_->isInitialized();
  ParseOpt();

  rosNode_.reset(new ros::NodeHandle(options_.nodeHandleName));
  pub_ = rosNode_->advertise<tuw_object_msgs::ObjectWithCovarianceArray>(
      options_.topicName, 1);
  pub_multi_queue_.startServiceThread();
  pub_queue_ =
      pub_multi_queue_.addPub<tuw_object_msgs::ObjectWithCovarianceArray>();

  update_connection_ = event::Events::ConnectWorldUpdateEnd(
      boost::bind(&GazeboRosMap::Update, this));

  std::srand(std::time(0));
}

double getColorShapeVariable() {
}

void GazeboRosMap::Update() {
  common::Time current_time = parent_->GetWorld()->SimTime();
  double dt = (current_time - last_update_time_).Double();
  if (dt < update_period_) {
    return;
  }
  last_update_time_ = current_time;
  cones_ = getConesInWorld(parent_->GetWorld());

  tuw_object_msgs::ObjectWithCovarianceArray od;

  auto robotPose = parent_->WorldPose();
  double robotX = robotPose.Pos().X(), robotY = robotPose.Pos().Y();

  for (auto &cone : cones_) {
    ignition::math::Vector3d cpos = cone->WorldPose().Pos();

    tuw_object_msgs::ObjectWithCovariance owc;
    tuw_object_msgs::Object object;

    object.ids.clear();
    object.ids_confidence.clear();

    object.ids.push_back(1);
    object.ids_confidence.push_back(1);

    object.pose.position.x = cpos.X();
    object.pose.position.y = cpos.Y();
    object.pose.position.z = 0;
    object.pose.orientation.x = 0;
    object.pose.orientation.y = 0;
    object.pose.orientation.z = 0;
    object.pose.orientation.w = 1;

    object.shape = tuw_object_msgs::Object::SHAPE_TRAFFIC_CONE;
    object.shape_variables.resize(2);
    object.shape_variables[0] = coneRadius;
    object.shape_variables[1] = getConeColorShapeVariable(cone);

    owc.covariance_pose.resize(9);
    for (size_t i = 0; i < 9; i++) {
      owc.covariance_pose[i] = 0.0;
    }
    owc.covariance_pose[0] = options_.objectCovariancePos;
    owc.covariance_pose[4] = options_.objectCovariancePos;
    owc.covariance_pose[8] = options_.objectCovariancePos;

    owc.object = object;

    od.objects.push_back(owc);
  }

  od.header.stamp = ros::Time(current_time.sec, current_time.nsec);
  od.header.frame_id = options_.frameId;

  pub_.publish(od);

  ROS_DEBUG("%i cones in the map published", (int)od.objects.size());
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosMap)
}

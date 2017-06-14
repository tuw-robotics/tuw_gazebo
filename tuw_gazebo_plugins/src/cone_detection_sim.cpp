#include <tuw_gazebo_plugins/cone_detection_sim.h>
#include <cstdlib>
#include <ctime>

namespace gazebo {

ConeDetectionSim::ConeDetectionSim() {}

ConeDetectionSim::~ConeDetectionSim() {
  pub_queue_.reset();
  pub_.shutdown();
}

void ConeDetectionSim::ParseOpt() {
  gazebo_ros_->getParameter<std::string>(options_.nodeHandleName,
                                         "nodeHandleName", "");

  gazebo_ros_->getParameter<std::string>(options_.topicName, "topicName",
                                         "cones_gt");

  gazebo_ros_->getParameter<std::string>(options_.frameId, "frameId",
                                         "base_link");

  gazebo_ros_->getParameter<double>(options_.detectionConfig.distanceMin,
                                    "distanceMin", 0.0);

  gazebo_ros_->getParameter<double>(options_.detectionConfig.distanceMax,
                                    "distanceMax", 0.0);

  gazebo_ros_->getParameter<double>(options_.detectionConfig.fovHorizontalDeg,
                                    "fovHorizontalDeg", 0.0);

  gazebo_ros_->getParameter<double>(options_.detectionConfig.fovVerticalDeg,
                                    "fovVerticalDeg", 0.0);

  noiseX_.loadParam(gazebo_ros_->Sdf()->GetElement("detection_noise"));
  noiseY_.loadParam(gazebo_ros_->Sdf()->GetElement("detection_noise"));
}

void ConeDetectionSim::Load(physics::ModelPtr parent, sdf::ElementPtr sdf) {
  parent_ = parent;
  gazebo_ros_ = GazeboRosPtr(new GazeboRos(parent, sdf, "ConeDetectionSim"));
  gazebo_ros_->isInitialized();
  ParseOpt();

  rosNode_.reset(new ros::NodeHandle(options_.nodeHandleName));
  pub_ = rosNode_->advertise<tuw_object_msgs::ObjectDetection>(
      options_.topicName, 1);
  pub_multi_queue_.startServiceThread();
  pub_queue_ = pub_multi_queue_.addPub<tuw_object_msgs::ObjectDetection>();

  update_connection_ = event::Events::ConnectWorldUpdateEnd(
      boost::bind(&ConeDetectionSim::Update, this));

  reconfigureServer_ = std::make_shared<
      dynamic_reconfigure::Server<tuw_gazebo_plugins::ConeDetectionSimConfig>>(
      ros::NodeHandle("~/" + gazebo_ros_->getPluginName()));
  reconfigureFnc_ =
      boost::bind(&ConeDetectionSim::callbackConfig, this, _1, _2);
  reconfigureServer_->setCallback(reconfigureFnc_);

  std::srand(std::time(0));
}

void ConeDetectionSim::callbackConfig(
    tuw_gazebo_plugins::ConeDetectionSimConfig &_config, uint32_t _level) {
  config_ = _config;
  ROS_DEBUG("%s: callbackConfig!", gazebo_ros_->info());
}

double degToRad(double deg) { return deg / 180 * M_PI; }

void ConeDetectionSim::setVisualDetectionCovariance(
    tuw_object_msgs::ObjectWithCovariance &owc) {
  double pxHeight = 360.0;
  double pxWidth = 1280.0;
  // measured average pixel error of visual cone detection
  double pxErr = 5.0;
  // TODO by configuration, maximum range of the camera
  double maxRange = 20;
  // horizontal fov of the camera
  double radFov = degToRad(90);
  double bearingErrPerPxAtMaxRange = degToRad(10);

  // + 1 due to sampling noise
  double rangeNoise = maxRange / pxHeight * (pxErr + 1);
  double bearingNoise = radFov / pxWidth * (pxErr + 1);

  double fps = 30;
  double dt = 1 / fps;
  double frameGrabDelayRangeNoise = dt * 5;  // TODO use speed measurement

  double corr = bearingErrPerPxAtMaxRange / maxRange;

  owc.covariance_pose.resize(9);
  for (size_t i = 0; i < 9; i++) {
    owc.covariance_pose[i] = 0.0;
  }

  double rang = config_.sig_range * (rangeNoise + frameGrabDelayRangeNoise);
  double brng = config_.sig_bearing * bearingNoise;

  // 0 1 2     (0) (1)  -
  // x x 0 --> (3) (4)  -
  // x x 0      -   -  (8)
  owc.covariance_pose[0] = rang;
  owc.covariance_pose[1] = config_.sig_correlation * corr;
  owc.covariance_pose[3] = config_.sig_correlation * corr;
  owc.covariance_pose[4] = brng;
  owc.covariance_pose[8] = 1.0;
}

void ConeDetectionSim::Update() {
  common::Time current_time = parent_->GetWorld()->SimTime();
  double dt = (current_time - last_update_time_).Double();

  update_period_ = 1.0 / config_.fps;
  if (dt < update_period_) {
    return;
  }
  last_update_time_ = current_time;

  cones_ = getConesInWorldSeenBy(parent_->GetWorld(), parent_,
                                 options_.detectionConfig);

  tuw_object_msgs::ObjectDetection od;

  ignition::math::Pose3d robotPose = parent_->WorldPose();
  double robotX = robotPose.Pos().X(), robotY = robotPose.Pos().Y();

  const double coneColorBlue = 0;
  const double coneColorYellow = 1;
  const double coneColorRed = 2;

  for (physics::ModelPtr cone : cones_) {
    double r = ((double)std::rand()) / RAND_MAX;
    if (r <= config_.p_detection) {
      auto conePosition = cone->WorldPose().Pos();

      double x = conePosition.X() - robotX;
      double y = conePosition.Y() - robotY;

      x = noiseX_.sim(x, dt);
      y = noiseY_.sim(y, dt);

      tuw_object_msgs::ObjectWithCovariance owc;
      tuw_object_msgs::Object object;

      object.ids.clear();
      object.ids_confidence.clear();

      object.ids.push_back(1);
      object.ids_confidence.push_back(1);

      object.pose.position.x = x;
      object.pose.position.y = y;
      object.pose.position.z = 0;
      object.pose.orientation.x = 0;
      object.pose.orientation.y = 0;
      object.pose.orientation.z = 0;
      object.pose.orientation.w = 1;

      object.shape = tuw_object_msgs::Object::SHAPE_TRAFFIC_CONE;
      object.shape_variables.resize(2);
      object.shape_variables[0] = 0.2;

      if (cone->GetScopedName(false).find("cone_blue") != std::string::npos) {
        object.shape_variables[1] = coneColorBlue;
      } else if (cone->GetScopedName(false).find("cone_yellow") !=
                 std::string::npos) {
        object.shape_variables[1] = coneColorYellow;
      } else if (cone->GetScopedName(false).find("cone") != std::string::npos) {
        object.shape_variables[1] = coneColorRed;
      }
      setVisualDetectionCovariance(owc);
      owc.object = object;

      od.objects.push_back(owc);
    }
  }

  od.header.stamp = ros::Time(current_time.sec, current_time.nsec);
  od.header.frame_id = options_.frameId;

  od.distance_min = options_.detectionConfig.distanceMin;
  od.distance_max = options_.detectionConfig.distanceMax;
  od.fov_horizontal = options_.detectionConfig.fovHorizontalDeg;
  od.fov_vertical = options_.detectionConfig.fovVerticalDeg;

  od.type = tuw_object_msgs::ObjectDetection::OBJECT_TYPE_TRAFFIC_CONE;
  od.sensor_type =
      tuw_object_msgs::ObjectDetection::SENSOR_TYPE_GENERIC_MONOCULAR_VISION;

  pub_.publish(od);

  ROS_DEBUG("%i cones detected and published", (int)od.objects.size());
}

GZ_REGISTER_MODEL_PLUGIN(ConeDetectionSim)
}

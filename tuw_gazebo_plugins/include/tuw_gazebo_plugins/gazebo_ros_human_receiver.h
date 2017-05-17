#ifndef GAZEBO_ROS_HUMAN_RECEIVER_HH
#define GAZEBO_ROS_HUMAN_RECEIVER_HH

#include <map>

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/JointState.h>
#include <dynamic_reconfigure/server.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

//#include <transitbuddy_msgs/PoseWithIDArray.h>
//#include <transitbuddy_msgs/LineWithIDArray.h>
#include <tuw_object_msgs/ObjectWithCovarianceArray.h>
#include <std_msgs/String.h>

#include <tuw_gazebo_plugins/human_receiverConfig.h>

namespace gazebo
{
class Joint;
class Entity;

class GazeboRosHumanReceiver : public WorldPlugin
{
public:
  GazeboRosHumanReceiver();
  void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);

  /*
   * receive humans as Objects with Covariance, make lokal copy of msg (thread save)
   * @param msg received message
   */
  void humanCallback(const tuw_object_msgs::ObjectWithCovarianceArrayConstPtr &msg);
  void callbackParameters(gazebo_human_receiver::human_receiverConfig &config, uint32_t level);

private:
  double map_offset_x_;
  double map_offset_y_;
  double map_offset_angle_;
  int max_humans_;
  double min_distance_between_humans_;
  double inactive_placement_offset_;
  double radius_;
  double length_collision_;
  double length_visual_;
  
  tuw_object_msgs::ObjectWithCovarianceArray msgHumans_;
  
  physics::WorldPtr world_;
  boost::shared_ptr<ros::NodeHandle> rosnode_;
  boost::shared_ptr<ros::NodeHandle> n_param_;
  std::string robot_namespace_;
  ros::Subscriber subHumanObject_;
  ros::Subscriber subCommand_;
  dynamic_reconfigure::Server<gazebo_human_receiver::human_receiverConfig> reconfigureServer_;
  dynamic_reconfigure::Server<gazebo_human_receiver::human_receiverConfig>::CallbackType reconfigureFnc_;
  
  /*
   * create Cylinder as SDF model and insert into world
   * @param name model name
   * @param pose model pose
   */
  void createHuman(const std::string &name, const ignition::math::Pose3d &pose);
  
  boost::interprocess::interprocess_mutex mutexHumans_;
  std::vector<physics::ModelPtr> humansInactive_;
  std::map<int, physics::ModelPtr> humansActive_;

  boost::shared_ptr<boost::thread> createHumansThread_;
  boost::shared_ptr<boost::thread> updateHumansThread_;

  /*
   * Generate inactive human models
   */
  void createHumansFnc();
  
  /*
   * Create new active humans or update them according to received object msg
   * New active humans are actually moved from inactive since model creation is slow
   */
  void updateHumansFnc();

  physics::ModelPtr getHuman(int id);
  std::string idToName(int id);
  
};
}

#endif

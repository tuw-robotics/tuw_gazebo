#include <tuw_gazebo_plugins/gazebo_ros_human_receiver.h>
#include <tuw_gazebo_plugins/gazebo_model_templates.h>
#include <gazebo/plugins/ActorPlugin.hh>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

namespace gazebo
{
// Constructor
GazeboRosHumanReceiver::GazeboRosHumanReceiver() : reconfigureServer_(ros::NodeHandle("GazeboRosHumanReceiver"))
{
}

void GazeboRosHumanReceiver::callbackParameters(tuw_gazebo_plugins::GazeboRosHumanReceiverConfig &config, uint32_t level)
{
  map_offset_x_ = config.map_offset_x;
  map_offset_y_ = config.map_offset_y;
  map_offset_angle_ = config.map_offset_angle;
}

void GazeboRosHumanReceiver::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{
  this->world_ = _parent;
  rosnode_ = boost::shared_ptr<ros::NodeHandle>(new ros::NodeHandle());
  n_param_ = boost::shared_ptr<ros::NodeHandle>(new ros::NodeHandle("~"));

  this->max_humans_ = 100;
  if (!_sdf->HasElement("max_humans"))
  {
    ROS_WARN("GazeboRosHumanReceiver Plugin (ns = %s) missing <max_humans>, defaults to %i",
             this->robot_namespace_.c_str(), this->max_humans_);
  }
  else
  {
    this->max_humans_ = _sdf->GetElement("max_humans")->Get<int>();
  }
  
  this->min_distance_between_humans_ = 0.5;
  if (!_sdf->HasElement("min_distance_between_humans"))
  {
    ROS_WARN("GazeboRosHumanReceiver Plugin (ns = %s) missing <min_distance_between_humans>, defaults to %f",
             this->robot_namespace_.c_str(), this->min_distance_between_humans_);
  }
  else
  {
    this->min_distance_between_humans_ = _sdf->GetElement("min_distance_between_humans")->Get<double>();
  }
  
  this->inactive_placement_offset_ = 0.0;
  if (!_sdf->HasElement("inactive_placement_offset"))
  {
    ROS_WARN("GazeboRosHumanReceiver Plugin (ns = %s) missing <inactive_placement_offset>, defaults to %f",
             this->robot_namespace_.c_str(), this->inactive_placement_offset_);
  }
  else
  {
    this->inactive_placement_offset_ = _sdf->GetElement("inactive_placement_offset")->Get<double>();
  }
  
  this->radius_ = 0.2;
  if (!_sdf->HasElement("radius"))
  {
    ROS_WARN("GazeboRosHumanReceiver Plugin (ns = %s) missing <radius>, defaults to %f",
             this->robot_namespace_.c_str(), this->radius_);
  }
  else
  {
    this->radius_ = _sdf->GetElement("radius")->Get<double>();
  }
  
  this->length_collision_ = 0.4;
  if (!_sdf->HasElement("length_collision"))
  {
    ROS_WARN("GazeboRosHumanReceiver Plugin (ns = %s) missing <length_collision>, defaults to %f",
             this->robot_namespace_.c_str(), this->length_collision_);
  }
  else
  {
    this->length_collision_ = _sdf->GetElement("length_collision")->Get<double>();
  }
  
  this->length_visual_ = 1.8;
  if (!_sdf->HasElement("length_visual"))
  {
    ROS_WARN("GazeboRosHumanReceiver Plugin (ns = %s) missing <length_visual>, defaults to %f",
             this->robot_namespace_.c_str(), this->length_visual_);
  }
  else
  {
    this->length_visual_ = _sdf->GetElement("length_visual")->Get<double>();
  }

  ROS_INFO("GazeboRosHumanReceiver");
  subHumanObject_ = rosnode_->subscribe("human_objects", 1, &GazeboRosHumanReceiver::humanCallback, this);
  updateHumansThread_ = boost::shared_ptr<boost::thread>(new boost::thread(&GazeboRosHumanReceiver::updateHumansFnc, this));
  createHumansThread_ = boost::shared_ptr<boost::thread>(new boost::thread(&GazeboRosHumanReceiver::createHumansFnc, this));
  reconfigureFnc_ = boost::bind(&GazeboRosHumanReceiver::callbackParameters, this, _1, _2);
  reconfigureServer_.setCallback(reconfigureFnc_);
}

void GazeboRosHumanReceiver::humanCallback(const tuw_object_msgs::ObjectWithCovarianceArrayConstPtr &msg)
{
  if (mutexHumans_.try_lock())
  {
    msgHumans_ = *msg;
    mutexHumans_.unlock();
  }
}

void GazeboRosHumanReceiver::createHuman(const std::string &name, const ignition::math::Pose3d &pose)
{
  double mass = 10;
  //double radius = 0.2, length_viusal = 1.8, length_collision = 0.4, mass = 10;
  double half_length_visual = length_visual_ / 2.0;
  double half_length_collision = length_collision_ / 2.0;
//  std::string modelStr = GazeboModelTemplates::cylinderTemplate(name, pose, radius_, mass,
//                                                                length_visual_, half_length_collision);
  std::string modelStr = GazeboModelTemplates::personTemplate(name, pose);
  sdf::SDF sdfModel;
  sdfModel.SetFromString(modelStr);
  this->world_->InsertModelSDF(sdfModel);
}

void GazeboRosHumanReceiver::createHumansFnc()
{
  world_->SetPaused(true); // pause until humans are instantiated (ensures that mped does not start beforehand)
  boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(mutexHumans_);
  sleep(2);
  int cols = ceil(sqrt(max_humans_));
  for (std::size_t id = 0; id < max_humans_; id++)
  {
    int r = id / cols;
    int c = id % cols;
    ignition::math::Pose3d pose(min_distance_between_humans_ * r + inactive_placement_offset_, min_distance_between_humans_ * c + inactive_placement_offset_, 0, 0, 0, 0);
    std::string name = idToName(id);
    createHuman(name, pose);
  }
  ROS_INFO("create  %i humans:", max_humans_);
  sleep(5);
  humansInactive_.resize(max_humans_);
  for (std::size_t id = 0; id < humansInactive_.size(); id++)
  {
    std::string name = idToName(id);
    physics::ModelPtr p = this->world_->ModelByName(name);
    std::cout << "name " << name << std::endl;
    if (p)
    {
      humansInactive_[id] = p;
    }
    else
    {
      ROS_INFO("createHumansFnc: Could not get pointer to %s", name.c_str());
    }
  }
  ROS_INFO("indexed humans done");
  
  world_->SetPaused(false);
}

void GazeboRosHumanReceiver::updateHumansFnc()
{
  int seq = msgHumans_.header.seq;
  while (true)
  {
    usleep(100000);
    
    // only update if new msg arrived
    if (seq != msgHumans_.header.seq)
    {
      seq = msgHumans_.header.seq;
      std::vector<int> logAdded;
      std::vector<int> logMoved;
      std::vector<int> logRemoved;
      boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(mutexHumans_);
      ignition::math::Pose3d pose(0, 0, 0, 0, 0, 0);
      double ca = cos(map_offset_angle_), sa = sin(map_offset_angle_);
      physics::ModelPtr p;
      std::map<int, physics::ModelPtr>::iterator it;
      std::map<int, physics::ModelPtr> lasthumansActive = humansActive_;
      humansActive_.clear();
      for (std::size_t i = 0; i < msgHumans_.objects.size(); i++)
      {
        if(msgHumans_.objects[i].object.ids.empty())
        {
          ROS_WARN("object with no ids received");
          continue;
        }
        const geometry_msgs::Point &pos = msgHumans_.objects[i].object.pose.position;
        const geometry_msgs::Twist &twist = msgHumans_.objects[i].object.twist;
        const geometry_msgs::Quaternion &quat = msgHumans_.objects[i].object.pose.orientation;
        double xw = ca * pos.x - sa * pos.y + map_offset_x_;
        double yw = sa * pos.x + ca * pos.y + map_offset_y_;
        double zw = 0.0;
        xw = pos.x;
        yw = pos.y;
        zw = 0;
        
        double vx = twist.linear.x;
        double vy = twist.linear.y;
        double vz = 0.0;
        
        // M_PI_2 seems to be necessary for this particular human model
        ignition::math::Pose3d poseModel(xw, yw, zw, 0, 0, atan2(vy, vx) + M_PI_2);
        
        it = lasthumansActive.find(msgHumans_.objects[i].object.ids[0]);
        if (it == lasthumansActive.end())
        {
          if (humansInactive_.empty())
          {
            ROS_INFO("updateHumansFnc: out of humans");
            continue;
          }
          else
          {
            p = humansInactive_.back();
            humansInactive_.pop_back();
            std::cout << "set world pose " << std::endl;
            p->SetWorldPose(poseModel);
            p->SetStatic(false);
          }
        }
        else
        {
          p = it->second;
          lasthumansActive.erase(it);
        }
        humansActive_[msgHumans_.objects[i].object.ids[0]] = p;
        const ignition::math::Pose3d &current = p->WorldPose();
        ignition::math::Vector3d diff = poseModel.Rot().Euler() - current.Rot().Euler();
        p->SetLinearVel(ignition::math::Vector3d(vx, vy, vz));
        p->SetAngularVel(ignition::math::Vector3d(diff.X(), diff.Y(), diff.Z()));
        p->SetAngularAccel(ignition::math::Vector3d::Zero);
        p->SetWorldPose(poseModel);
      }
      int cols = ceil(sqrt(max_humans_));
      for (it = lasthumansActive.begin(); it != lasthumansActive.end(); ++it)
      {
        p = it->second;
        int id = humansInactive_.size();
        humansInactive_.push_back(p);
        int r = id / cols;
        int c = id % cols;
        ignition::math::Pose3d pose(min_distance_between_humans_ * r, min_distance_between_humans_ * c, 0, 0, 0, 0);
        p->SetStatic(true);
        p->SetAngularVel(ignition::math::Vector3d::Zero);
        p->SetAngularAccel(ignition::math::Vector3d::Zero);
        p->SetLinearVel(ignition::math::Vector3d::Zero);
        p->SetLinearAccel(ignition::math::Vector3d::Zero);
        p->SetWorldPose(pose);
      }

      ROS_INFO("GazeboRosHumanReceiver Plugin (ns = %s) active humans  %i", this->robot_namespace_.c_str(),
               (int)humansActive_.size());
    }
  }
}

physics::ModelPtr GazeboRosHumanReceiver::getHuman(int id)
{
  std::string name = idToName(id);
  return world_->ModelByName(name);
}

std::string GazeboRosHumanReceiver::idToName(int id)
{
  char text[0xF];
  sprintf(text, "h%03i", id);
  return text;
}

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(GazeboRosHumanReceiver)
}

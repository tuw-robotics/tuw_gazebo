/***************************************************************************
 *   Software License Agreement (BSD License)                              *
 *   Copyright (C) 2015 by Horatiu George Todoran <todorangrg@gmail.com>   *
 *                                                                         *
 *   Redistribution and use in source and binary forms, with or without    *
 *   modification, are permitted provided that the following conditions    *
 *   are met:                                                              *
 *                                                                         *
 *   1. Redistributions of source code must retain the above copyright     *
 *      notice, this list of conditions and the following disclaimer.      *
 *   2. Redistributions in binary form must reproduce the above copyright  *
 *      notice, this list of conditions and the following disclaimer in    *
 *      the documentation and/or other materials provided with the         *
 *      distribution.                                                      *
 *   3. Neither the name of the copyright holder nor the names of its      *
 *      contributors may be used to endorse or promote products derived    *
 *      from this software without specific prior written permission.      *
 *                                                                         *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS   *
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT     *
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS     *
 *   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE        *
 *   COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,  *
 *   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,  *
 *   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;      *
 *   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER      *
 *   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT    *
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY *
 *   WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           *
 *   POSSIBILITY OF SUCH DAMAGE.                                           *
 ***************************************************************************/

#ifndef GAZEBO_ROS_BRIDGE_MODEL_PLUGIN_H
#define GAZEBO_ROS_BRIDGE_MODEL_PLUGIN_H

#include <tuw_gazebo_plugins/gazebo_ros_utils.h>

#include <vector>
#include <array>
#include <functional>

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

// ROS
#include <ros/ros.h>
#include <ros/assert.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <tuw_nav_msgs/JointsIWS.h>
#include <tuw_gazebo_plugins/GazeboRosBridgeModelPluginConfig.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

// Utils
#include <gazebo/msgs/wrench.pb.h>
#include <gazebo/msgs/vector3d.pb.h>
#include <gazebo/transport/transport.hh>

namespace gazebo {

class GAZEBO_VISIBLE GazeboRosBridgeModelPlugin : public ModelPlugin {

    public   : GazeboRosBridgeModelPlugin();
    public   : virtual ~GazeboRosBridgeModelPlugin();
    
    ///@brief Loads the plugin (loads sdf parameters, publishes/subscribes ros topics etc. ).
    public   : virtual void Load (physics::ModelPtr _parent, sdf::ElementPtr _sdf) override;
    ///@brief Resets the plugin.
    public   : virtual void Reset() override;
    ///@brief Initializes the plugin.
    public   : virtual void Init () override;

    ///@brief Function called at each simulator cycle.
    protected: void UpdateChild();
    
    ///@brief Loads sdf-defined parameters.
    private  : void loadParameters(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    
    
    private  : void loadJoints          ();
    private  : void setJointsConstraints();
    private  : void initAllStates       (const nav_msgs::Odometry& body_state);
    
    
    private  : void setCmdMode          (size_t _jointsTypeIdx, const std::string& _mode) ;
    
    //Joints stuff
    private  : enum JointsTypes { REVOL, STEER, ENUM_SIZE };
    
    //Joints Constraints/Params
    private  : double constr_hi_stop_steering_;
    private  : double constr_lo_stop_steering_;
    private  : std::array<double, JointsTypes::ENUM_SIZE> constr_max_torque_;
    private  : std::array<double, JointsTypes::ENUM_SIZE> constr_max_vel_;
    private  : std::array<double, JointsTypes::ENUM_SIZE> constr_damping_;
    private  : std::array<double, JointsTypes::ENUM_SIZE> constr_friction_;
    
    
    private  : std::array<std::vector<physics::JointPtr>, JointsTypes::ENUM_SIZE> joints_;
    
    private  : std::array<std::function<  void(physics::JointPtr&, double)>, JointsTypes::ENUM_SIZE> setJointCmd;
    private  : std::array<std::function<double(physics::JointPtr&)        >, JointsTypes::ENUM_SIZE> getJointMeasure;
    
    private  : std::array<std::string, JointsTypes::ENUM_SIZE> jointsCmdType_;
    private  : std::array<std::string, JointsTypes::ENUM_SIZE> jointsMeasureType_;
    
    //Chassis state stuff
    private  : nav_msgs::Odometry body_state_initial_;
    private  : nav_msgs::Odometry body_state_;
    
    // Update Rate
    private  : double       update_rate_;
    private  : double       update_period_;
    private  : common::Time last_update_time_;
    
    // Flags
    private  : bool alive_;
    private  : bool publishTfs_;
    private  : bool publishJointStates_;
    private  : bool publishOdometry_;
    private  : bool firstChildUpdate_;
    
    //Publishers / subscribers
    private  : void jointsCmdCallback       ( const tuw_nav_msgs::JointsIWS::ConstPtr& joints_cmd_msg );
    private  : void publishJointMeasurements();
    private  : void publishJointsStates     ();
    private  : void publishTFs              ();
    private  : void publishWorldOdometry    ( double step_time );

    // GAZEBO STUFF
    private  : GazeboRosPtr         gazeboRos_;
    private  : physics::ModelPtr    parent_;
    private  : event::ConnectionPtr updateConnection_;

    // Custom ROS Callback Queue
    private  : void QueueThread();
    private  : ros::CallbackQueue queue_;
    private  : boost::thread callbackQueueThread_;
    
    // ROS STUFF
    private  : ros::Subscriber subJointsCmd_;
    private  : ros::Publisher  pubJointsStates_;
    private  : ros::Publisher  pubOdometry_;
    private  : ros::Publisher  pubJointMeasures_;
    
    private  : tuw_gazebo_plugins::GazeboRosBridgeModelPluginConfig config_;
    private  : std::shared_ptr<dynamic_reconfigure::Server<tuw_gazebo_plugins::GazeboRosBridgeModelPluginConfig> > reconfigureServer_; /// parameter server stuff
    private  : dynamic_reconfigure::Server<tuw_gazebo_plugins::GazeboRosBridgeModelPluginConfig>::CallbackType reconfigureFnc_;  /// parameter server stuff
    private  : void callbackConfig ( tuw_gazebo_plugins::GazeboRosBridgeModelPluginConfig &config, uint32_t level ); /// callback function on incoming parameter changes
    
    private  : boost::shared_ptr<tf::TransformBroadcaster> transform_broadcaster_;
    private  : sensor_msgs::JointState  joint_state_;
    private  : tuw_nav_msgs::JointsIWS  jointsCmd_;
    
    //topic, namespaces & joint/link names
    private  : std::string robot_namespace_   ;
    private  : std::string joints_cmd_topic_  ;
    private  : std::string joints_measures_topic_;
    private  : std::string joint_states_topic_;
    private  : std::string odometry_topic_  ;
    private  : std::string odometry_frame_  ;
    private  : std::string robot_base_frame_;
};

}

#endif // GAZEBO_ROS_BRIDGE_MODEL_PLUGIN_H


/*
 * Copyright (c) 2010
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the <organization> nor the
 *      names of its contributors may be used to endorse or promote products
 *      derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY Antons Rebguns <email> ''AS IS'' AND ANY
 *  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL Antons Rebguns <email> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **/


#ifndef DOOR_PLUGIN_HH
#define DOOR_PLUGIN_HH

#include <map>

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <tuw_gazebo_plugins/gazebo_ros_utils.h>

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/JointState.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

namespace gazebo {

  class Joint;
  class Entity;

  class GazeboDoorPlugin : public ModelPlugin {

    public:
      GazeboDoorPlugin();
      ~GazeboDoorPlugin();
      void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) override;
      void Reset() override;
      void Init() override;

    protected:
      void UpdateChild();
      void FiniChild();
      void QueueThread();

      GazeboRosPtr gazebo_ros_;
      physics::ModelPtr parent_;
      event::ConnectionPtr update_connection_;

      physics::JointPtr joint_;

      // ROS STUFF
      ros::Subscriber cmd_vel_subscriber_;
      sensor_msgs::JointState joint_state_; 

      private: ros::CallbackQueue ros_cmd_queue_;
      private: boost::thread ros_cmd_thread_;

      std::string namespace_;
      std::string command_topic_;
      
      float value_ = 0;
      bool alive_ = true;
      bool update_ = true;
      float maxAngle_ = 0;
      
      // DiffDrive stuff
      void cmdVelCallback(const std_msgs::Float32::ConstPtr& cmd_msg);

  };

}

#endif


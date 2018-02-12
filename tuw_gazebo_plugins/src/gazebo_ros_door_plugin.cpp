/*
    Copyright (c) 2010, Daniel Hewlett, Antons Rebguns
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
        * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
        * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
        * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY Antons Rebguns <email> ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL Antons Rebguns <email> BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

/*
 * \file  gazebo_ros_diff_drive.cpp
 *
 * \brief A differential drive plugin for gazebo. Based on the diffdrive plugin
 * developed for the erratic robot (see copyright notice above). The original
 * plugin can be found in the ROS package gazebo_erratic_plugins.
 *
 * \author  Piyush Khandelwal (piyushk@gmail.com)
 *
 * $ Id: 06/21/2013 11:23:40 AM piyushk $
 */


/*
 *
 * The support of acceleration limit was added by
 * \author   George Todoran <todorangrg@gmail.com>
 * \author   Markus Bader <markus.bader@tuwien.ac.at>
 * \date 22th of May 2014
 */

#include <algorithm>
#include <assert.h>

#include <tuw_gazebo_plugins/gazebo_ros_door_plugin.h>

#include <gazebo/math/gzmath.hh>
#include <sdf/sdf.hh>

#include <ros/ros.h>

namespace gazebo {

GazeboDoorPlugin::GazeboDoorPlugin() {
}


// Destructor
GazeboDoorPlugin::~GazeboDoorPlugin() {
  /*
  odometry_publisher_.shutdown();
  joint_state_publisher_.shutdown();
  cmd_vel_subscriber_.shutdown();

  queue_.clear();
  queue_.disable();
  alive_ = false;
  callback_queue_thread_.join();
  */
}

// Load the controller
void GazeboDoorPlugin::Load (physics::ModelPtr _parent, sdf::ElementPtr _sdf) {

  parent_ = _parent;
  gazebo_ros_ = GazeboRosPtr (new GazeboRos (_parent, _sdf, "TUWDoorCtrl"));
  // Make sure the ROS node for Gazebo has already been initialized
  gazebo_ros_->isInitialized();

  gazebo_ros_->getParameter<std::string> (command_topic_, "commandTopic", "cmd_vel");

  if (_sdf->HasElement ("topic")) {
    command_topic_ = _sdf->GetElement ("topic")->GetValue()->GetAsString();
  }


  std::string jointStr = "door_joint";
  if (_sdf->HasElement ("joint")) {
    jointStr = _sdf->GetElement ("joint")->GetValue()->GetAsString();
  }
  joint_ = gazebo_ros_->getJoint (parent_, "doorJoint", jointStr);
  ROS_INFO ("%s: added door joint!", gazebo_ros_->info());

  ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float32> (command_topic_, 1, boost::bind (&GazeboDoorPlugin::cmdVelCallback, this, _1), ros::VoidPtr(), &ros_cmd_queue_);
  cmd_vel_subscriber_ = gazebo_ros_->node()->subscribe (so);
  ROS_INFO ("%s: Subscribed to %s!", gazebo_ros_->info(), command_topic_.c_str());

  std::string max_val = "3.1416";
  maxAngle_ = 3.1416;
  if (_sdf->HasElement ("angle_max")) {
    max_val = _sdf->GetElement ("angle_max")->GetValue()->GetAsString();
    try {
      maxAngle_ = std::stof (max_val);
    } catch (...) {
    }
  }

  std::string value = "0";
  if (_sdf->HasElement ("value")) {
    value = _sdf->GetElement ("value")->GetValue()->GetAsString();
    try {
      value_ = std::stof (value);
    } catch (...) {
    }
  }

  alive_ = true;
  // start custom queue for diff drive
  this->ros_cmd_thread_ = boost::thread (boost::bind (&GazeboDoorPlugin::QueueThread, this));


  // listen to the update event (broadcast every simulation iteration)
  this->update_connection_ =
    event::Events::ConnectWorldUpdateBegin (boost::bind (&GazeboDoorPlugin::UpdateChild, this));
}

void GazeboDoorPlugin::QueueThread() {
  static const double timeout = 0.01;

  while (alive_ && gazebo_ros_->node()->ok()) {
    ros_cmd_queue_.callAvailable (ros::WallDuration (timeout));
  }
}


void GazeboDoorPlugin::Init() {
  gazebo::ModelPlugin::Init();
}


void GazeboDoorPlugin::Reset() {
  gazebo::ModelPlugin::Reset();
  joint_->SetParam ("fmax", 0, 100.0);
  joint_->SetParam ("vel", 0, 1.0);

}

// Update the controller
void GazeboDoorPlugin::UpdateChild() {
  float angle = value_ * maxAngle_;
  joint_->SetPosition (0, angle);
}

// Finalize the controller
void GazeboDoorPlugin::FiniChild() {
  alive_ = false;
  ros_cmd_queue_.clear();
  ros_cmd_queue_.disable();
  gazebo_ros_->node()->shutdown();
  ros_cmd_thread_.join();
}

void GazeboDoorPlugin::cmdVelCallback (const std_msgs::Float32::ConstPtr& cmd_msg) {
  if (cmd_msg->data > 1.0)
    value_ = 1.0;
  else if (cmd_msg->data < 0.0)
    value_ = 0.0;
  else
    value_ = cmd_msg->data;
}


GZ_REGISTER_MODEL_PLUGIN (GazeboDoorPlugin)
}


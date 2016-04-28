/***************************************************************************
 *   Software License Agreement (BSD License)                              *
 *   Copyright (C) 2016 by Horatiu George Todoran <todorangrg@gmail.com>   *
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


#ifndef TUW_GAZEBO_ROS_IMU9DOF_H
#define TUW_GAZEBO_ROS_IMU9DOF_H

#include <string>

#include <thread>
#include <functional>

#include <ros/ros.h>
#include <ros/advertise_options.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <tuw_gazebo_plugins/PubQueue.h>
#include <tuw_gazebo_plugins/gazebo_ros_utils.h>
#include <tuw_gazebo_plugins/noise_sim_model.h>

namespace gazebo
{
  class GazeboRosImu9Dof : public ModelPlugin
  {
    private: enum Coord { X, Y, Z };
    
    /// \brief Constructor
    public: GazeboRosImu9Dof();

    /// \brief Destructor
    public: ~GazeboRosImu9Dof();

    /// \brief Load the plugin
    /// \param take in SDF root element
    public: virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    public: virtual void Init();
    public: virtual void Reset();
    
    private: void Update();
    
    private: void publishImu( common::Time& _current_time );
    private: void publishMag( common::Time& _current_time );
    
    /// \brief connection to WorldUpdateEnd Gazebo event
    event::ConnectionPtr update_connection_;
    /// \brief Gazebo model pointer
    physics::ModelPtr model_;
    /// \brief object performing gazebo <--> ROS manipulation
    private: GazeboRosPtr gazebo_ros_;
    
    /// \brief publisher multi-queue (prevents blocking)
    private: PubMultiQueue pub_multi_queue;
    
    /// \brief publisher custom queue
    private: PubQueue<sensor_msgs::Imu>          ::Ptr pub_queue_imu_;
    private: PubQueue<sensor_msgs::MagneticField>::Ptr pub_queue_mag_;
    
    /// \brief ROS lase sensor publisher
    private: ros::Publisher ros_pub_imu_;
    private: ros::Publisher ros_pub_mag_;
    
    
    
    //SDF Params:
    /// \brief imu usage flag
    private: bool use_imu_;
    /// \brief magnetometer usage flag
    private: bool use_mag_;
    /// \brief imu ROS topic name
    private: std::string topic_name_imu_;
    /// \brief magnetometer ROS topic name
    private: std::string topic_name_mag_;
    /// \brief sensor link name, should match a gazebo link
    private: std::string link_name_;
    /// \brief controller update period
    private: double update_period_;
    /// \brief previous iteration controller update time
    private: common::Time last_update_time_;
    /// \brief offset from model base_link to sensor link
    private: math::Pose offset_;
    /// \brief sensor link name
    private: std::string sensor_link_name_;
    
    private: math::Vector3    model_velocity_;
    private: math::Quaternion sns_orient_;
    private: math::Vector3    imu_acc_lin_;
    private: math::Vector3    imu_vel_ang_;
    private: math::Vector3    mag_fld_lin_;
    
    /// \brief imu linear accelerations noise models
    private: tuw::NoiseSimModel noise_imu_acc_lin_[3];
    /// \brief imu angular velocity noise models
    private: tuw::NoiseSimModel noise_imu_vel_ang_[3];
    /// \brief magnetometer magnetic field noise models
    private: tuw::NoiseSimModel noise_mag_fld_lin_[3];
  };
}
#endif //TUW_GAZEBO_ROS_IMU9DOF_H

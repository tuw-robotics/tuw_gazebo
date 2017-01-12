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


#ifndef TUW_GAZEBO_ROS_LASER_H
#define TUW_GAZEBO_ROS_LASER_H

#include <string>

#include <thread>
#include <functional>

#include <ros/ros.h>
#include <ros/advertise_options.h>
#include <sensor_msgs/LaserScan.h>

#include <gazebo/common/Plugin.hh>
#include <tuw_gazebo_plugins/gazebo_ros_utils.h>
#include <tuw_gazebo_plugins/PubQueue.h>

namespace gazebo
{
  class GazeboRosLaser : public SensorPlugin
  {
    /// \brief Constructor
    public: GazeboRosLaser();

    /// \brief Destructor
    public: ~GazeboRosLaser();

    /// \brief Load the plugin
    /// \param take in SDF root element
    public: virtual void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    /// \brief The parent sensor
    private: sensors::RaySensorPtr parent_ray_sensor_;
    
    
    
    /// \brief connection to the gazebo laser object
    private: event::ConnectionPtr laser_connection_;
    
    /// \brief Keep track of number of connctions
    private: int laser_connect_count_;
    
    /// \brief Fuction called when a ROS topic subscribes to the laser publisher
    private: void LaserConnect();
    
    /// \brief Fuction called when a ROS topic unsubscribes to the laser publisher
    private: void LaserDisconnect();
    
    /// \brief Function called when gazebo Ray
    private: void OnScan(sensors::RaySensorPtr &_ray);

    
    
    /// \brief object performing gazebo <--> ROS manipulation
    private: GazeboRosPtr gazebo_ros_;
    
    /// \brief publisher multi-queue (prevents blocking)
    private: PubMultiQueue pub_multi_queue;
    
    /// \brief publisher custom queue
    private: PubQueue<sensor_msgs::LaserScan>::Ptr pub_queue_;
    
    /// \brief ROS lase sensor publisher
    private: std::vector<ros::Publisher> ros_pub_laser_;
    
    
    
    //SDF Params:
    /// \brief topic name
    private: std::vector<std::string> topic_name_;
    
    /// \brief frame transform name, should match link name
    private: std::vector<std::string> frame_name_;
    
    /// \brief tf prefix
    private: std::string tf_prefix_;
    
    /// \brief minimum sensor range ( whatever gazebo senses under this value is set to inf )
    private: double snsRangeMin_;
  };
}
#endif //TUW_GAZEBO_ROS_LASER_H

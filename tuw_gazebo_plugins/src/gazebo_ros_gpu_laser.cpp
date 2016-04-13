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

#include <algorithm>
#include <string>

#include <gazebo/sensors/GpuRaySensor.hh>

#include <gazebo_plugins/gazebo_ros_gpu_laser.h>

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN ( GazeboRosGpuLaser )

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosGpuLaser::GazeboRosGpuLaser() { 
    
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosGpuLaser::~GazeboRosGpuLaser() {
  parent_ray_sensor_->DisconnectUpdated(laser_connection_);
  pub_queue_.reset();
  ros_pub_laser_.shutdown();
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosGpuLaser::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf) {

    parent_ray_sensor_ = std::dynamic_pointer_cast<sensors::GpuRaySensor> ( _parent );
    if ( !parent_ray_sensor_ ) { gzthrow ( "GazeboRosGpuLaser controller requires a GpuRay Sensor as its parent" ); }
    parent_ray_sensor_->SetActive ( false );// sensor data modification during a cycle set to false

    gazebo_ros_ = GazeboRosPtr ( new GazeboRos ( _parent, _sdf, "TUWLaser" ) );
    gazebo_ros_->isInitialized(); // Make sure the ROS node for Gazebo has already been initialized
    gazebo_ros_->getParameter<std::string> ( frame_name_, "frameName", "front_laser" );
    gazebo_ros_->getParameter<std::string> ( topic_name_, "topicName", "front_laser/laser" );
    gazebo_ros_->getParameter<double>      ( snsRangeMin_, "rangeMin" , 0.2 );

    using rosAO = ros::AdvertiseOptions;
    rosAO ao = rosAO::create<sensor_msgs::LaserScan> ( topic_name_, 1, std::bind ( &GazeboRosGpuLaser::LaserConnect   , this ),
                                                                       std::bind ( &GazeboRosGpuLaser::LaserDisconnect, this ), ros::VoidPtr(), NULL );
    ros_pub_laser_ = gazebo_ros_->node()->advertise ( ao );

    pub_multi_queue.startServiceThread();
    pub_queue_ = pub_multi_queue.addPub<sensor_msgs::LaserScan>();
    
    laser_connect_count_ = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboRosGpuLaser::LaserConnect() {
    if ( ++laser_connect_count_ == 1 ) { laser_connection_ = parent_ray_sensor_->ConnectUpdated(std::bind( &GazeboRosGpuLaser::OnScan, this, parent_ray_sensor_ ) ); }
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboRosGpuLaser::LaserDisconnect() {
    if ( --laser_connect_count_ == 0 ) { parent_ray_sensor_->DisconnectUpdated(laser_connection_); }
}

////////////////////////////////////////////////////////////////////////////////
// Convert new Gazebo message to ROS message and publish it
void GazeboRosGpuLaser::OnScan(sensors::GpuRaySensorPtr &_ray) {
    // We got a new message from the Gazebo sensor.  Stuff a
    // corresponding ROS message and publish it.
    sensor_msgs::LaserScan laser_msg;
    laser_msg.header.stamp    = ros::Time ( _ray->LastUpdateTime().sec, _ray->LastUpdateTime().nsec );
    laser_msg.header.frame_id = gazebo_ros_->getNamespace() + frame_name_;
    laser_msg.angle_min       = _ray->AngleMin().Radian();
    laser_msg.angle_max       = _ray->AngleMax().Radian();
    laser_msg.angle_increment = _ray->AngleResolution();
    laser_msg.time_increment  = 0;  // instantaneous simulator scan
    laser_msg.scan_time       = 1./_ray->UpdateRate();
    laser_msg.range_min       = _ray->RangeMin();
    laser_msg.range_max       = _ray->RangeMax();
    laser_msg.ranges.     resize ( _ray->RangeCount() );
    laser_msg.intensities.resize ( _ray->RangeCount() );
    for( std::size_t i = 0; i < _ray->RangeCount(); ++i ) { 
	if( _ray->Range(i) >= snsRangeMin_ ) { laser_msg.ranges[i] = _ray->Range(i); laser_msg.intensities[i] = _ray->Retro(i); }
	else                                 { laser_msg.ranges[i] = 1./0./* inf */; laser_msg.intensities[i] = 0;              }
    }
    
    pub_queue_->push ( laser_msg, ros_pub_laser_ );
}

}

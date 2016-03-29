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
#include <float.h>

#include <gazebo/sensors/ImuSensor.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <gazebo/sensors/MagnetometerSensor.hh>
#include <gazebo/sensors/Noise.hh>

#include <gazebo_plugins/gazebo_ros_imu9dof.h>


#include <iostream>

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN ( GazeboRosImu9Dof )

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosImu9Dof::GazeboRosImu9Dof() { 
    
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosImu9Dof::~GazeboRosImu9Dof() {
  pub_queue_imu_.reset();
  pub_queue_mag_.reset();
  ros_pub_imu_.shutdown();
  ros_pub_mag_.shutdown();
  ROS_DEBUG("Destructed TUWImu9Dof Plugin");
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosImu9Dof::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    model_ = _model;
    gazebo_ros_ = GazeboRosPtr ( new GazeboRos ( _model, _sdf, "TUWImu9Dof" ) );
    gazebo_ros_->isInitialized(); // Make sure the ROS node for Gazebo has already been initialized
    
    pub_multi_queue.startServiceThread();
    
    if ( use_imu_ = _sdf->HasElement ( "imu" ) ) {
	ROS_INFO ( "%s: using IMU data", gazebo_ros_->info() );
        gazebo_ros_->Sdf() = _sdf->GetElement ( "imu" );
	
        if ( gazebo_ros_->Sdf()->HasElement ( "linear_acceleration" ) ) {
	    bool foundNoiseSdf = false;
	    for( size_t i = 0; i < 3; i++ ) { foundNoiseSdf = noise_imu_acc_lin_[i].loadParam(gazebo_ros_->Sdf()->GetElement ( "linear_acceleration" ) );  }
	    if( !foundNoiseSdf ) { ROS_WARN ( "%s: <linear_acceleration>: missing <noise type=\"tuw_advanced\">, defaults to no noise", gazebo_ros_->info() ); }
        }
        if ( gazebo_ros_->Sdf()->HasElement ( "angular_velocity" ) ) {
            bool foundNoiseSdf = false;
	    for( size_t i = 0; i < 3; i++ ) { foundNoiseSdf = noise_imu_vel_ang_[i].loadParam(gazebo_ros_->Sdf()->GetElement ( "angular_velocity" ) );  }
	    if( !foundNoiseSdf ) { ROS_WARN ( "%s: <angular_velocity>: missing <noise type=\"tuw_advanced\"> defaults to no noise", gazebo_ros_->info() ); }
        }
        gazebo_ros_->getParameter<std::string> ( topic_name_imu_, "topic", "imu" );
	ros_pub_imu_ = gazebo_ros_->node()->advertise<sensor_msgs::Imu> ( topic_name_imu_, 1 );
	pub_queue_imu_ = pub_multi_queue.addPub<sensor_msgs::Imu>();
    }
    if ( use_mag_ = _sdf->HasElement ( "magnetometer" ) ) {
	ROS_INFO ( "%s: using Magnetometer data", gazebo_ros_->info() );
	gazebo_ros_->Sdf() = _sdf->GetElement ( "magnetometer" );
	
        if ( gazebo_ros_->Sdf()->HasElement ( "magnetic_field" ) ) {
            bool foundNoiseSdf = false;
	    for( size_t i = 0; i < 3; i++ ) { foundNoiseSdf = noise_mag_fld_lin_[i].loadParam(gazebo_ros_->Sdf()->GetElement ( "magnetic_field" ) );  }
	    if( !foundNoiseSdf ) { ROS_WARN ( "%s: <magnetic_field>: missing <noise type=\"tuw_advanced\"> defaults to no noise", gazebo_ros_->info() ); }
        }
	gazebo_ros_->getParameter<std::string> ( topic_name_mag_, "topic", "mag" );
	ros_pub_mag_ = gazebo_ros_->node()->advertise<sensor_msgs::MagneticField> ( topic_name_mag_, 1 );
	pub_queue_mag_ = pub_multi_queue.addPub<sensor_msgs::MagneticField>();
    }
    gazebo_ros_->Sdf() = gazebo_ros_->baseSdf();
    
    offset_ = math::Pose();
    if( _sdf->HasElement("origin") ) {
	offset_.pos = _sdf->GetElement("origin")->Get<math::Vector3>( "xyz" );
	offset_.rot = math::Quaternion(_sdf->GetElement("origin")->Get<math::Vector3>( "rpy" ) );
    } else { ROS_WARN ( "%s: missing <%s> default is %s", gazebo_ros_->info(), "origin", "<0 0 0 , 0 0 0>" ); }
    
    gazebo_ros_->getParameter<double> ( update_period_   , "updateRate", 100.0 ); update_period_ = 1./update_period_;
    gazebo_ros_->getParameter<std::string> ( sensor_link_name_, "link_name", "imu9dof");
    
    // listen to the update event (broadcast every simulation iteration)
    update_connection_ = event::Events::ConnectWorldUpdateEnd( boost::bind ( &GazeboRosImu9Dof::Update, this ) );
}

void GazeboRosImu9Dof::Init() {
    gazebo::ModelPlugin::Init();
}

void GazeboRosImu9Dof::Reset() {
    gazebo::ModelPlugin::Reset();
}


void GazeboRosImu9Dof::Update() {
    common::Time current_time = model_->GetWorld()->GetSimTime();
    double dt = ( current_time - last_update_time_ ).Double();
    if ( dt < update_period_ ) { return; }
    
    math::Vector3 gravity            = model_->GetWorld()->GetPhysicsEngine()->GetGravity();
    math::Pose    model_pose         = model_->GetWorldPose();
    math::Vector3 model_velocity_new = model_->GetWorldLinearVel();
     // get velocity in world frame
    
    math::Quaternion sns_orient_new = offset_.rot * model_pose.rot; sns_orient_new.Normalize();
    
    if ( (use_imu_)&&(dt > 0.0) ) { 
	math::Quaternion delta_orient       = sns_orient_.GetInverse() * sns_orient_new;
	imu_acc_lin_ = offset_.rot.RotateVectorReverse((model_velocity_new - model_velocity_) / update_period_/* - gravity*/);
	imu_vel_ang_ = 2.0 * acos(std::max(std::min(delta_orient.w, 1.0), -1.0)) * math::Vector3(delta_orient.x, delta_orient.y, delta_orient.z).Normalize() / update_period_;
	
	imu_acc_lin_.x = noise_imu_acc_lin_[X].sim( imu_acc_lin_.x, dt );
	imu_acc_lin_.y = noise_imu_acc_lin_[Y].sim( imu_acc_lin_.y, dt );
	imu_acc_lin_.z = noise_imu_acc_lin_[Z].sim( imu_acc_lin_.z, dt );
	
	imu_vel_ang_.x = noise_imu_vel_ang_[X].sim( imu_vel_ang_.x, dt );
	imu_vel_ang_.y = noise_imu_vel_ang_[Y].sim( imu_vel_ang_.y, dt );
	imu_vel_ang_.z = noise_imu_vel_ang_[Z].sim( imu_vel_ang_.z, dt );
	publishImu( current_time );
    }
    if( (use_mag_) ) {
	mag_fld_lin_ = sns_orient_new.RotateVectorReverse( model_->GetWorld()->MagneticField() );
	
	mag_fld_lin_.x = noise_mag_fld_lin_[X].sim( mag_fld_lin_.x, dt );
	mag_fld_lin_.y = noise_mag_fld_lin_[X].sim( mag_fld_lin_.y, dt );
	mag_fld_lin_.z = noise_mag_fld_lin_[X].sim( mag_fld_lin_.z, dt );
	publishMag( current_time );
    }
    model_velocity_   = model_velocity_new;
    sns_orient_       = sns_orient_new;    
    last_update_time_ = current_time;
}

void GazeboRosImu9Dof::publishImu( common::Time& _current_time ) {
    sensor_msgs::Imu imu_msg;
    imu_msg.header.stamp    = ros::Time ( _current_time.sec, _current_time.nsec );
    imu_msg.header.frame_id = gazebo_ros_->getNamespace() + sensor_link_name_;
    imu_msg.linear_acceleration.x = imu_acc_lin_.x;
    imu_msg.linear_acceleration.y = imu_acc_lin_.y;
    imu_msg.linear_acceleration.z = imu_acc_lin_.z;
    imu_msg.angular_velocity.x    = imu_vel_ang_.x;
    imu_msg.angular_velocity.y    = imu_vel_ang_.y;
    imu_msg.angular_velocity.z    = imu_vel_ang_.z;
    imu_msg.orientation.w         = sns_orient_.w;
    imu_msg.orientation.x         = sns_orient_.x;
    imu_msg.orientation.y         = sns_orient_.y;
    imu_msg.orientation.z         = sns_orient_.z;
    
    for(size_t i = 0; i < 9; i++) { imu_msg.linear_acceleration_covariance[i] = 0; }
    imu_msg.linear_acceleration_covariance[0] = noise_imu_acc_lin_[X].sigmaWhiteNoise();
    imu_msg.linear_acceleration_covariance[4] = noise_imu_acc_lin_[Y].sigmaWhiteNoise();
    imu_msg.linear_acceleration_covariance[8] = noise_imu_acc_lin_[Z].sigmaWhiteNoise();
    
    for(size_t i = 0; i < 9; i++) { imu_msg.angular_velocity_covariance[i] = 0; }
    imu_msg.angular_velocity_covariance[0] = noise_imu_vel_ang_[X].sigmaWhiteNoise();
    imu_msg.angular_velocity_covariance[4] = noise_imu_vel_ang_[Y].sigmaWhiteNoise();
    imu_msg.angular_velocity_covariance[8] = noise_imu_vel_ang_[Z].sigmaWhiteNoise();
    
    pub_queue_imu_->push ( imu_msg, ros_pub_imu_ );
}

void GazeboRosImu9Dof::publishMag ( common::Time& _current_time ) {
    sensor_msgs::MagneticField mag_msg;
    mag_msg.header.stamp    = ros::Time ( _current_time.sec, _current_time.nsec );
    mag_msg.header.frame_id = gazebo_ros_->getNamespace() + sensor_link_name_;
    mag_msg.magnetic_field.x = mag_fld_lin_.x;
    mag_msg.magnetic_field.y = mag_fld_lin_.y;
    mag_msg.magnetic_field.z = mag_fld_lin_.z;
    
    pub_queue_mag_->push ( mag_msg, ros_pub_mag_ );
}

}

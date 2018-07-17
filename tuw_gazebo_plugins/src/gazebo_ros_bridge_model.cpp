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

#include <algorithm>
#include <assert.h>

#include <tuw_gazebo_plugins/gazebo_ros_bridge_model.h>

#include <sdf/sdf.hh>
#include <boost/graph/graph_concepts.hpp>

#include <ros/ros.h>

using namespace std;
// using namespace tuw;

namespace gazebo {

GazeboRosBridgeModelPlugin::GazeboRosBridgeModelPlugin():firstChildUpdate_(false) {}

// Destructor
GazeboRosBridgeModelPlugin::~GazeboRosBridgeModelPlugin() {
    
    //event::Events::DisconnectWorldUpdateBegin ( updateConnection_ );			//DEPRECATED
    
    pubJointMeasures_.shutdown();
    pubJointsStates_.shutdown();
    pubOdometry_    .shutdown();
    subJointsCmd_   .shutdown();
    
    queue_.clear();
    queue_.disable();
    alive_ = false;
    callbackQueueThread_.join();
}

// Load the controller
void GazeboRosBridgeModelPlugin::Load ( physics::ModelPtr _parent, sdf::ElementPtr _sdf ) {
    
    loadParameters      ( _parent,_sdf );
    loadJoints          ();
    setJointsConstraints();
    
    ///@todo create proper odom message that can init the ICC as well
    body_state_initial_.pose.pose.position.x  = 0; body_state_initial_.pose.pose.position.y  = 0; body_state_initial_.pose.pose.position.z  = 0;
    body_state_initial_.twist.twist.linear.x  = 0; body_state_initial_.twist.twist.linear.y  = 0; body_state_initial_.twist.twist.linear.z  = 0;
    body_state_initial_.twist.twist.angular.x = 0; body_state_initial_.twist.twist.angular.y = 0; body_state_initial_.twist.twist.angular.z = 0;
    initAllStates(body_state_initial_); 
    
    // Initialize update rate stuff
    if ( update_rate_ > 0.0 ) { update_period_ = 1.0 / update_rate_; }
    else                      { update_period_ = 0.0;                }
    last_update_time_ = parent_->GetWorld()->SimTime();

    //Publishers
    pubJointMeasures_ = gazeboRos_->node()->advertise<tuw_nav_msgs::JointsIWS>(joints_measures_topic_.c_str(), 1);
    ROS_INFO("%s: Advertise joint_measures!", gazeboRos_->info());
    
    
    if (publishJointStates_) {
	pubJointsStates_ = gazeboRos_->node()->advertise<sensor_msgs::JointState>(joint_states_topic_.c_str(), 1);
	ROS_INFO("%s: Advertise joint_states!", gazeboRos_->info());
    }
    if ( publishOdometry_ ) {
	pubOdometry_ = gazeboRos_->node()->advertise<nav_msgs::Odometry>(odometry_topic_, 1);
	ROS_INFO("%s: Advertise ground-truth odometry on %s !", gazeboRos_->info(), odometry_topic_.c_str());
    }
    
    transform_broadcaster_ = boost::shared_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster());
    if (publishTfs_) {
	ROS_INFO("%s: Advertise tf-s!", gazeboRos_->info() );
    }
    
    //Subscribers
    queue_.enable();
    
    ROS_INFO("%s: Try to subscribe to %s!", gazeboRos_->info(), joints_cmd_topic_.c_str());
    ros::SubscribeOptions so;
    so = ros::SubscribeOptions::create<tuw_nav_msgs::JointsIWS>(joints_cmd_topic_, 1, boost::bind(&GazeboRosBridgeModelPlugin::jointsCmdCallback, this, _1), ros::VoidPtr(), &queue_);
    so.transport_hints = ros::TransportHints().reliable().tcpNoDelay(true);
    subJointsCmd_ = gazeboRos_->node()->subscribe(so);
    ROS_INFO("%s: Subscribe to %s!", gazeboRos_->info(), joints_cmd_topic_.c_str());
    
    // listen to the update event (broadcast every simulation iteration)
    updateConnection_ = event::Events::ConnectWorldUpdateBegin ( boost::bind ( &GazeboRosBridgeModelPlugin::UpdateChild, this ) );
    
    //reconfigure stuff
    reconfigureServer_ = std::make_shared< dynamic_reconfigure::Server<tuw_gazebo_plugins::GazeboRosBridgeModelPluginConfig> > ( ros::NodeHandle ( "~/" + gazeboRos_->getPluginName()) );
    reconfigureFnc_    = boost::bind ( &GazeboRosBridgeModelPlugin::callbackConfig, this,  _1, _2 );
    reconfigureServer_->setCallback ( reconfigureFnc_ );
    
    // start custom queue
    alive_ = true;
    callbackQueueThread_ = boost::thread ( boost::bind ( &GazeboRosBridgeModelPlugin::QueueThread     , this ) );
}

void GazeboRosBridgeModelPlugin::loadParameters(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
    parent_ = _parent;
    gazeboRos_ = GazeboRosPtr ( new GazeboRos ( _parent, _sdf, "TUWBridgeModel", false ) );
    // Make sure the ROS node for Gazebo has already been initialized
    gazeboRos_->isInitialized();
    
    gazeboRos_->getParameter         (       joint_states_topic_,       "joints_states_topic", std::string(     "joint_states") );
    gazeboRos_->getParameter         (    joints_measures_topic_,     "joints_measures_topic", std::string(  "joints_measures") );
    gazeboRos_->getParameter         (         joints_cmd_topic_,          "joints_cmd_topic", std::string(      "joints_cmds") );
    gazeboRos_->getParameter         (           odometry_topic_,                "odom_topic", std::string("odom_ground_truth") );
    gazeboRos_->getParameter         (           odometry_frame_,                "odom_frame", std::string("odom_ground_truth") );
    gazeboRos_->getParameter         (         robot_base_frame_,          "model_base_frame", std::string(   "base_footprint") );
    gazeboRos_->getParameterBoolean  (               publishTfs_,               "publish_tfs",                            false );
    gazeboRos_->getParameterBoolean  (       publishJointStates_,     "publish_joints_states",                            false );
    gazeboRos_->getParameterBoolean  (          publishOdometry_, "publish_odom_ground_truth",                             true );
    
    gazeboRos_->getParameter<double> ( constr_hi_stop_steering_ ,       "steering_hi_stop"   ,                            M_PI  );
    gazeboRos_->getParameter<double> ( constr_lo_stop_steering_ ,       "steering_lo_stop"   ,                          - M_PI  );
    gazeboRos_->getParameter<double> ( constr_max_torque_[STEER],       "steering_max_torque",                            10.0  );
    gazeboRos_->getParameter<double> ( constr_max_vel_   [STEER],       "steering_max_vel"   ,                            10.0  );
    gazeboRos_->getParameter<double> ( constr_damping_   [STEER],       "steering_damping"   ,                             1.0  );
    gazeboRos_->getParameter<double> ( constr_friction_  [STEER],       "steering_friction"  ,                             0.0  );
    
    gazeboRos_->getParameter<double> ( constr_max_torque_[REVOL],       "revolute_max_torque",                            10.0  );
    gazeboRos_->getParameter<double> ( constr_max_vel_   [REVOL],       "revolute_max_vel"   ,                            10.0  );
    gazeboRos_->getParameter<double> ( constr_damping_   [REVOL],       "revolute_damping"   ,                             1.0  );
    gazeboRos_->getParameter<double> ( constr_friction_  [REVOL],       "revolute_friction"  ,                             0.0  );
    
    gazeboRos_->getParameter<double> (              update_rate_,                "updateRate",                            100.0 );
    
    
    gazeboRos_->getParameter         ( jointsMeasureType_[STEER],    "steering_measures_type", std::string("measured_position") );
    gazeboRos_->getParameter         ( jointsMeasureType_[REVOL],    "revolute_measures_type", std::string("measured_velocity") );
    
    
    for(size_t i = 0; i < jointsMeasureType_.size(); ++i ) {
	//if      ( !jointsMeasureType_[i].compare("measured_position") ) { getJointMeasure[i] = []( physics::JointPtr& _joint ){ return _joint->GetAngle   ( 0 ).Radian(); }; }		//DEPRECATED
	if      ( !jointsMeasureType_[i].compare("measured_position") ) { getJointMeasure[i] = []( physics::JointPtr& _joint ){ return _joint->Position(0); }; }
	else if ( !jointsMeasureType_[i].compare("measured_velocity") ) { getJointMeasure[i] = []( physics::JointPtr& _joint ){ return _joint->GetVelocity( 0 );          }; }
	else if ( !jointsMeasureType_[i].compare("measured_torque  ") ) { getJointMeasure[i] = []( physics::JointPtr& _joint ){ return _joint->GetForce   ( 0 );          }; }
	else {
	    getJointMeasure[i] = []( physics::JointPtr& _joint ){ return nan(""); }; 
	    ROS_ERROR("%s: Joint measurement mode: \"%s\" is not supported", gazeboRos_->info(), jointsMeasureType_[i].c_str()  );
	}
    }
    for(auto& setJointCmdTypeI : setJointCmd) { setJointCmdTypeI = []( physics::JointPtr& _joint, double _cmd ){}; }
    
    jointsCmd_.type_steering = "cmd_position";
    jointsCmd_.type_revolute = "cmd_position";
    setCmdMode(STEER, jointsCmd_.type_steering);
    setCmdMode(REVOL, jointsCmd_.type_revolute);
}

void GazeboRosBridgeModelPlugin::setCmdMode(size_t _jointsTypeIdx, const std::string& _mode) {
    if( !jointsCmdType_[_jointsTypeIdx].compare(_mode) ) { return; }
    
    jointsCmdType_[_jointsTypeIdx] = _mode;
    if      ( !_mode.compare("cmd_position") ) { setJointCmd[_jointsTypeIdx] = []( physics::JointPtr& _joint, double _cmd ){ _joint->SetPosition( 0, _cmd ); }; }
    else if ( !_mode.compare("cmd_velocity") ) { setJointCmd[_jointsTypeIdx] = []( physics::JointPtr& _joint, double _cmd ){ /*_joint->SetForce   ( 0, 1 * (_cmd - _joint->GetVelocity(0)) ); ROS_INFO("err=%lf, vel_des=%lf, vel_now=%lf, damping=%lf",_cmd - _joint->GetVelocity(0), _cmd, _joint->GetVelocity(0), _joint->GetDamping(0) );*/ _joint->SetVelocity( 0, _cmd ); }; }
    else if ( !_mode.compare("cmd_torque  ") ) { setJointCmd[_jointsTypeIdx] = []( physics::JointPtr& _joint, double _cmd ){ _joint->SetForce   ( 0, _cmd ); }; }
    else {
	setJointCmd[_jointsTypeIdx] = []( physics::JointPtr& _joint, double _cmd ){}; 
	if (joints_[_jointsTypeIdx].size() > 0) {
	    ROS_ERROR("%s: Joint cmd mode: \"%s\" is not supported", gazeboRos_->info(), _mode.c_str() );
	}
    }
}

void GazeboRosBridgeModelPlugin::loadJoints() {
    sdf::ElementPtr modelSdf = gazeboRos_->Sdf();
    if ( ! ( modelSdf->HasElement ( "steering_joints_names" )|| modelSdf->HasElement ( "revolute_joints_names" ) ) ) {
	ROS_ERROR("%s: Loaded SDF contains no steering_joints_names and no revolute_joints_names!", gazeboRos_->info());
    }
    if ( modelSdf->HasElement ( "steering_joints_names" ) ) {
	joints_[STEER].clear();
        sdf::ElementPtr element = modelSdf->GetElement ( "steering_joints_names" ) ;
	std::vector<string> steerJointsNamesVec;
        std::string steerJointsNames = element->Get<std::string>();
        boost::erase_all ( steerJointsNames, " " );
        boost::split ( steerJointsNamesVec, steerJointsNames, boost::is_any_of ( "," ) );
	for ( size_t i = 0; i < steerJointsNamesVec.size(); ++i ) {
	    joints_[STEER].emplace_back ( parent_->GetJoint ( steerJointsNamesVec[i] ) );
	    if( joints_[STEER].back() == nullptr ) { ROS_ERROR ( "%s: Name of revolute joint number %lu (%s) not found in model!", gazeboRos_->info(), i, steerJointsNamesVec[i].c_str() ); }
	}
    }
    if ( modelSdf->HasElement ( "revolute_joints_names" ) ) {
	joints_[REVOL].clear();
        sdf::ElementPtr element = modelSdf->GetElement ( "revolute_joints_names" ) ;
	std::vector<string> revolJointsNamesVec;
        std::string revolJointsNames = element->Get<std::string>();
        boost::erase_all ( revolJointsNames, " " );
        boost::split ( revolJointsNamesVec, revolJointsNames, boost::is_any_of ( "," ) );
	for ( size_t i = 0; i < revolJointsNamesVec.size(); ++i ) {
	    joints_[REVOL].emplace_back ( parent_->GetJoint ( revolJointsNamesVec[i] ) );
	    if( joints_[REVOL].back() == nullptr ) { ROS_ERROR ( "%s: Name of revolute joint number %lu (%s) not found in model!", gazeboRos_->info(), i, revolJointsNamesVec[i].c_str() ); }
	}
    }
    
    jointsCmd_.revolute.resize(joints_[REVOL].size(), 0);
    jointsCmd_.steering.resize(joints_[STEER].size(), 0);
}


void GazeboRosBridgeModelPlugin::setJointsConstraints() {
    for ( auto& steerJointI : joints_[STEER] ) {
	//gazebo::math::Angle angle;			//DEPRECATED
	ignition::math::Angle angle;
	//angle.SetFromRadian(constr_hi_stop_steering_); steerJointI->SetHighStop(0, angle );	//DEPRECATED
	steerJointI->SetUpperLimit(0, constr_hi_stop_steering_ );				
	//angle.SetFromRadian(constr_lo_stop_steering_); steerJointI->SetLowStop (0, angle );	//DEPRECATED
	steerJointI->SetLowerLimit (0, constr_lo_stop_steering_ );				
    }
    for ( size_t i = 0; i < joints_.size(); ++i ) {
	for ( auto& JointI : joints_[i] ) {
// 	    JointI->SetEffortLimit      (            0, constr_max_torque_[i] ); 
// 	    JointI->SetVelocityLimit    (            0, constr_max_vel_   [i] ); 
// 	    JointI->SetParam            ("friction", 0, constr_damping_   [i] );
// 	    JointI->SetDamping          (            0, constr_damping_   [i] ); 
// 	    JointI->Update();
	}
    }
}

void GazeboRosBridgeModelPlugin::initAllStates(const nav_msgs::Odometry& body_state) {
    body_state_ = body_state;
    for ( size_t i = 0; i < joints_.size(); ++i ) {
	for ( auto& JointI : joints_[i] ) {
	    JointI->Init(); 
	}
    }
    double leg_phi = 0; for ( auto& steerJointI : joints_[STEER] ) { steerJointI->SetPosition(0, leg_phi); }///@todo make it proper
    double leg_v   = 0; for ( auto& revolJointI : joints_[REVOL] ) { revolJointI->SetVelocity(0, leg_v  ); }///@todo make it proper
}



// read/write into sim
void GazeboRosBridgeModelPlugin::UpdateChild() {
    if(!firstChildUpdate_){ setJointsConstraints(); firstChildUpdate_ = true; }
    
    common::Time current_time = parent_->GetWorld()->SimTime();
    double seconds_since_last_update = ( current_time - last_update_time_ ).Double();
    
    if ( seconds_since_last_update > update_period_ ) {
        last_update_time_ = current_time;

	if ( true )                { publishJointMeasurements(); }
        if ( publishOdometry_ )    { publishWorldOdometry ( seconds_since_last_update ); }
        if ( publishTfs_ )         { publishTFs(); }
        if ( publishJointStates_ ) { publishJointsStates(); }
    } 
    for ( size_t i = 0; i < jointsCmd_.steering.size(); ++i ) { setJointCmd[STEER](joints_[STEER][i], jointsCmd_.steering[i]); }
    for ( size_t i = 0; i < jointsCmd_.revolute.size(); ++i ) { setJointCmd[REVOL](joints_[REVOL][i], jointsCmd_.revolute[i]); }
}

void GazeboRosBridgeModelPlugin::callbackConfig ( tuw_gazebo_plugins::GazeboRosBridgeModelPluginConfig& _config, uint32_t _level ) {
    config_ = _config;
    ROS_DEBUG ( "%s: callbackConfig!", gazeboRos_->info() );
}

void GazeboRosBridgeModelPlugin::jointsCmdCallback ( const tuw_nav_msgs::JointsIWS::ConstPtr& _msgJointsCmd ) {
    jointsCmd_ = *_msgJointsCmd;
    if ( jointsCmd_.steering.size() != joints_[STEER].size() ){ ROS_WARN("%s Received a meesage with \"steering\" field size not equal to %lu", gazeboRos_->info(), joints_[STEER].size() ); }
    if ( jointsCmd_.revolute.size() != joints_[REVOL].size() ){ ROS_WARN("%s Received a meesage with \"revolute\" field size not equal to %lu", gazeboRos_->info(), joints_[REVOL].size() ); }
    setCmdMode(STEER, jointsCmd_.type_steering);
    setCmdMode(REVOL, jointsCmd_.type_revolute);
}


void GazeboRosBridgeModelPlugin::publishJointsStates() {
    ros::Time current_time = ros::Time::now();

    joint_state_.header.stamp = current_time;
    size_t allJointsSize = 0; for ( auto& jointsTypeI : joints_) { allJointsSize += jointsTypeI.size(); }
    joint_state_.name.resize     ( allJointsSize );
    joint_state_.position.resize ( allJointsSize );
    joint_state_.velocity.resize ( allJointsSize );
    size_t i = 0;
    for ( auto& jointsTypeI : joints_ ) {
	for ( auto& jointI : jointsTypeI ) {
	    joint_state_.name    [i] = jointI->GetName    (   );
	    //joint_state_.position[i] = jointI->GetAngle   ( 0 ).Radian() ;		//DEPRECATED
	    joint_state_.position[i] = jointI->Position(0) ;
	    joint_state_.velocity[i] = jointI->GetVelocity( 0 ) ;
	    i++;
	}
    }
    pubJointsStates_.publish ( joint_state_ );
}

void GazeboRosBridgeModelPlugin::publishJointMeasurements() {
    ros::Time current_time = ros::Time::now();

    boost::shared_ptr<tuw_nav_msgs::JointsIWS> joints_measure_(new tuw_nav_msgs::JointsIWS);
    joints_measure_->header.stamp = current_time;
    
    joints_measure_->type_revolute = jointsMeasureType_[REVOL];
    joints_measure_->revolute.resize( joints_[REVOL].size() );
    for ( size_t i = 0; i < joints_[REVOL].size(); ++i ) { joints_measure_->revolute[i] = getJointMeasure[REVOL](joints_[REVOL][i]); }
    
    joints_measure_->type_steering = jointsMeasureType_[STEER];
    joints_measure_->steering.resize( joints_[STEER].size() );
    for ( size_t i = 0; i < joints_[STEER].size(); ++i ) { joints_measure_->steering[i] = getJointMeasure[STEER](joints_[STEER][i]); }
    
    pubJointMeasures_.publish ( joints_measure_ );
}


void GazeboRosBridgeModelPlugin::publishTFs() {
    ros::Time current_time = ros::Time::now();
    size_t i = 0;
    for ( auto& jointsTypeI : joints_ ) {
	for ( auto& jointI : jointsTypeI ) {
	    string child_frame  = gazeboRos_->resolveTF(jointI-> GetChild()->GetName ());
	    string parent_frame = gazeboRos_->resolveTF(jointI->GetParent()->GetName ());
        
	    //math::Pose pose_child   = jointI->GetChild() ->GetRelativePose();//this returns relative pose to parent model! i.e. base link	//DEPRECATED
	    ignition::math::Pose3d pose_child = jointI->GetChild()->RelativePose();//this returns relative pose to parent model! i.e. base link
	    //math::Pose pose_parrent = jointI->GetParent()->GetRelativePose();									//DEPRECATED
	    ignition::math::Pose3d pose_parrent = jointI->GetParent()->RelativePose();
	    //math::Pose pose_rel_to_parrent = pose_child - pose_parrent;									//DEPRECATED
	    ignition::math::Pose3d pose_rel_to_parrent = pose_child - pose_parrent;

	    //tf::Quaternion qt ( pose_rel_to_parrent.rot.x, pose_rel_to_parrent.rot.y, pose_rel_to_parrent.rot.z, pose_rel_to_parrent.rot.w );	//DEPRECATED
	    tf::Quaternion qt ( pose_rel_to_parrent.Rot().X(),pose_rel_to_parrent.Rot().Y(),pose_rel_to_parrent.Rot().Z(),pose_rel_to_parrent.Rot().W() );
	    //tf::Vector3    vt ( pose_rel_to_parrent.pos.x, pose_rel_to_parrent.pos.y, pose_rel_to_parrent.pos.z );				//DEPRECATED			
	    tf::Vector3    vt ( pose_rel_to_parrent.Pos().X(), pose_rel_to_parrent.Pos().Y(), pose_rel_to_parrent.Pos().Z() );

	    tf::Transform tfChild ( qt, vt );
	    transform_broadcaster_->sendTransform ( tf::StampedTransform ( tfChild, current_time, parent_frame, child_frame ) ); 
	}
    }
}

void GazeboRosBridgeModelPlugin::publishWorldOdometry ( double step_time ) {
   
    ros::Time current_time           = ros::Time::now();
    std::string odom_frame           = gazeboRos_->resolveTF ( odometry_frame_   );
    std::string base_footprint_frame = gazeboRos_->resolveTF ( robot_base_frame_ );

    tf::Quaternion qt;
    tf::Vector3 vt;
    
    //math::Pose pose = parent_->GetWorldPose();					//DEPRECATED
    ignition::math::Pose3d pose = parent_->WorldPose();				
    
    //getting data form encoder integration
    //qt = tf::Quaternion ( pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w );
    qt = tf::Quaternion ( pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W() );		//DEPRECATED
    //vt = tf::Vector3    ( pose.pos.x, pose.pos.y, pose.pos.z );
    vt = tf::Vector3    ( pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z() );				//DEPRECATED
    
//     if (this->publishTfs_) {
	tf::Transform base_footprint_to_odom  ( qt, vt );
	transform_broadcaster_->sendTransform ( tf::StampedTransform ( base_footprint_to_odom, current_time, odom_frame, base_footprint_frame ) );
//     }
    
    nav_msgs::Odometry odom_;
    
    // set pose position						//DEPRECATED
    odom_.pose.pose.position.x = pose.Pos().X();
    odom_.pose.pose.position.y = pose.Pos().Y();

    odom_.pose.pose.orientation.x = pose.Rot().X();
    odom_.pose.pose.orientation.y = pose.Rot().Y();
    odom_.pose.pose.orientation.z = pose.Rot().Z();
    odom_.pose.pose.orientation.w = pose.Rot().W();
    odom_.pose.covariance[0 ] = 0.00001;
    odom_.pose.covariance[7 ] = 0.00001;
    odom_.pose.covariance[14] = 1000000000000.0;
    odom_.pose.covariance[21] = 1000000000000.0;
    odom_.pose.covariance[28] = 1000000000000.0;
    odom_.pose.covariance[35] = 0.01;

    // set pose velocity
    //math::Vector3 linear;				//DEPRECATED
    ignition::math::Vector3<double> linear;
    linear = parent_->WorldLinearVel();
    odom_.twist.twist.angular.z = parent_->WorldAngularVel().Z();

    // convert velocity to child_frame_id (aka base_footprint)
    //float yaw = pose.rot.GetYaw();						//DEPRECATED
    float yaw = pose.Rot().Yaw();
    odom_.twist.twist.linear.x = cos(yaw) * linear.X() + sin(yaw) * linear.Y();
    odom_.twist.twist.linear.y = cos(yaw) * linear.Y() - sin(yaw) * linear.X();

    // set header
    odom_.header.stamp    = current_time;
    odom_.header.frame_id = odom_frame;
    odom_.child_frame_id  = base_footprint_frame;

    pubOdometry_.publish ( odom_ );
}

void GazeboRosBridgeModelPlugin::Init() {
    gazebo::ModelPlugin::Init();
}


void GazeboRosBridgeModelPlugin::Reset() {
    gazebo::ModelPlugin::Reset();
    last_update_time_ = parent_->GetWorld()->SimTime();
    initAllStates( body_state_initial_ ); 
}

void GazeboRosBridgeModelPlugin::QueueThread() {
    static const double timeout = 0.01;
    while ( alive_ && gazeboRos_->node()->ok() ) { queue_.callAvailable ( ros::WallDuration ( timeout ) ); }
}


GZ_REGISTER_MODEL_PLUGIN ( GazeboRosBridgeModelPlugin )
}

#!/usr/bin/env python

import roslib; roslib.load_manifest('tuw_gazebo')
import rospy
import actionlib
import time
import tf
from visualization_msgs.msg import *
from std_msgs.msg import String
from geometry_msgs.msg import *
from gazebo_msgs.msg import *
#from pudb import set_trace; set_trace()

robot_name = "r1"
covarianceXY = 0.1
covarianceAlpha = 0.05

class SetRobotNode():
  def __init__(self):   
    rospy.init_node('goal_test_client') 
    rospy.on_shutdown(self.shutdown)
    self.pub_command_topic_name = robot_name + '/cmd_vel'
    self.pub_command = rospy.Publisher(self.pub_command_topic_name, Twist, queue_size=10)
    self.pub_model_state_topic_name = 'gazebo/set_model_state'
    self.pub_model_state = rospy.Publisher(self.pub_model_state_topic_name, ModelState, queue_size=10)
    rospy.sleep(1.0)

  def shutdown(self):
    rospy.loginfo("end")

  def stoppRobot(self):
    cmd = Twist()
    cmd.linear.x = 0
    cmd.linear.y = 0
    cmd.linear.z = 0
    cmd.angular.x = 0
    cmd.angular.y = 0
    cmd.angular.z = 0
    self.pub_command.publish(cmd)
    rospy.loginfo("Stopp robot on: " + self.pub_command_topic_name)

  def setRobot(self, pose):
    model_state = ModelState()
    model_state.model_name = robot_name
    model_state.reference_frame = 'world'
    model_state.pose.position.x = poseWorld.position.x
    model_state.pose.position.y = poseWorld.position.y
    model_state.pose.orientation.x = poseWorld.orientation.x
    model_state.pose.orientation.y = poseWorld.orientation.y
    model_state.pose.orientation.z = poseWorld.orientation.z
    model_state.pose.orientation.w = poseWorld.orientation.w
    self.pub_model_state.publish(model_state)
    model_state.pose.position.z = 0.1
    rospy.loginfo("Set model on: " + self.pub_model_state_topic_name)
    

if __name__ == '__main__':
  try:
    poseWorld = Pose()
    poseWorld.position.x = 0
    poseWorld.position.y = 0
    poseWorld.position.z = 0
    poseWorld.orientation.x = 0
    poseWorld.orientation.y = 0
    poseWorld.orientation.z = 0
    poseWorld.orientation.w = 1
    
    
    print "\nusage:  " + sys.argv[0] + " model_name x y theta\n"
    if (len(sys.argv) > 1) :
      robot_name = sys.argv[1]
    if (len(sys.argv) > 2) :
      poseWorld.position.x = float(sys.argv[2])
    if (len(sys.argv) > 3) :
      poseWorld.position.y = float(sys.argv[3])
    if (len(sys.argv) > 4) :
      poseWorld.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, float(sys.argv[4])))
      
    robot = SetRobotNode()
    robot.stoppRobot()
    rospy.sleep(1.0)
    robot.setRobot(poseWorld)
  except rospy.ROSInterruptException:
    pass

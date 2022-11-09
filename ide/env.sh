#export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
#export RMW_IMPLEMENTATION=rmw_gurumdds_cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

#export GAZEBO_PLUGIN_PATH=$HOME/projects/ros2/gazebo/ws00/install/gazebo_plugins/lib/

source /opt/ros/$ROS_DISTRO/setup.bash
echo "** ROS2 $ROS_DISTRO initialized with $RMW_IMPLEMENTATION**"
source ${HOME}/projects/ros2/gazebo/ws00/install/setup.bash
source ${HOME}/projects/ros2/gazebo/ws01/install/setup.bash

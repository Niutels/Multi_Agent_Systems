# Multi_Agent_Systems

Useful lines to put in your .bashrc:
source /opt/ros/kinetic/setup.bash
source /home/<user>/<workspace>/devel/setup.bash
alias cdc='cd ~/<workspace> && catkin_make && source devel/setup.bash'
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/home/<user>/<workspace>/src/turtlebot3_gazebo_plugin/build
export GAZEBO_MODEL_PATH=/home/<user>/<workspace>/src/turtlebot3_gazebo_plugin/models
export URDF_PATH=/home/<user>/<workspace>/src/turtlebot3/turtlebot3_description/urdf


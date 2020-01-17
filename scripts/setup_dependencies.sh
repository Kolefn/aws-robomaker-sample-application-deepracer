#!/usr/bin/env bash

# download and setup rotors_simulator dependencies
wstool init simulation_ws/src/
wget -O simulation_ws/src/rotors_hil.rosinstall https://raw.githubusercontent.com/ethz-asl/rotors_simulator/master/rotors_hil.rosinstall
wstool merge -t simulation_ws/src simulation_ws/src/rotors_hil.rosinstall
wstool update -t simulation_ws/src

# install additional dependencies for rotors_simulator
curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py
sudo python get-pip.py
sudo pip install future
sudo apt-get install libgeographic-dev
sudo apt-get install geographiclib-tools
sudo apt-get install libignition-math2-dev

# fix bug in dependency code
sed -i 's/msg.twist_covariance/msg.velocity_covariance/g' simulation_ws/src/mavros/mavros_extras/src/plugins/odom.cpp

# patch rotors_gazebo_plugins build so libmav_msgs.so is available to the librotors_gazebo_ros_interface_plugin
sed -i '216i list(APPEND targets_to_install mav_msgs)' simulation_ws/src/rotors_simulator/rotors_gazebo_plugins/CMakeLists.txt

# additional deps
sudo apt-get install ros-kinetic-ros-control ros-kinetic-ros-controllers

# Setup for bundling & running
sudo rosdep init
rosdep update
rosdep install --from-paths simulation_ws/src --ignore-src -r -y
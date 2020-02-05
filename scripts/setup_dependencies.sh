
# download and setup rotors_simulator dependencies
wstool init simulation_ws/src/
wget -O simulation_ws/src/rotors_hil.rosinstall https://raw.githubusercontent.com/ethz-asl/rotors_simulator/master/rotors_hil.rosinstall
wstool merge -t simulation_ws/src simulation_ws/src/rotors_hil.rosinstall
wstool update -t simulation_ws/src

# install additional dependencies for rotors_simulator
sudo apt-get install -y \
    libgeographic-dev \
    geographiclib-tools \
    libignition-math2-dev \
    ros-kinetic-ros-control \
    ros-kinetic-ros-controllers \
    ntp

# fix bug in dependency code
sed -i 's/msg.twist_covariance/msg.velocity_covariance/g' simulation_ws/src/mavros/mavros_extras/src/plugins/odom.cpp

# patch rotors_gazebo_plugins build so libmav_msgs.so is available to the librotors_gazebo_ros_interface_plugin
sed -i '216i list(APPEND targets_to_install mav_msgs)' simulation_ws/src/rotors_simulator/rotors_gazebo_plugins/CMakeLists.txt

# Setup for bundling & running
sudo rosdep init
rosdep update
rosdep install --from-paths simulation_ws/src --ignore-src -r -y

# Install Python packages used by build scripts and the sagemaker agent
python3 -m pip install -U pip
python3 -m pip install 'zipp==1.0.0' 'tensorflow<2'
python3 -m pip install -U \
    awscli \
    boto3 \
    futures \
    gym \
    kubernetes \
    minio \
    numpy \
    pandas \
    Pillow \
    pygame \
    PyYAML \
    redis \
    rospkg \
    scipy \
    rl-coach-slim

# Pre build setup
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Build the workspace
cd simulation_ws/
colcon build
cd ..

# Post Build Setup
echo "source $(pwd)/simulation_ws/install/local_setup.bash" >> ~/.bashrc
source ~/.bashrc
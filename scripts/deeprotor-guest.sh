#!/usr/bin/env bash

source $(dirname $BASH_SOURCE)/deeprotor-common.sh

# Configures roslaunch and catkin.
ROS_BASE_SETUP=/opt/ros/kinetic/setup.bash

# Configures roslaunch to use the built project, including the base setup.
ROS_RUN_SETUP=$DEEPROTOR_ROOT/simulation_ws/install/setup.bash

deeprotor-guest() {
  case "$1" in
    setup)
      setup
      ;;
    train-local)
      shift
      launch-simulation local_training $@ 
      ;;
    evaluate)
      shift
      launch-simulation evaluate $@ 
      ;;
    run-gui)
      run_gui
      ;;
    build-local)
      shift
      build_local $@
      ;;
    refresh-creds)
      shift
      refresh_creds $@ 
      ;;
    *)
      echo "Usage: deeprotor cd|setup|refresh-creds|build-local|train-local||run-gui"
      ;;
  esac
}

# Sources and exports variables necessary to run the GUI or simulation.
export_run_env() {
  export_env_and_creds
  source $ROS_RUN_SETUP
  export DISPLAY=":$XDISPLAY_NUM"
  export XAUTHORITY="$XAUTHORITY_PATH"
}

launch-simulation() {  
  export_run_env
  start_gui
  if [ "$2" == "--verbose" ]; then
    roslaunch -v $SIMULATION_PACKAGE $1.launch verbose:=true
  else
    roslaunch $SIMULATION_PACKAGE $1.launch
  fi
}

# Start a headless X server running an LXQT desktop
run_Xvfb() {
  xvfb-run \
    --server-num=$XDISPLAY_NUM \
    --auth-file=$XAUTHORITY \
    --server-args="-screen 0 ${SCREEN_WIDTH}x${SCREEN_HEIGHT}x24" \
    startlxqt
}

# Start a VNC server accessible without a password at localhost:5900
run_x11vnc() {
  x11vnc \
    -auth $XAUTHORITY \
    -display $DISPLAY \
    -nevershared \
    -loop \
    -forever
}

# Start the GUI in the background and kill it when the shell exits 
start_gui() {
  trap 'kill $(jobs -p)' EXIT
  run_Xvfb &
  run_x11vnc &
}

# Standalone command to start the gui
run_gui() {
  export_run_env
  start_gui
  tail -f /dev/null
}

build_local() {
  source $ROS_BASE_SETUP
  cd simulation_ws
  colcon build --symlink-install $@ 
}

setup() {
  echo "Installing ROS"
  bash -x scripts/setup_ros.sh || quit "Error installing ROS"

  echo "Adding ROS setup to profile"
  source_from_profile $ROS_BASE_SETUP

  echo "Installing project dependencies"
  bash -x scripts/setup_dependencies.sh || quit "Error installing dependencies"

  echo "Adding deeprotor CLI to profile"
  source_from_profile $DEEPROTOR_CLI
}

source_from_profile() {
  local file=$1
  source $file
  if ! grep -q "$file" ~/.bashrc; then
    echo "Sourcing $file from ~/.bashrc"
    printf "\nsource $file\n" >> ~/.bashrc
  fi
}

log deeprotor-guest $@
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
    build-local)
      shift
      build_local $@
      ;;
    refresh-creds)
      shift
      refresh_creds $@ 
      ;;
    *)
      echo "Usage: deeprotor train-local|build-local|refresh-creds|cd|setup"
      ;;
  esac
}

launch-simulation() {
  export_env_and_creds
  source $ROS_RUN_SETUP

  if [ "$2" == "--verbose" ]; then
    roslaunch -v deeprotor_simulation $1.launch verbose:=true
  else
    roslaunch deeprotor_simulation $1.launch
  fi
}

build_local() {
  source $ROS_BASE_SETUP
  cd simulation_ws
  colcon build $@ 
}

refresh_creds() {
  local profile=${1:-default}

  [ "$(aws configure get output --profile $profile)" == "text" ] || \
      quit "Add 'output = text' to your profile at ~/.aws/config"

  aws sts get-session-token --duration-seconds 129600 --profile $profile | \
  awk '{ 
    print "AWS_ACCESS_KEY_ID=" $2; 
    print "AWS_SECRET_ACCESS_KEY=" $4;
    print "AWS_SESSION_TOKEN=" $5;
  }' > .creds
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

deeprotor-guest $@
#!/usr/bin/env bash

source $(dirname $BASH_SOURCE)/deeprotor-common.sh

deeprotor() {
  case "$1" in
    ""|help)
      echo "Usage: deeprotor setup|cd|up|ssh|halt"
      ;;
    setup)
      setup
      ;;
    destroy)
      echo "Use `vagrant destroy` if you really want to destroy this VM"
      return
      ;;
    *)
      # Forward everything else to vagrant
      vagrant $@
      ;;
  esac
}

setup() {
  [ -z "$(which VBoxManage)" ] && \
      quit "VBoxManage not on path. Is Virtualbox installed?"  
  [ -z "$(which vagrant)" ] && \
      quit "vagrant not on path. Is Vagrant installed?"

  if ! grep -q "$DEEPROTOR_CLI" ~/.bash_profile; then
    echo "Adding deeprotor command to profile"
    printf "\nsource $DEEPROTOR_CLI\n" >> ~/.bash_profile
  fi

  if [ ! -e .env ]; then
    echo "Initializing .env"
    echo "source .env.default" > .env
  fi

  echo "Installing Vagrant plugins"
  vagrant plugin list | grep -q vagrant-vbguest || \
      vagrant plugin install vagrant-vbguest
  vagrant plugin list | grep -q vagrant-disksize || \
      vagrant plugin install vagrant-disksize

  echo "Bringing up VM"
  vagrant up
}

deeprotor $@
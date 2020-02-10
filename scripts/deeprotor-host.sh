#!/usr/bin/env bash

source $(dirname $BASH_SOURCE)/deeprotor-common.sh

deeprotor-host() {
  case "$1" in
    ""|help)
      echo "Usage: deeprotor setup|cd|up|ssh|halt"
      ;;
    setup)
      shift
      setup $@
      ;;
    destroy)
      echo "Use `vagrant destroy` if you really want to destroy this VM"
      return
      ;;
    refresh-creds)
      shift
      refresh_creds $@ 
      ;;
    *)
      # Forward everything else to vagrant
      export_env_and_creds
      vagrant $@
      ;;
  esac
}

setup() {
  local profile=${1:-~/.bash_profile}

  [ -z "$(which VBoxManage)" ] && \
      quit "VBoxManage not on path. Is Virtualbox installed?"  
  [ -z "$(which vagrant)" ] && \
      quit "vagrant not on path. Is Vagrant installed?"

  if ! grep -q "$DEEPROTOR_CLI" $profile; then
    echo "Adding deeprotor command to profile"
    printf "\nsource $DEEPROTOR_CLI\n" >> $profile
  fi

  if [ ! -e .env ]; then
    echo "Initializing .env"
    echo "source .env.default" > .env
  fi

  echo "Installing Vagrant plugins"
  vagrant plugin list | grep -q vagrant-vbguest || \
      sudo vagrant plugin install vagrant-vbguest
  vagrant plugin list | grep -q vagrant-disksize || \
      sudo vagrant plugin install vagrant-disksize

  echo "Bringing up VM"
  vagrant up
}

log deeprotor-host $@
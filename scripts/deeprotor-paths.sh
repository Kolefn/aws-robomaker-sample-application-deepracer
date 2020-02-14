#!/usr/bin/env bash
# Exports project paths on the host and guest.

# Credit: https://stackoverflow.com/users/2351351/geoff-nixon
# source: https://stackoverflow.com/a/18443300
realpath() {
  OURPWD=$PWD
  cd "$(dirname "$1")"
  LINK=$(readlink "$(basename "$1")")
  while [ "$LINK" ]; do
    cd "$(dirname "$LINK")"
    LINK=$(readlink "$(basename "$1")")
  done
  REALPATH="$PWD/$(basename "$1")"
  cd "$OURPWD"
  echo "$REALPATH"
}

export DEEPROTOR_ROOT=$(realpath $(dirname $BASH_SOURCE)/..)
export DEEPROTOR_CLI=$DEEPROTOR_ROOT/scripts/deeprotor
export DEEPROTOR_HOST=$DEEPROTOR_ROOT/scripts/deeprotor-host.sh
export DEEPROTOR_GUEST=$DEEPROTOR_ROOT/scripts/deeprotor-guest.sh
# Exports project paths on the host and guest.

export DEEPROTOR_ROOT=$(realpath $(dirname $BASH_SOURCE)/..)
export DEEPROTOR_CLI=$DEEPROTOR_ROOT/scripts/deeprotor
export DEEPROTOR_HOST=$DEEPROTOR_ROOT/scripts/deeprotor-host.sh
export DEEPROTOR_GUEST=$DEEPROTOR_ROOT/scripts/deeprotor-guest.sh
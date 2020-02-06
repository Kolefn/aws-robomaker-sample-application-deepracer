# Provides shared functionality for the host and guest scripts.

source $(dirname $BASH_SOURCE)/deeprotor-paths.sh

cd $DEEPROTOR_ROOT

quit() {
  echo $@
  exit 1
}
# Provides shared functionality for the host and guest scripts.

source $(dirname $BASH_SOURCE)/deeprotor-paths.sh

cd $DEEPROTOR_ROOT

quit() {
  echo $@
  exit 1
}

export_env_and_creds() {
  ENVFILE=${ENVFILE:-.env}
  CREDFILE=${CREDFILE:-.creds}
  
  set -o allexport
    [ -e "$ENVFILE" ] && source $ENVFILE
    [ -e "$CREDFILE" ] && source $CREDFILE
  set +o allexport
}
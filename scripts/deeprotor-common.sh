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

log() {
  mkdir -p log
  local exec_time=$(date -Iseconds)
  local log_file="$DEEPROTOR_ROOT/log/$exec_time.log"
  ln -sf $log_file "$DEEPROTOR_ROOT/log/latest"
  echo "$exec_time > $@" > $log_file
  $@ 2>&1 | tee -a "$log_file"
}

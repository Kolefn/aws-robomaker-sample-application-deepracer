ACCESS_KEY_ID=$1
SECRET_ACCESS_KEY=$2
SESSION_TOKEN=$3

echo "export MARKOV_PRESET_FILE=\"deeprotor.py\"" >> ~/.bashrc
echo "export MODEL_S3_BUCKET=\"deeprotor-model-bucket\"" >> ~/.bashrc
echo "export MODEL_S3_PREFIX=\"model-store\"" >> ~/.bashrc
echo "export WORLD_NAME=\"easy_track\"" >> ~/.bashrc
echo "export ROS_AWS_REGION=\"us-east-1\"" >> ~/.bashrc
echo "export AWS_ACCESS_KEY_ID=\"$ACCESS_KEY_ID\"" >> ~/.bashrc
echo "export AWS_SECRET_ACCESS_KEY=\"$SECRET_ACCESS_KEY\"" >> ~/.bashrc
echo "export AWS_SESSION_TOKEN=\"$SESSION_TOKEN\"" >> ~/.bashrc

source ~/.bashrc
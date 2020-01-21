# Deep Rotor

A simulation which trains a reinforcement learning (RL) model to fly a drone.

_AWS RoboMaker sample applications include third-party software licensed under open-source licenses and is provided for demonstration purposes only. Incorporation or use of RoboMaker sample applications in connection with your production workloads or a commercial products or devices may affect your legal rights or obligations under the applicable open-source licenses. Source code information can be found [here](https://s3.console.aws.amazon.com/s3/buckets/robomaker-applications-us-east-1-72fc243f9355/deep-racer/?region=us-east-1)._

Keywords: Reinforcement learning, AWS, RoboMaker, Drone

## Requirements

- ROS Kinetic / Melodic (optional) - To run the simulation locally. Other distributions of ROS may work, however they have not been tested
- Gazebo (optional) - To run the simulation locally
- An AWS S3 bucket - To store the trained reinforcement learning model

## AWS Account Setup

### AWS Credentials
You will need to create an AWS Account and configure the credentials to be able to communicate with AWS services. You may find [AWS Configuration and Credential Files](https://docs.aws.amazon.com/cli/latest/userguide/cli-config-files.html) helpful.

### Creating a Session Token

1. Download and install the Amazon Command Line Interface http://docs.aws.amazon.com/cli/latest/userguide/installing.html
2. Configure the command line interface http://docs.aws.amazon.com/cli/latest/userguide/cli-chap-getting-started.html
    - `aws configure`
2. Request the session token and temp access key/secret
    - `aws sts get-session-token --duration-seconds 129600`
3. Set the `AWS_ACCESS_KEY_ID`, `AWS_SECRET_ACCESS_KEY`, `AWS_SESSION_TOKEN` env vars with the output


### AWS Permissions

To train the reinforcement learning model in simulation, you need an IAM role with the following policy. You can find instructions for creating a new IAM Policy
[here](https://docs.aws.amazon.com/IAM/latest/UserGuide/access_policies_create.html#access_policies_create-start). In the JSON tab paste the following policy document:

```
{
    "Version": "2012-10-17",
    "Statement": [
        {
            "Action": [
                "cloudwatch:PutMetricData",
                "logs:CreateLogGroup",
                "logs:CreateLogStream",
                "logs:PutLogEvents",
                "logs:DescribeLogStreams",
                "s3:Get*",
                "s3:List*",
                "s3:Put*",
                "s3:DeleteObject"
            ],
            "Effect": "Allow",
            "Resource": "*"
        }
    ]
}
```

### Getting Started
1. Setup your development VM by following [these instructions](https://github.com/Kolefn/cantina-deeprotor/wiki/VM-Setup).

Run these commands in your VM terminal

2. `git clone https://github.com/kolefn/cantina-deeprotor.git`
3. `cd cantina-deeprotor`
4. `bash ./scripts/setup_ros.sh` - **10 Minutes (Y/n prompts)**
5. `bash ./scripts/setup_dependencies.sh` - **3 Minutes (Y/n prompts)**
6. `bash ./scripts/first_build.sh` - **~10 Minutes**

#### Running the simulation


The following environment variables must be set when you run your simulation:

- `MARKOV_PRESET_FILE` - Defines the hyperparameters of the reinforcement learning algorithm. This should be set to `deeprotor.py`.
- `MODEL_S3_BUCKET` - The name of the S3 bucket in which you want to store the trained model.
- `MODEL_S3_PREFIX` - The path where you want to store the model.
- `WORLD_NAME` - The track to train the model on. Can be one of easy_track, medium_track, or hard_track.
- `ROS_AWS_REGION` - The region of the S3 bucket in which you want to store the model.
- `AWS_ACCESS_KEY_ID` - The access key for the role you created in the "AWS Permissions" section
- `AWS_SECRET_ACCESS_KEY` - The secret access key for the role you created in the "AWS Permissions" section
- `AWS_SESSION_TOKEN` - The session token for the role you created in the "AWS Permissions" section

Once the environment variables are set, you can run local training using the roslaunch command

```bash
source simulation_ws/install/setup.sh
roslaunch deeprotor_simulation local_training.launch
```

#### Seeing your robot learn

As the reinforcement learning model improves, the reward function will increase. You can see the graph of this reward function at

All -> AWSRoboMakerSimulation -> Metrics with no dimensions -> Metric Name -> DeepRotorRewardPerEpisode

You can think of this metric as an indicator into how well your model has been trained. If the graph has plateaus, then your robot has finished learning.

![deeprotor-metrics.png](docs/images/deeprotor-metrics.png)

### Evaluating the model

#### Building the simulation bundle

You can reuse the bundle from the training phase again in the simulation phase.

#### Running the simulation

The evaluation phase requires that the same environment variables be set as in the training phase. Once the environment variables are set, you can run
evaluation using the roslaunch command

```bash
source simulation_ws/install/setup.sh
roslaunch deeprotor_simulation evaluation.launch
```

### Troubleshooting

###### The robot does not look like it is training

The training algorithm has two phases. The first is when the reinforcement learning model is used to make the drone move in the world, 
while the second is when the algorithm uses the information gathered in the first phase to improve the model. In the second
phase, no new commands are sent to the drone, meaning it will appear as if it is stopped, spinning in circles, or drifting off
aimlessly.

## Using this sample with AWS RoboMaker

You first need to install colcon. Python 3.5 or above is required.

```bash
apt-get update
apt-get install -y python3-pip python3-apt
pip3 install colcon-ros-bundle
```

After colcon is installed you need to build your robot or simulation, then you can bundle with:

```bash
# Bundling Simulation Application
cd simulation_ws
colcon bundle
```

This produces `simulation_ws/bundle/output.tar`.
You'll need to upload this artifact to an S3 bucket. You can then use the bundle to
[create a simulation application](https://docs.aws.amazon.com/robomaker/latest/dg/create-simulation-application.html),
and [create a simulation job](https://docs.aws.amazon.com/robomaker/latest/dg/create-simulation-job.html) in AWS RoboMaker.

## License

Most of this code is licensed under the MIT-0 no-attribution license. However, the sagemaker_rl_agent package is
licensed under Apache 2. See LICENSE.txt for further information.

## How to Contribute

Create issues and pull requests against this Repository on Github

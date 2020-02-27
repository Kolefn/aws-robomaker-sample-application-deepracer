from __future__ import print_function

import time

# only needed for fake driver setup
import boto3
# gym
import gym
import numpy as np
from gym import spaces
from PIL import Image
import os
import math
import random

# Type of worker
SIMULATION_WORKER = "SIMULATION_WORKER"
SAGEMAKER_TRAINING_WORKER = "SAGEMAKER_TRAINING_WORKER"

node_type = os.environ.get("NODE_TYPE", SIMULATION_WORKER)

if node_type == SIMULATION_WORKER:
    import rospy
    from gazebo_msgs.msg import ModelState
    from gazebo_msgs.srv import SetModelState, GetModelState
    from scipy.spatial.transform import Rotation
    from sensor_msgs.msg import Image as sensor_image
    from mav_msgs.msg import RollPitchYawrateThrust

TRAINING_IMAGE_SIZE = (160, 120)

# SLEEP INTERVALS
SLEEP_AFTER_RESET_TIME_IN_SECOND = 0.5
SLEEP_BETWEEN_ACTION_AND_REWARD_CALCULATION_TIME_IN_SECOND = 0.1
SLEEP_WAITING_FOR_IMAGE_TIME_IN_SECOND = 0.01

MAX_NUM_STEPS_PER_EPISODE = 100
WORLD_BOUND_X = 15
WORLD_BOUND_Y = 15
WORLD_BOUND_Z = 15
RESET_ACTION = [0,0,0,0]

DEG_TO_RAD = math.pi / 180.0

FULL_ROTATION = 360 * DEG_TO_RAD

MAX_ROLL = 10.0 * DEG_TO_RAD
MAX_PITCH = 10.0 * DEG_TO_RAD
MAX_YAW_RATE = 45.0 * DEG_TO_RAD
MAX_THRUST = 10.0



### Gym Env ###
class DeepRotorEnv(gym.Env):
    def __init__(self):

        screen_height = TRAINING_IMAGE_SIZE[1]
        screen_width = TRAINING_IMAGE_SIZE[0]

        self.start_x = 0
        self.start_y = 0
        self.start_z = 0

        self.reward_in_episode = 0
        self.reward = None
        self.done = False
        self.next_state = None
        self.image = None
        self.steps = 0

        # actions -> [rotor_0, rotor_1, rotor_2, rotor_3] w/ values 0->1
        self.action_space = spaces.Box(low=np.array([0, 0, 0, 0]), high=np.array([+1, +1, +1, +1]), dtype=np.float32)

        # given image from simulator
        self.observation_space = spaces.Box(low=0, high=255,
                                            shape=(screen_height, screen_width, 3), dtype=np.uint8)

        if node_type == SIMULATION_WORKER:
            # ROS initialization
            rospy.wait_for_service('/gazebo/get_model_state')
            rospy.wait_for_service('/gazebo/set_model_state')
            self.get_model_service = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            self.set_model_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            self.control_publisher = rospy.Publisher("/drone/command/roll_pitch_yawrate_thrust", RollPitchYawrateThrust, queue_size=1)

            rospy.init_node('rl_coach', anonymous=True)

            # Subscribe to ROS topics and register callbacks
            rospy.Subscriber('/drone/camera/zed/rgb/image_rect_color', sensor_image, self.callback_image)

            self.world_name = rospy.get_param('WORLD_NAME')
            self.aws_region = rospy.get_param('ROS_AWS_REGION')

    def reset(self):
        if node_type == SAGEMAKER_TRAINING_WORKER:
            return self.observation_space.sample()
        # self.send_reward_to_cloudwatch(self.reward_in_episode)

        self.reward_in_episode = 0
        self.reward = None
        self.done = False
        self.next_state = None
        self.image = None
        self.steps = 0

        self.start_x = random.randint(-WORLD_BOUND_X,WORLD_BOUND_X)
        self.start_y = random.randint(-WORLD_BOUND_Y, WORLD_BOUND_Y)

        # Reset drone in Gazebo
        self.send_action(RESET_ACTION)
        self.drone_reset()

        self.infer_reward_state(RESET_ACTION)
        return self.next_state

    def drone_reset(self):
        modelState = ModelState()
        modelState.pose.position.x = self.start_x
        modelState.pose.position.y = self.start_y
        modelState.pose.position.z = self.start_z
        modelState.pose.orientation.x = 0
        modelState.pose.orientation.y = 0
        modelState.pose.orientation.z = 0
        modelState.pose.orientation.w = 0 
        modelState.twist.linear.x = 0
        modelState.twist.linear.y = 0
        modelState.twist.linear.z = 0
        modelState.twist.angular.x = 0
        modelState.twist.angular.y = 0
        modelState.twist.angular.z = 0
        modelState.model_name = 'drone'

        self.set_model_service(modelState)
        time.sleep(SLEEP_AFTER_RESET_TIME_IN_SECOND)

    def step(self, action):
        if node_type == SAGEMAKER_TRAINING_WORKER:
            return self.observation_space.sample(), 0, False, {}

        # initialize rewards, next_state, done
        self.reward = None
        self.done = False
        self.next_state = None

        self.steps += 1

        self.send_action(action)

        time.sleep(SLEEP_BETWEEN_ACTION_AND_REWARD_CALCULATION_TIME_IN_SECOND)
        self.infer_reward_state(action)

        info = {}  # additional data, not to be used for training
        return self.next_state, self.reward, self.done, info

    def callback_image(self, data):
        self.image = data

    def send_action(self, action):
        msg = RollPitchYawrateThrust()
        msg.roll = action[0]
        msg.pitch = action[1]
        msg.yaw_rate = action[2]
        msg.thrust.z = action[3]
        self.control_publisher.publish(msg)

    def exp_decay(self, x):
        return 0.1 ** (2*x)

    def reward_function(self, params):
        target_x = params['target_x']
        target_y = params['target_y']
        target_z = params['target_z']

        x = params['x']
        y = params['y']
        z = params['z']

        if abs(z - params['start_z']) <= 0.01:
            return 0

        ideal_x = params['start_x']
        ideal_y = params['start_y']
        ideal_z = WORLD_BOUND_Z / 2 # target_z

        x_bound = params['x_bound']
        y_bound = params['y_bound']
        z_bound = params['z_bound']

        max_dist_x = abs(-x_bound - abs(ideal_x))
        max_dist_y = abs(-y_bound - abs(ideal_y))
        # can only travel along positive z axis
        max_dist_z = z_bound - ideal_z

        norm_dist_x = (abs(ideal_x - x) / max_dist_x)
        norm_dist_y = (abs(ideal_y - y) / max_dist_y)
        norm_dist_z = (abs(ideal_z - z) / max_dist_z)

        axis_weight = 0.333
        factor_x = self.exp_decay(norm_dist_x) * axis_weight
        factor_y = self.exp_decay(norm_dist_y) * axis_weight
        factor_z = 1 - norm_dist_z

        reward = factor_z

        return float(reward)

    def infer_reward_state(self, action):
        # # Wait till we have a image from the camera
        # while not self.image:
        #     time.sleep(SLEEP_WAITING_FOR_IMAGE_TIME_IN_SECOND)

        # # Camera spits out BGR images by default. Converting to the
        # # image to RGB.
        # image = Image.frombytes('RGB', (self.image.width, self.image.height),
        #                         self.image.data, 'raw', 'BGR', 0, 1)
        # # resize image ans perform anti-aliasing
        # image = image.resize(TRAINING_IMAGE_SIZE, resample=2).convert("RGB")
        # state = np.array(image)

        done = False

        model_state = self.get_model_service('drone', '')
        model_orientation = Rotation.from_quat([
            model_state.pose.orientation.x,
            model_state.pose.orientation.y,
            model_state.pose.orientation.z,
            model_state.pose.orientation.w]).as_euler('zyx')
        model_x = model_state.pose.position.x
        model_y = model_state.pose.position.y
        model_z = model_state.pose.position.z

        # state = np.array([model_x, model_y, model_z, model_orientation[0] % FULL_ROTATION, model_orientation[1] % FULL_ROTATION, model_orientation[2] % FULL_ROTATION])
        state = np.array([model_z])

        # model_flipped = abs(model_orientation[1]) >= 2.0 or abs(model_orientation[2]) >= 2.0
        model_out_of_bound = (abs(model_x) >= WORLD_BOUND_X) or (abs(model_y) >= WORLD_BOUND_Y) or (abs(model_z) >= WORLD_BOUND_Z)
        model_crashed = model_out_of_bound

        reward = 0
        if model_crashed:
            done = True
        else:
            done = False
            target_state = self.get_model_service('target', '')
            params = {
                'x': model_x,
                'y': model_y,
                'z': model_z,
                'start_x': self.start_x,
                'start_y': self.start_y,
                'start_z': self.start_z,
                'roll': model_orientation[2],
                'pitch': model_orientation[1],
                'yaw': model_orientation[0],
                'target_x': target_state.pose.position.x,
                'target_y': target_state.pose.position.y,
                'target_z': target_state.pose.position.z,
                'x_bound': WORLD_BOUND_X,
                'y_bound': WORLD_BOUND_Y,
                'z_bound': WORLD_BOUND_Z,
                'action': action,
                'steps': self.steps,
            }
            reward = self.reward_function(params)
        
        if self.steps >= MAX_NUM_STEPS_PER_EPISODE:
            done = True

        self.reward_in_episode += reward
        self.reward = reward
        self.done = done
        self.next_state = state

    def send_reward_to_cloudwatch(self, reward):
        session = boto3.session.Session()
        cloudwatch_client = session.client('cloudwatch', region_name=self.aws_region)
        cloudwatch_client.put_metric_data(
            MetricData=[
                {
                    'MetricName': 'DeepRotorRewardPerEpisode',
                    'Unit': 'None',
                    'Value': reward
                },
            ],
            Namespace='AWSRoboMakerSimulation'
        )

class DeepRotorDiscreteEnv(DeepRotorEnv):
    def __init__(self):
        DeepRotorEnv.__init__(self)

        self.actions = self.create_discrete_actions()

        # self.observation_space = spaces.Box(low=np.array([-WORLD_BOUND_X, -WORLD_BOUND_Y, -WORLD_BOUND_Z, -FULL_ROTATION, -FULL_ROTATION, -FULL_ROTATION]), high=np.array([WORLD_BOUND_X, WORLD_BOUND_Y, WORLD_BOUND_Z, FULL_ROTATION, FULL_ROTATION, FULL_ROTATION]), dtype=np.float32)
        self.observation_space = spaces.Box(low=np.array([-WORLD_BOUND_Z]), high=np.array([WORLD_BOUND_Z]), dtype=np.float32)

        # actions -> index within discrete list of rotor velocity sets
        self.action_space = spaces.Discrete(len(self.actions))

    def create_discrete_actions(self):
        normal_mins = [0, 0, 0, 0]
        normal_maxs = [0, 0, 0, 1.0]
        normal_incs = [1.0, 1.0, 1.0, 0.05]  
        normal_values = [[],[],[],[]]

        for i in range(0, len(normal_values)):
            n_min = normal_mins[i]
            n_max = normal_maxs[i]
            inc = normal_incs[i]
            for j in range(0, int((n_max - n_min) / inc) + 1):
                normal_values[i].append(n_min+(j*inc))

        from itertools import product

        action_tuples = product(*normal_values)

        actions = []
        for action_tuple in action_tuples:
            action_list = list(action_tuple)
            roll = action_list[0] * MAX_ROLL
            pitch = action_list[1] * MAX_PITCH
            yaw_rate = action_list[2] * MAX_YAW_RATE
            thrust = action_list[3] * MAX_THRUST
            actions.append([roll, pitch, yaw_rate, thrust])

        return actions

    def step(self, action):

        continous_action = self.actions[action]

        return super().step(continous_action)
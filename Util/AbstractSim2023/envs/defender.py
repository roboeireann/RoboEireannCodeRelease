from re import A
import gym
import pygame
import numpy as np
import torch
import time
import sys
sys.path.append(sys.path[0] + "/..")

from gym import spaces
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize, VecMonitor
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.evaluation import evaluate_policy

from envs.base import BaseEnv

import warnings
from utils.utils import save_vec_normalize_data

warnings.filterwarnings("ignore")

LENGTH = 500
TRAINING_STEPS = 1000000
BALL_SPEED = 50


class DefenderEnv(BaseEnv):
    def __init__(self):
        super().__init__()

        """
        OBSERVATION SPACE:
            - x-cordinate of robot with respect to target
            - y-cordinate of robot with respect to target
            - sin(Angle between robot and target)
            - cos(Angle between robot and target)
        """
        observation_space_size = 12

        observation_space_low = -1 * np.ones(observation_space_size)
        observation_space_high = np.ones(observation_space_size)
        self.observation_space = gym.spaces.Box(
            observation_space_low, observation_space_high
        )

        self.reset()



    def move_robot(self, action):
            # Find location policy is trying to reach
        policy_target_x = self.robot_x + (
            (
                (np.cos(self.robot_angle) * np.clip(action[1], -1, 1))
                + (np.cos(self.robot_angle + np.pi / 2) * np.clip(action[2], -1, 1))
            )
            * 200
        )  # the x component of the location targeted by the high level action
        policy_target_y = self.robot_y + (
            (
                (np.sin(self.robot_angle) * np.clip(action[1], -1, 1))
                + (np.sin(self.robot_angle + np.pi / 2) * np.clip(action[2], -1, 1))
            )
            * 200
        )  # the y component of the location targeted by the high level action

        # Update robot position
        self.robot_x = (
            self.robot_x * (1 - self.displacement_coef)
            + policy_target_x * self.displacement_coef
        )  # weighted sums based on displacement coefficient
        self.robot_y = (
            self.robot_y * (1 - self.displacement_coef)
            + policy_target_y * self.displacement_coef
        )  # the idea is we move towards the target position and angle

        self.position_rule()

        # Find distance between robot and target
        robot_location = np.array([self.robot_x, self.robot_y])
        target_location = np.array([self.target_x, self.target_y])
        distance_robot_target = np.linalg.norm(target_location - robot_location)



        self.ball_x_offset = self.target_x - self.ball_target_x
        self.ball_y_offset = self.target_y - self.ball_target_y

        direction_vector = np.array([self.ball_x_offset, self.ball_y_offset])
        ball_target_location = np.array([self.ball_target_x, self.ball_target_y])

        distance_ball_ball_target = np.linalg.norm(target_location - ball_target_location)

        self.ball_x_displacement = self.ball_x_offset / distance_ball_ball_target
        self.ball_y_displacement = self.ball_y_offset / distance_ball_ball_target

        self.target_x = self.target_x - self.ball_x_displacement * BALL_SPEED
        self.target_y = self.target_y - self.ball_y_displacement * BALL_SPEED




        # push ball, if collision with robot
        if distance_robot_target < (self.robot_radius + self.target_radius) * 5:
            # changed to reached reward mode
            self.contacted_ball = True

            self.target_x = np.random.uniform(-1000,1000)
            self.target_y = np.random.uniform(-3000,3000)
            self.ball_target_y = np.random.uniform(-300,300)
        else:
            self.contacted_ball = False




        #check if robot in defense areas

        if self.robot_x >= -4500 and self.robot_x <= 0 and self.robot_y >= -3000 and self.robot_y <= 3000 :
            self.in_defense_area = True
        else:
            self.in_defense_area = False

        # Turn towards the target as suggested by the policy
        self.robot_angle = self.robot_angle + self.displacement_coef * action[0] 

        # reset ball location if it reaches the goal
        if self.target_x <= -4500 and self.target_y >= -750 and self.target_y <= 750:
            self.reached_goal = True
            self.target_x = np.random.uniform(-1000,1000)
            self.target_y = np.random.uniform(-3000,3000)
            self.ball_target_y = np.random.uniform(-300,300)
        else:
            self.reached_goal = False


    def calculate_reward(self):
        reward = 0
        if not self.check_facing_ball():
            reward += -.01
        if  self.contacted_ball:
            reward += 1
        if self.reached_goal:
            reward -=1
        if self.robot_x>-1500:
            reward -=.02

        return reward



    def reset(self):
        self.time = 0

        self.displacement_coef = 0.2

        self.contacted_ball = False

        self.robot_x = -2250
        self.robot_y = 0
        self.robot_angle = 0

        self.target_x = np.random.uniform(-1000,1000)
        self.target_y = np.random.uniform(-3000,3000)

        self.ball_target_x = -4510
        self.ball_target_y = np.random.uniform(-300,300)

        self.goal_x = 4800
        self.goal_y = 0

        self.dummy1_x = (self.target_x + self.goal_x) / 2
        self.dummy1_y = (self.target_y + self.goal_y) / 2

        self.dummy2_x = (self.target_x + self.robot_x) / 2
        self.dummy2_y = (self.target_y + self.robot_y) / 2

        self.update_goal_value()

        robot_location = np.array([self.robot_x, self.robot_y])
        target_location = np.array([self.target_x, self.target_y])
        self.initial_distance = np.linalg.norm(target_location - robot_location)

        return self._observe_state()


    def _observe_state(self):

        self.update_target_value()
        self.update_goal_value()

        return np.array(
            [
                (self.dummy1_x - self.robot_x) / 9000,
                (self.dummy1_y - self.robot_y) / 6000,
                (self.dummy2_x - self.robot_x) / 9000,
                (self.dummy2_y - self.robot_y) / 6000,
                (self.target_x - self.robot_x) / 9000,
                (self.target_y - self.robot_y) / 6000,
                (self.goal_x - self.target_x) / 9000,
                (self.goal_y - self.target_y) / 6000,
                np.sin(self.relative_angle - self.robot_angle),
                np.cos(self.relative_angle - self.robot_angle),
                np.sin(self.goal_relative_angle - self.robot_angle),
                np.cos(self.goal_relative_angle - self.robot_angle),
            ]
        )

    def _observe_global_state(self):
        return [
            self.robot_x / 9000,
            self.robot_y / 6000,
            self.target_x / 9000,
            self.target_y / 6000,
            self.dummy1_x / 9000,
            self.dummy1_y / 6000,
            self.dummy2_x / 9000,
            self.dummy2_y / 6000,
            np.sin(self.robot_angle),
            np.cos(self.robot_angle),
        ]

    def set_abstract_state(self, abstract_state):

        self.robot_x = abstract_state[0] * 9000
        self.robot_y = abstract_state[1] * 6000
        self.target_x = abstract_state[2] * 9000
        self.target_y = abstract_state[3] * 6000

        self.dummy1_x = abstract_state[4] * 9000
        self.dummy1_y = abstract_state[5] * 6000
        self.dummy2_x = abstract_state[6] * 9000
        self.dummy2_y = abstract_state[7] * 6000

        self.robot_angle = np.arctan2(abstract_state[8], abstract_state[9])



if __name__ == "__main__":
    if not len(sys.argv) == 2:
        print("usage: python ./push_ball_to_goal.py <policy and vector path folder>")
        exit(1)

    policy_path = sys.argv[1] + "/policy.zip"
    normalization_path = sys.argv[1] + "/vector_normalize"

    env = VecNormalize.load(
        normalization_path, make_vec_env(DefenderEnv, n_envs=1)
    )
    env.norm_obs = True
    env.norm_reward = False
    env.clip_obs = 1.0
    env.training = False

    # #derived from https://github.com/DLR-RM/rl-baselines3-zoo/blob/75afd65fa4a1f66814777d43bd14e4bba18d96db/enjoy.py#L171
    #     newer_python_version = sys.version_info.major == 3 and sys.version_info.minor >= 8

    custom_objects = {
    "lr_schedule": lambda x: .003,
    "clip_range": lambda x: .02
    }   
    model = PPO.load(policy_path, custom_objects = custom_objects, env= env)

   
    it = 0

    obs = env.reset()

    while True:
        action = model.predict(obs, deterministic = True)
        obs, reward, done, _ = env.step(action[0])
        env.envs[0].render()
        it = it + 1

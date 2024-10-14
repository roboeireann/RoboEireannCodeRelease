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

LENGTH = 100
TRAINING_STEPS = 500000


class WalkToPointEnv(BaseEnv):

    EPISODE_LENGTH = LENGTH

    def __init__(self):
        super().__init__()

        """
        OBSERVATION SPACE:
            - x-cordinate of robot with respect to target
            - y-cordinate of robot with respect to target
            - sin(Angle between robot and target)
            - cos(Angle between robot and target)
        """
        observation_space_low = -1 * np.ones(12)
        observation_space_high = np.ones(12)
        self.observation_space = gym.spaces.Box(
            observation_space_low, observation_space_high
        )

        self.reset()

    def reset(self):
        self.time = 0

        self.displacement_coef = 0.2

        self.contacted_ball = False

        self.robot_x = np.random.uniform(-4500, 4500)
        self.robot_y = np.random.uniform(-3000, 3000)
        self.robot_angle = np.random.uniform(0, 2 * np.pi)

        self.target_x = np.random.uniform(-4500, 4500)
        self.target_y = np.random.uniform(-3000, 3000)

        self.goal_x = 4500
        self.goal_y = 0

        self.update_goal_value()
        self.update_target_value()

        robot_location = np.array([self.robot_x, self.robot_y])
        target_location = np.array([self.target_x, self.target_y])
        self.initial_distance = np.linalg.norm(target_location - robot_location)

        return self._observe_state()


    def _observe_state(self):
        return np.array(
            [
                0,
                0,
                0,
                0,
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

    def calculate_reward(self):
        # Find distance between robot and target
        robot_location = np.array([self.robot_x, self.robot_y])
        target_location = np.array([self.target_x, self.target_y])
        distance_robot_target = np.linalg.norm(target_location - robot_location)

        reward =  min(1 / distance_robot_target,1)
        if self.check_facing_down_field() and  distance_robot_target < 400:
            reward += 10
        return reward

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

        # push ball, if collision with robot
        if distance_robot_target < (self.robot_radius + self.target_radius) * 5:
            # changed to reached reward mode
            self.contacted_ball = True

            # Update Relative Angle
            self.delta_x = self.target_x - self.robot_x
            self.delta_y = self.target_y - self.robot_y
            self.theta_radians = np.arctan2(self.delta_y, self.delta_x)

          
                # Check if line of ball intersects goal line
            A = self.Point(self.target_x, self.target_y)
            B = self.Point(self.robot_x, self.robot_y)
            C = self.Point(-4500, -750)
            D = self.Point(-4500, 750)

            # Put ball at goal location (-4500, 0)
            if self.intersect(A, B, C, D):
                self.target_x = -4501
                self.target_y = 0

        # Turn towards the target as suggested by the policy
        self.robot_angle = self.robot_angle + self.displacement_coef * action[0]



if __name__ == "__main__":
    if not len(sys.argv) == 2:
        print("usage: python ./walk_to_ball.py <policy and vector path folder>")
        exit(1)

    policy_path = sys.argv[1] + "/policy.zip"
    normalization_path = sys.argv[1] + "/vector_normalize"

    env = VecNormalize.load(
        normalization_path, make_vec_env(WalkToPointEnv, n_envs=12)
    )
    env.norm_obs = True
    env.norm_reward = True
    env.clip_obs = 1.0
    env.training = True

    # #derived from https://github.com/DLR-RM/rl-baselines3-zoo/blob/75afd65fa4a1f66814777d43bd14e4bba18d96db/enjoy.py#L171
    #     newer_python_version = sys.version_info.major == 3 and sys.version_info.minor >= 8

    custom_objects = {
    "lr_schedule": lambda x: .003,
    "clip_range": lambda x: .02
    }   
    model = PPO.load(policy_path, custom_objects = custom_objects, env= env)

    mean_reward, std_reward = evaluate_policy(
        model, model.get_env(), n_eval_episodes=10
    )
    print(mean_reward, std_reward)

    it = 0

    obs = env.reset()

    while True:
        action = model.predict(obs, deterministic = True)
        obs, reward, done, _ = env.step(action[0])
        env.envs[0].render()
        it = it + 1

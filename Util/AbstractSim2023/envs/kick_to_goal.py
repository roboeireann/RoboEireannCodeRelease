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


class KickToGoalEnv(BaseEnv):
    def __init__(self):
        super().__init__()

        action_space_low = np.array([-np.pi / 2, -1, -1, 0])
        action_space_high = np.array([np.pi / 2, 1, 1, 1])
        self.action_space = gym.spaces.Box(action_space_low, action_space_high)

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

    def reset(self):
        self.time = 0

        self.displacement_coef = 0.2

        self.contacted_ball = False

        self.robot_x = np.random.uniform(-3500, 3500)
        self.robot_y = np.random.uniform(-2500, 2500)
        self.robot_angle = np.random.uniform(0, 2 * np.pi)

        self.target_x = np.random.uniform(-2500, 2500)
        self.target_y = np.random.uniform(-2000, 2000)

        self.goal_x = -4500
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

    def step(self, action):
        self.time += 1

        # Attempt to kick ball
        if action[3] > 0.95:
            self.kick_ball()       
        # If not kicking, move robot
        else:
           self.move_robot(action)

        self.update_target_value()
        self.update_goal_value()

        done = self.time > BaseEnv.EPISODE_LENGTH

        reward = self.calculate_reward()

        new_obs = self._observe_state()

        return (new_obs, reward, done, {})

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
        normalization_path, make_vec_env(KickToGoalEnv, n_envs=12)
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
        action = model.predict(obs)
        obs, reward, done, _ = env.step(action[0])
        env.envs[0].render()
        it = it + 1

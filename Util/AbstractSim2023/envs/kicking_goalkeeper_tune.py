from re import A
import gym
import pygame
import numpy as np
import torch
import random
import time
import os
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


class KickingGoalKeeperTuneEnv(BaseEnv):
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

        action_space_low = np.array([-np.pi / 2, -1, -1,0])
        action_space_high = np.array([np.pi / 2, 1, 1,1])
        self.action_space = gym.spaces.Box(action_space_low, action_space_high)

        self.def_positions_list = [
            [-3500, 0],
            [-3500, 1000],
            [-3500, -1000],
            [-2900, 0],
            [-2900, 1000],
            [-2900, -1000],
            [-2300, 0],
            [-2300, 1000],
            [-2300, -1000],
        ]


        self.reward_init()
        self.reset()

    # create a reward function and writ it to a text file
    def reward_init(self):
        if os.path.exists('goalie_reward.py'):
            os.remove('goalie_reward.py')

        # return reward
        with open('goalie_reward.py', 'w') as f:
            f.write("distanceToGoal = ((self.target_x - (-self.goal_x))**2 + (self.target_y - self.goal_y)**2)**0.5\n")
            f.write("if self.check_facing_ball():\n")
            f.write("    reward += 0.2\n")
            f.write("if self.check_if_blocking_goal():\n")
            f.write("    reward += 0.7\n")
            f.write("if self.kicked:\n")
            f.write("    reward -=.001\n")
            f.write("    if self.contacted_ball:\n")
            f.write("        reward += 0.5\n")
            f.write("    if self.defender_kicking:\n")
            f.write("        reward -= 0.1\n")  
            f.write("if self.target_x < self.robot_x: # ball has gone past the goalie\n")   # ball has gone past the goalie
            f.write("    reward -= 0.4\n")
            f.write("if self.robot_x < -4300:\n")
            f.write("    reward -= 0.02\n")
            f.write("if self.robot_x < -4500:\n")
            f.write("    reward -= 100\n")
            f.write("if distanceToGoal > 500:\n")
            f.write("    reward -= 0.3\n")
            f.write("reward -= (0.2/(self.goalpost_1_distance+.000000001))\n")
            f.write("reward -= (0.2/(self.goalpost_2_distance+.000000001))\n")
            f.write("# Regularization - discourage large changes in reward\n")
            f.write("reward -= 0.02 * abs(reward)\n")
            f.write("# Ensure final reward stays within range\n")
            f.write("reward = max(-1, min(1, reward))\n")


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


        #check if robot in goal areas

        if policy_target_x >= -4500 and policy_target_x <= -3900 and policy_target_y >= -600 and policy_target_y <= 600 :
            self.in_goal_area = True
                # Update robot position
            self.robot_x = (
                self.robot_x * (1 - self.displacement_coef)
                + policy_target_x * self.displacement_coef
            )  # weighted sums based on displacement coefficient
            self.robot_y = (
                self.robot_y * (1 - self.displacement_coef)
                + policy_target_y * self.displacement_coef
            )  # the idea is we move towards the target position and angle
        else:
            self.in_goal_area = False


      

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


        self.kicked = (action[3] > 0.7)

        distanceDummy1Target = np.linalg.norm(np.array([self.dummy1_x, self.dummy1_y]) - np.array([self.target_x, self.target_y]))
        distanceDummy2Target = np.linalg.norm(np.array([self.dummy2_x, self.dummy2_y]) - np.array([self.target_x, self.target_y]))
        self.defender_kicking = False
        if distanceDummy1Target < 500 or distanceDummy2Target < 500:
            self.defender_kicking = True


        # push ball, if collision with robot
        if distance_robot_target < (self.robot_radius + self.target_radius) * 20 and self.kicked:
            # changed to reached reward mode
            self.contacted_ball = True

            self.target_x = np.random.uniform(-4000,0)
            self.target_y = np.random.uniform(-3000,3000)
            self.ball_target_y = np.random.uniform(-300,300)
        else:
            self.contacted_ball = False





        self.goalpost_1_distance = np.linalg.norm(np.array([self.robot_x, self.robot_y]) - np.array([-4500,600])) #conservative goalpost estimates, real ones wider apart
        self.goalpost_2_distance = np.linalg.norm(np.array([self.robot_x, self.robot_y]) - np.array([-4500,-600]))


        # Turn towards the target as suggested by the policy
        self.robot_angle = self.robot_angle + self.displacement_coef * action[0] 

        # reset ball location if it reaches the goal
        if self.target_x <= -4500 and self.target_y >= -750 and self.target_y <= 750:
            self.target_x = np.random.uniform(-4000,0)
            self.target_y = np.random.uniform(-3000,3000)
            self.ball_target_y = np.random.uniform(-300,300)

    def check_if_blocking_goal(self):
        A = self.target_y - self.ball_target_y
        B = self.ball_target_x - self.target_x
        C = self.target_x * self.ball_target_x - self.ball_target_y * self.target_y
        
        # distance from point to line
        # ax + by + c / sqrt(a^2 + b^2)
        distance = abs(A * self.robot_x + B * self.robot_y + C) / (A**2 + B**2)**0.5

        if distance < 50:
            return True
        return False
    
    def calculate_reward(self):
        # return reward
        local = {
            "self": self,
            "reward": 0,
        }
        with open('goalie_reward.py', 'r') as f:
            exec(f.read(), globals(), local)
        return local['reward']



    def reset(self):
        self.time = 0

        self.displacement_coef = 0.2

        self.contacted_ball = False

        self.base_pos1 = random.choice(self.base_positions_list)
        self.base_pos2 = random.choice(self.base_positions_list)
        while (self.base_pos1 == self.base_pos2):
            self.base_pos2 = random.choice(self.base_positions_list)

        self.robot_x = -4500
        self.robot_y = 0
        self.robot_angle = 0

        self.target_x = np.random.uniform(-4000,0)
        self.target_y = np.random.uniform(-3000,3000)


        self.ball_target_x = -4510
        self.ball_target_y = np.random.uniform(-1000,1000)

        self.goal_x = 4500
        self.goal_y = 0

        self.dummy1_x = self.base_pos1[0]
        self.dummy1_y = self.base_pos1[1]

        self.dummy2_x = self.base_pos2[0]
        self.dummy2_y = self.base_pos2[1]

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
        normalization_path, make_vec_env(KickingGoalKeeperTuneEnv, n_envs=1)
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

from re import A
import gym
import pygame
import numpy as np
import torch
import time
import random
import sys
from dotenv import load_dotenv
import os
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
load_dotenv()

LENGTH = 500
TRAINING_STEPS = 1000000
BALL_SPEED = random.uniform(0,20)


class KickingDefenderTuneEnv(BaseEnv):
    def __init__(self):
        super().__init__()

        """
        OBSERVATION SPACE:
            - x-cordinate of robot with respect to target
            - y-cordinate of robot with respect to target
            - sin(Angle between robot and target)
            - cos(Angle between robot and target)
        """
        observation_space_size = 14

        observation_space_low = -1 * np.ones(observation_space_size)
        observation_space_high = np.ones(observation_space_size)
        self.observation_space = gym.spaces.Box(
            observation_space_low, observation_space_high
        )

        action_space_low = np.array([-np.pi / 2, -1, -1,-1])
        action_space_high = np.array([np.pi / 2, 1, 1,1])
        self.action_space = gym.spaces.Box(action_space_low, action_space_high)

        self.base_positions_list = [
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
        if os.path.exists('reward.py'):
            os.remove('reward.py')

        with open('reward.py', 'w') as f:
            f.write("distanceBallGoal = ((self.target_x - self.ball_target_x)**2 + (self.target_y - self.ball_target_y)**2)**0.5\n")
            f.write("distanceToBase = ((self.robot_base_x - self.robot_x)**2 + (self.robot_base_y - self.robot_y)**2)**0.5\n")
            f.write("if self.check_facing_ball():\n")
            f.write("    reward += .03\n")
            f.write("if self.robot_x > self.target_x:\n")
            f.write("    reward -= .03\n")
            f.write("if self.pressing:\n")
            f.write("    if self.kicked and self.contacted_ball:\n")
            f.write("        reward += .6\n")
            f.write("    elif self.kicked:\n")
            f.write("        reward -= .01\n")
            f.write("    if distanceToBase < 500:\n")
            f.write("        reward += 10/(distanceToBase+0.0001)\n")
            f.write("    if distanceToBase > 1000:\n")
            f.write("        reward -= (0.001*distanceToBase)\n")
            f.write("else:\n")
            f.write("    if self.kicked and self.contacted_ball:\n")
            f.write("        reward += .2\n")
            f.write("    elif self.kicked:\n")
            f.write("        reward -= .01\n")
            f.write("    if distanceToBase < 500:\n")
            f.write("        reward += 10/(distanceToBase+0.0001)\n")
            f.write("    if distanceToBase > 1000:\n")
            f.write("        reward -= (0.001*distanceToBase)\n")
            f.write("reward -= 0.02 * abs(reward)\n")
            f.write("reward = max(-1, min(1, reward))\n")


    def move_robot(self, action):
        attackerAction = (self.attackerModel.predict(self.attacker_observation(), deterministic = True))[0]

        distanceSelfBall = ((self.robot_x - self.target_x) ** 2 + (self.robot_y - self.target_y) ** 2) ** 0.5
        distanceTeammateBall = ((self.dummy1_x - self.target_x) ** 2 + (self.dummy1_y - self.target_y) ** 2) ** 0.5

        self.pressing = False
        if distanceSelfBall < distanceTeammateBall:
            self.pressing = True

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


        # moving the ball
        self.ball_x_offset = self.target_x - self.ball_target_x
        self.ball_y_offset = self.target_y - self.ball_target_y

        ball_target_location = np.array([self.ball_target_x, self.ball_target_y])

        distance_ball_ball_target = np.linalg.norm(target_location - ball_target_location)

        self.ball_x_displacement = self.ball_x_offset / distance_ball_ball_target
        self.ball_y_displacement = self.ball_y_offset / distance_ball_ball_target
        # move the attacker and ball
        attackerPolicyX = self.attacker_x + (
            (
                (np.cos(self.attacker_angle) * np.clip(attackerAction[1], -1, 1))
                + (np.cos(self.attacker_angle + np.pi / 2) * np.clip(attackerAction[2], -1, 1))
            )
            * 200
        )
        attackerPolicyY = self.attacker_y + (
            (
                (np.sin(self.attacker_angle) * np.clip(attackerAction[1], -1, 1))
                + (np.sin(self.attacker_angle + np.pi / 2) * np.clip(attackerAction[2], -1, 1))
            )
            * 200
        )
        self.attacker_x = (
            self.attacker_x * (1 - self.displacement_coef)
            + attackerPolicyX * self.displacement_coef
        )  # weighted sums based on displacement coefficient
        self.attacker_y = (
            self.attacker_y * (1 - self.displacement_coef)
            + attackerPolicyY * self.displacement_coef
        )  # the idea is we move towards the target position and angle

        self.position_rule()
        # move the ball if the attacker comes into contact with it


        self.kicked = (action[3] > 0)

        # push ball, if collision with robot
        if distance_robot_target < (self.robot_radius + self.target_radius) * 20 and self.kicked:
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
        self.attacker_angle = self.attacker_angle + self.displacement_coef * attackerAction[0]

        # reset ball location if it reaches the goal
        if self.target_x <= -4500 and self.target_y >= -750 and self.target_y <= 750:
            self.reached_goal = True
            self.target_x = np.random.uniform(-1000,1000)
            self.target_y = np.random.uniform(-3000,3000)
            self.ball_target_y = np.random.uniform(-300,300)
        else:
            self.reached_goal = False


    def calculate_reward(self):
        local = {
            "self": self,
            "reward": 0,
        }

        with open('reward.py', 'r') as f:
            exec(f.read(), globals(), local)
        return local['reward'] 

        
    def reset(self):
        self.time = 0

        self.displacement_coef = 0.2

        self.contacted_ball = False

        self.ball_move = False

        self.base_pos = random.choice(self.base_positions_list)

        self.robot_base_x = self.base_pos[0]
        self.robot_base_y = self.base_pos[1]

        self.robot_x = self.robot_base_x
        self.robot_y = self.robot_base_y
        self.robot_angle = 0

        self.target_x = np.random.uniform(-1000,1000)
        self.target_y = np.random.uniform(-3000,3000)

        self.ball_target_x = -4510
        self.ball_target_y = np.random.uniform(-300,300)

        self.goal_x = 4800
        self.goal_y = 0

        otherBase = random.choice(self.base_positions_list)
        if otherBase == self.base_pos:
            otherBase = random.choice(self.base_positions_list)
        self.dummy1_x = otherBase[0]
        self.dummy1_y = otherBase[1]

        self.dummy2_x = (self.target_x + self.robot_x) / 2
        self.dummy2_y = (self.target_y + self.robot_y) / 2

        self.attacker_x = self.target_x + 100
        self.attacker_y = self.target_y + 100
        self.attacker_angle = -np.pi

        self.attackerPath = './Models/first_test_push_to_goal/policy'
        self.attackerModel = PPO.load(self.attackerPath)

        self.update_goal_value()
        self.update_att_goal_value()

        robot_location = np.array([self.robot_x, self.robot_y])
        target_location = np.array([self.target_x, self.target_y])
        self.initial_distance = np.linalg.norm(target_location - robot_location)

        return self.noisy_observation()


    def _observe_state(self):

        self.update_target_value()
        self.update_goal_value()    
        
        return np.array(
            [
                (self.robot_base_x - self.robot_x) / 9000,
                (self.robot_base_y - self.robot_y) / 6000,
                (self.dummy1_x - self.robot_x) / 9000,
                (self.dummy1_y - self.robot_y) / 6000,
                (self.attacker_x - self.robot_x) / 9000,
                (self.attacker_y - self.robot_y) / 6000,
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

    def noisy_observation(self):
        observation = self._observe_state()
        noise = np.random.normal(0, 0.1, observation.shape)
        observation = observation + noise
        return observation

    def step(self, actions):

        self.maxReward = float(os.getenv('MAX_REWARD'))
        self.blockingReward = float(os.getenv('BLOCKING_REWARD'))
        self.facingBallReward = float(os.getenv('FACING_BALL_REWARD')) 

        self.time += 1

        self.move_robot(actions)

        self.update_target_value()
        self.update_goal_value()
        self.update_att_target_value()
        self.update_att_goal_value()

        done = self.time > BaseEnv.EPISODE_LENGTH

        if self.ball_move == False:
            # enable robot collision with dummy defenders
            self.att_collision_ball()
            self.position_rule()

            # Turn towards the target as suggested by the policy
            self.robot_angle = self.robot_angle + self.displacement_coef * actions[0]

            self.update_target_value()
            self.update_goal_value()
            self.update_att_target_value()
            self.update_att_goal_value()

        # by goal kick, goalie,attacker get little pause during ball moving.
        elif self.ball_move == True:
            self.ball_move = False

            self.speed = [self.ball_go_x, self.ball_go_y]
            self.rect2 = self.rect2.move(self.speed)
            self.target_x = self.rect2.centerx
            self.target_y = self.rect2.centery

        reward = self.calculate_reward()

        new_obs = self.noisy_observation()

        return (new_obs, reward, done, {})

    def render(self, mode="display"):
        Field_length = 1200
        time.sleep(0.01)

        if self.rendering_init == False:
            pygame.init()

            self.field = pygame.display.set_mode((Field_length, Field_length * (2 / 3)))

            self.basic_field(Field_length)
            pygame.display.set_caption("Point Targeting Environment")
            self.clock = pygame.time.Clock()

            self.rendering_init = True

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                exit()

        self.basic_field(Field_length)

        render_robot_x = (self.robot_x / 5200 + 1) * (Field_length / 2)
        render_robot_y = (self.robot_y / 3700 + 1) * (Field_length / 3)

        render_attacker_x = (self.attacker_x / 5200 + 1) * (Field_length / 2)
        render_attacker_y = (self.attacker_y / 3700 + 1) * (Field_length / 3)

        render_target_x = (self.target_x / 5200 + 1) * (Field_length / 2)
        render_target_y = (self.target_y / 3700 + 1) * (Field_length / 3)

        render_base_x = (self.robot_base_x / 5200 + 1) * (Field_length / 2)
        render_base_y = (self.robot_base_y / 3700 + 1) * (Field_length / 3)

        render_dummy1_x = (self.dummy1_x / 5200 + 1) * (Field_length / 2)
        render_dummy1_y = (self.dummy1_y / 3700 + 1) * (Field_length / 3)

        pygame.draw.circle(
            self.field,
            pygame.Color(40, 40, 40),
            (render_target_x, render_target_y),
            self.target_radius,
        )

        pygame.draw.circle(
            self.field,
            pygame.Color(148, 17, 0),
            (render_robot_x, render_robot_y),
            self.robot_radius,
            width=5,
        )

        pygame.draw.circle(
            self.field,
            pygame.Color(148, 17, 0),
            (render_dummy1_x, render_dummy1_y),
            self.robot_radius,
            width=5,
        )

        pygame.draw.circle(
            self.field,
            pygame.Color(0, 0, 200),
            (render_attacker_x, render_attacker_y),
            self.robot_radius,
            width=5,
        )

        pygame.draw.line(
            self.field,
            pygame.Color(40, 40, 40),
            (render_robot_x, render_robot_y),
            (
                render_robot_x + self.robot_radius * np.cos(self.robot_angle),
                render_robot_y + self.robot_radius * np.sin(self.robot_angle),
            ),
            width=5,
        )

        pygame.draw.line(
            self.field,
            pygame.Color(40, 40, 40),
            (render_attacker_x, render_attacker_y),
            (
                render_attacker_x + self.robot_radius * np.cos(self.attacker_angle),
                render_attacker_y + self.robot_radius * np.sin(self.attacker_angle),
            ),
            width=5,
        )
        # draw field of view
        pygame.draw.line(
            self.field,
            pygame.Color(155, 155, 155),
            (render_robot_x, render_robot_y),
            (
                render_robot_x + (5000 * 1200 / 9000) * np.cos(self.robot_angle - np.pi / 3),
                render_robot_y + (5000 * 1200 / 9000) * np.sin(self.robot_angle - np.pi / 3),
            ),
            width=2,
        )
        pygame.draw.line(
            self.field,
            pygame.Color(155, 155, 155),
            (render_robot_x, render_robot_y),
            (
                render_robot_x + (5000 * 1200 / 9000) * np.cos(self.robot_angle + np.pi / 3),
                render_robot_y + (5000 * 1200 / 9000) * np.sin(self.robot_angle + np.pi / 3),
            ),
            width=2,
        )
        pygame.draw.circle(
            self.field,
            pygame.Color(40, 40, 40),
            (render_base_x, render_base_y),
            self.robot_radius,
            width=5,
        )


        pygame.display.update()
        self.clock.tick(60)

    def attacker_observation(self):
        self.update_att_target_value()
        self.update_att_goal_value()
        return np.array(
            [
                (self.robot_x - self.attacker_x) / 9000,
                (self.robot_y - self.attacker_y) / 6000,
                (self.dummy1_x - self.attacker_x) / 9000,
                (self.dummy1_y - self.attacker_y) / 6000,
                (self.target_x - self.attacker_x) / 9000,
                (self.target_y - self.attacker_y) / 6000,
                (self.ball_target_x - self.target_x) / 9000,
                (self.ball_target_y - self.target_y) / 6000,
                np.sin(self.att_relative_angle - self.attacker_angle),
                np.cos(self.att_relative_angle - self.attacker_angle),
                np.sin(self.goal_att_relative_angle - self.attacker_angle),
                np.cos(self.goal_att_relative_angle - self.attacker_angle),
            ]
        )

    def update_att_target_value(self):
        # Update Relative Angle
        self.att_delta_x = self.target_x - self.attacker_x
        self.att_delta_y = self.target_y - self.attacker_y
        self.att_theta_radians = np.arctan2(self.att_delta_y, self.att_delta_x)

        if self.att_theta_radians >= 0:
            self.att_relative_angle = self.att_theta_radians
        else:
            self.att_relative_angle = self.att_theta_radians + 2 * np.pi

    def update_att_goal_value(self):
        # Update Relative Angle to goal
        self.goal_att_delta_x = self.ball_target_x - self.attacker_x
        self.goal_att_delta_y = self.ball_target_y - self.attacker_y
        self.goal_att_theta_radians = np.arctan2(self.goal_att_delta_y, self.goal_att_delta_x)

        if self.goal_att_theta_radians >= 0:
            self.goal_att_relative_angle = self.goal_att_theta_radians
        else:
            self.goal_att_relative_angle = self.goal_att_theta_radians + 2 * np.pi

    def att_collision_ball(self):
        # Find distance between robot and target
        robot_location = np.array([self.attacker_x, self.attacker_y])
        target_location = np.array([self.target_x, self.target_y])
        distance_robot_target = np.linalg.norm(target_location - robot_location)

        # push ball, if collision with robot
        if distance_robot_target < (self.robot_radius + self.target_radius) * 5:
            # changed to reached reward mode
            self.att_contacted_ball = True

            # Update Relative Angle
            self.att_delta_x = self.target_x - self.attacker_x
            self.att_delta_y = self.target_y - self.attacker_y
            self.att_theta_radians = np.arctan2(self.att_delta_y, self.att_delta_x)
            self.target_x = (
                self.target_x
                + np.cos(self.att_theta_radians)
                * (self.robot_radius + self.target_radius)
                * 5
            )
            self.target_y = (
                self.target_y
                + np.sin(self.att_theta_radians)
                * (self.robot_radius + self.target_radius)
                * 5
            )
    
    def check_if_blocking_goal(self):
        A = self.target_y - self.ball_target_y
        B = self.ball_target_x - self.target_x
        C = self.target_x * self.ball_target_x - self.ball_target_y * self.target_y
        
        # distance from point to line
        # ax + by + c / sqrt(a^2 + b^2)
        distance = abs(A * self.robot_x + B * self.robot_y + C) / (A**2 + B**2)**0.5

        if distance < 100:
            return True
        return False


if __name__ == "__main__":
    if not len(sys.argv) == 2:
        print("usage: python ./push_ball_to_goal.py <policy and vector path folder>")
        exit(1)

    policy_path = sys.argv[1] + "/policy.zip"
    normalization_path = sys.argv[1] + "/vector_normalize"

    os.environ["RLHF"] = "disabled"

    env = VecNormalize.load(
        normalization_path, make_vec_env(KickingDefenderTuneEnv, n_envs=1)
    )
    env.norm_obs = True
    env.norm_reward = False
    env.clip_obs = 1.0
    env.training = False

    # #derived from https://github.com/DLR-RM/rl-baselines3-zoo/blob/75afd65fa4a1f66814777d43bd14e4bba18d96db/enjoy.py#L171
    #     newer_python_version = sys.version_info.major == 3 and sys.version_info.minor >= 8

    custom_objects = {
    "lr_schedule": lambda x: .003,
    "clip_range": lambda x: .02,
    }   
    model = PPO.load(policy_path, custom_objects = custom_objects, env= env)

   
    it = 0

    obs = env.reset()

    while True:
        action = model.predict(obs, deterministic = True)
        obs, reward, done, _ = env.step(action[0])
        env.envs[0].render()
        it = it + 1

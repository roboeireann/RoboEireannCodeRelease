from cmath import rect
import gym
import pygame
import numpy as np
import torch
import time
import sys
from pygame.rect import *
sys.path.append(sys.path[0] + "/..")

from gym import spaces
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize, VecMonitor
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.evaluation import evaluate_policy

from envs.base import BaseEnv

import warnings
from utils.utils import save_vec_normalize_data

LENGTH = 500
TRAINING_STEPS = 2000000


class GoalieEnv(BaseEnv):

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
        self.defender_collision = False
    
        self.move_robot(action)

        self.update_target_value()
        self.update_goal_value()

        if self.ball_move == False:

            self.goalie_logic()
            # enable robot collision with dummy defenders
            self.collision_dummy(self.dummy1_x, self.dummy1_y)
            self.collision_ball()
            self.position_rule()

            # Turn towards the target as suggested by the policy
            self.robot_angle = self.robot_angle + self.displacement_coef * action[0]

            self.update_target_value()
            self.update_goal_value()

            self.goalie_logic()

        # by goal kick, goalie,attacker get little puase during ball moving.
        elif self.ball_move == True:
            self.ball_move = False

            self.speed = [self.ball_go_x, self.ball_go_y]
            self.rect2 = self.rect2.move(self.speed)
            self.target_x = self.rect2.centerx
            self.target_y = self.rect2.centery

        new_obs = self._observe_state()
        # done = (self.time > PushBallToGoalEnv.EPISODE_LENGTH) or distance_robot_target < self.target_radius
        done = self.time > GoalieEnv.EPISODE_LENGTH

        reward = self.calculate_goalie_reward()

        return (new_obs, reward, done, {})

    def render(self, mode="display"):
        time.sleep(0.01)

        # your screen size for pygame
        Field_length = 1200

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

        render_target_x = (self.target_x / 5200 + 1) * (Field_length / 2)
        render_target_y = (self.target_y / 3700 + 1) * (Field_length / 3)

        render_dummy1_x = (self.dummy1_x / 5200 + 1) * (Field_length / 2)
        render_dummy1_y = (self.dummy1_y / 3700 + 1) * (Field_length / 3)

        render_dummy2_x = (self.dummy2_x / 5200 + 1) * (Field_length / 2)
        render_dummy2_y = (self.dummy2_y / 3700 + 1) * (Field_length / 3)

        pygame.draw.circle(
            self.field,
            pygame.Color(40, 40, 40),
            (render_target_x, render_target_y),
            self.target_radius,
        )
        pygame.draw.circle(
            self.field,
            pygame.Color(0, 32, 96),
            (render_dummy1_x, render_dummy1_y),
            self.robot_radius,
            width=4,
        )

        pygame.draw.circle(
            self.field,
            pygame.Color(148, 17, 0),
            (render_robot_x, render_robot_y),
            self.robot_radius,
            width=5,
        )
        pygame.draw.line(
            self.field,
            pygame.Color(50, 50, 50),
            (render_robot_x, render_robot_y),
            (
                render_robot_x + self.robot_radius * np.cos(self.robot_angle),
                render_robot_y + self.robot_radius * np.sin(self.robot_angle),
            ),
            width=5,
        )

        pygame.display.update()

        self.clock.tick(60)

    def calculate_goalie_reward(self):

        reward = self.calculate_reward()
        if self.defender_collision:
            reward -= 0.01

        return reward

    def reset(self):
        self.time = 0
        self.displacement_coef = 0.2

        self.ball_move = False

        self.contacted_ball = False
        self.ball_go_x = np.random.uniform(500, 1500)
        self.ball_go_y = np.random.uniform(-500, 500)

        self.robot_x = np.random.uniform(-3500, 3500)
        self.robot_y = np.random.uniform(-2500, 2500)
        self.robot_angle = np.random.uniform(0, 2 * np.pi)

        # Experimental values for RL
        # self.robot_x = 2500
        # self.robot_y = 0
        # self.robot_angle = 0

        self.target_x = np.random.uniform(-2500, 2500)
        self.target_y = np.random.uniform(-2000, 2000)

        # Experimental values for RL
        # self.target_x = 1000
        # self.target_y = 0

        # for animation effect for keeper kick
        self.rect2 = Rect(self.target_x, self.target_y, 0, 0)

        self.goal_x = -4500  # center of goal area_x
        self.goal_y = 0  # center of goal area_y

        self.dummy1_x = -4000
        self.dummy1_y = 0

        self.dummy2_x = 5000
        self.dummy2_y = 3400

        self.speed = [0, 10]

        self.update_target_value()
        self.update_goal_value()

        robot_location = np.array([self.robot_x, self.robot_y])
        target_location = np.array([self.target_x, self.target_y])
        self.initial_distance = np.linalg.norm(target_location - robot_location)

        return self._observe_state()

    def collision_dummy(self, dummy_x, dummy_y):
        # Find distance between dummy defender 1 and robot
        dummy_defender_location = np.array([dummy_x, dummy_y])
        robot_location = np.array([self.robot_x, self.robot_y])
        distance_dummy_robot = np.linalg.norm(robot_location - dummy_defender_location)

        # push robot, if collision with dummy defender 1
        if distance_dummy_robot < (self.robot_radius * 2) * 5:
            self.defender_collision = True

            # Update Relative Angle
            delta_x = self.robot_x - dummy_x
            delta_y = self.robot_y - dummy_y
            theta_radians = np.arctan2(delta_y, delta_x)
            self.robot_x = (
                self.robot_x + np.cos(theta_radians) * (self.robot_radius * 2) * 5
            )
            self.robot_y = (
                self.robot_y + np.sin(theta_radians) * (self.robot_radius * 2) * 5
            )

    def collision_ball(self):
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
            self.target_x = (
                self.target_x
                + np.cos(self.theta_radians)
                * (self.robot_radius + self.target_radius)
                * 5
            )
            self.target_y = (
                self.target_y
                + np.sin(self.theta_radians)
                * (self.robot_radius + self.target_radius)
                * 5
            )

    def goalie_logic(self):
        # goalie position : make rect to use pygmae- speed function
        rect1 = Rect(self.dummy1_x, self.dummy1_y, 0, 0)

        # goalie can move inside goali area
        if (
            rect1.centerx > -4450
            and rect1.centerx < -3500
            and rect1.centery > -1100
            and rect1.centery < 1100
        ):
            # goalie goes btw ball and goaline
            x_toward = (self.target_x - 3750) * (7 / 12) - self.dummy1_x

            # goalie movement for Y
            if self.target_y < -750:
                y_toward = -700 - self.dummy1_y
            elif self.target_y > 750:
                y_toward = 700 - self.dummy1_y
            else:
                y_toward = self.target_y - self.dummy1_y

            # get goalie direction vector
            self.speed = [x_toward, y_toward]
            _size = np.linalg.norm(self.speed)
            self.speed = [x_toward / _size * 20, y_toward / _size * 20]

            # this range check help goalie doesn;t go out the goalie area
            if (
                rect1.centerx + self.speed[0] > -4350
                and rect1.centerx + self.speed[0] < -3700
                and rect1.centery + self.speed[1] > -1000
                and rect1.centery + self.speed[1] < 1000
            ):
                rect1 = rect1.move(self.speed)

        # update goali position
        self.dummy1_y = rect1.centery
        self.dummy1_x = rect1.centerx

        # goalie kick : if ball and goalie close enough can kick
        goalie_location = np.array([rect1.centerx, rect1.centery])
        target_location = np.array([self.target_x, self.target_y])
        distance_dummy_target = np.linalg.norm(target_location - goalie_location)

        if distance_dummy_target < (self.robot_radius + self.target_radius) * 8:
            self.ball_move = True

            self.rect2.centerx = self.target_x
            self.rect2.centery = self.target_y

            self.speed = [self.ball_go_x, self.ball_go_y]
            self.rect2 = self.rect2.move(self.speed)
            self.target_x = self.rect2.centerx
            self.target_y = self.rect2.centery


if __name__ == "__main__":
    if not len(sys.argv) == 2:
        print("usage: python ./goalie.py <policy and vector path folder>")
        exit(1)

    policy_path = sys.argv[1] + "/policy.zip"
    normalization_path = sys.argv[1] + "/vector_normalize"

    env = VecNormalize.load(
        normalization_path, make_vec_env(GoalieEnv, n_envs=12)
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

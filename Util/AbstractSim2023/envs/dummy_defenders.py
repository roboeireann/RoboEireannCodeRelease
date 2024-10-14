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

LENGTH = 600
TRAINING_STEPS = 2000000


class DummyDefendersEnv(BaseEnv):

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

        # returns the state of the environment, with global angles and coordinates.

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

        self.update_target_value()
        self.update_goal_value()

        robot_location = np.array([self.robot_x, self.robot_y])
        target_location = np.array([self.target_x, self.target_y])
        self.initial_distance = np.linalg.norm(target_location - robot_location)

        return self._observe_state()

    def calculate_reward(self):
        target_location = np.array([self.target_x, self.target_y])
        goal_location = np.array([self.goal_x, self.goal_y])
        distance_target_goal = np.linalg.norm(goal_location - target_location)
        reward = 0
        if self.defender_collision:
            reward -= 0.001

        if self.contacted_ball:
            reward += 1 / distance_target_goal
        return reward

    def step(self, action):
        self.time += 1
        self.defender_collision = False

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

        self.collision_dummy(self.dummy1_x, self.dummy1_y)
        self.collision_dummy(self.dummy2_x, self.dummy2_y)

        self.collision_ball()
        self.collision_ball_dummy_1()
        self.collision_ball_dummy_2()

        # Turn towards the target as suggested by the policy
        self.robot_angle = self.robot_angle + self.displacement_coef * action[0]

        self.update_target_value()
        self.update_goal_value()

        new_obs = self._observe_state()
        # done = (self.time > PushBallToGoalEnv.EPISODE_LENGTH) or distance_robot_target < self.target_radius
        done = self.time > DummyDefendersEnv.EPISODE_LENGTH

        reward = self.calculate_reward()

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
            pygame.Color(0, 32, 96),
            (render_dummy2_x, render_dummy2_y),
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

    def collision_ball_dummy_1(self):
        # Find distance between robot and target
        robot_location = np.array([self.dummy1_x, self.dummy1_y])
        target_location = np.array([self.target_x, self.target_y])
        distance_robot_target = np.linalg.norm(target_location - robot_location)

        # push ball, if collision with robot
        if distance_robot_target < (self.robot_radius + self.target_radius) * 5:
            # changed to reached reward mode
            self.contacted_ball = True

            # Update Relative Angle
            delta_x = self.target_x - self.dummy1_x
            delta_y = self.target_y - self.dummy1_y
            theta_radians = np.arctan2(delta_y, delta_x)
            self.target_x = (
                self.target_x
                + np.cos(theta_radians) * (self.robot_radius + self.target_radius) * 5
            )
            self.target_y = (
                self.target_y
                + np.sin(theta_radians) * (self.robot_radius + self.target_radius) * 5
            )

    def collision_ball_dummy_2(self):
        # Find distance between robot and target
        robot_location = np.array([self.dummy2_x, self.dummy2_y])
        target_location = np.array([self.target_x, self.target_y])
        distance_robot_target = np.linalg.norm(target_location - robot_location)

        # push ball, if collision with robot
        if distance_robot_target < (self.robot_radius + self.target_radius) * 5:
            # changed to reached reward mode
            self.contacted_ball = True

            # Update Relative Angle
            delta_x = self.target_x - self.dummy2_x
            delta_y = self.target_y - self.dummy2_y
            theta_radians = np.arctan2(delta_y, delta_x)
            self.target_x = (
                self.target_x
                + np.cos(theta_radians) * (self.robot_radius + self.target_radius) * 5
            )
            self.target_y = (
                self.target_y
                + np.sin(theta_radians) * (self.robot_radius + self.target_radius) * 5
            )


if __name__ == "__main__":
    if not len(sys.argv) == 2:
        print("usage: python ./dummy_defenders.py <policy and vector path folder>")
        exit(1)

    policy_path = sys.argv[1] + "/policy.zip"
    normalization_path = sys.argv[1] + "/vector_normalize"

    env = VecNormalize.load(
        normalization_path, make_vec_env(DummyDefendersEnv, n_envs=12)
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

import math
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

import warnings
from utils.utils import save_vec_normalize_data

# Import base environment
from envs.base import BaseEnv

warnings.filterwarnings("ignore")

'''
    Keeyaway environment
'''

# Hyperparameters
ADVERSARY_SPEED = 1

class KeepawayEnv(BaseEnv):
    def __init__(self):
        self.rendering_init = False

        """
        ACTION SPACE:
            - Angle to turn clockwise
            - Translation along x-axis
            - Translation along y-axis
        """
        self.num_agents = 2
        self.num_adversaries = 1

        action_space_low = np.array([-np.pi / 2, -1, -1, 0] * self.num_agents)
        action_space_high = np.array([np.pi / 2, 1, 1, 1] * self.num_agents)
        self.action_space = gym.spaces.Box(action_space_low, action_space_high)

        """
        OBSERVATION SPACE:
            - x-cordinate of robot with respect to target
            - y-cordinate of robot with respect to target
            - sin(Angle between robot and target)
            - cos(Angle between robot and target)
        """
        observation_space_size = 2 + 2 * self.num_agents + 2 * self.num_adversaries
        observation_space_low = -1 * np.ones(observation_space_size)
        observation_space_high = np.ones(observation_space_size)
        self.observation_space = gym.spaces.Box(
            observation_space_low, observation_space_high
        )

        self.target_radius = 10
        self.robot_radius = 20

        self.reset()


    def _observe_state(self):

        self.update_target_value()
        self.update_goal_value()


        state = []
     
        # Add target positions
        state.extend([
            self.target_x,
            self.target_y,
        ]
        )

        # Add robot positions
        for i in range(self.num_agents):
            state.extend([
                self.robot_x[i],
                self.robot_y[i],
            ])

        # Add adversary positions
        for i in range(self.num_adversaries):
            state.extend([
                self.adversary_x[i],
                self.adversary_y[i],
            ])
            
        return np.array(state)

    def reset(self):
        self.time = 0
        self.displacement_coef = 0.2

        self.contacted_ball = False
        self.adversary_contacted_ball = False

        self.robot_x = []
        self.robot_y = []
        self.robot_angle = []

        self.relative_angle = []
        self.goal_relative_angle = []

        for i in range(self.num_agents):
            self.robot_x.append(np.random.uniform(-3500, 3500))
            self.robot_y.append(np.random.uniform(-2500, 2500))
            self.robot_angle.append(np.random.uniform(0, 2 * np.pi))
            self.relative_angle.append(0)
            self.goal_relative_angle.append(0)

        self.adversary_x = []
        self.adversary_y = []

        for i in range(self.num_adversaries):
            self.adversary_x.append(np.random.uniform(-3500, 3500))
            self.adversary_y.append(np.random.uniform(-2500, 2500))

        self.target_x = np.random.uniform(-2500, 2500)
        self.target_y = np.random.uniform(-2000, 2000)

        self.goal_x = -4500
        self.goal_y = 0

        self.update_goal_value()
        self.update_target_value()

        return self._observe_state()

    def out_of_bounds(self):
        # Check if any robots are out of bounds
        for i in range(self.num_agents):
            # Top/Bottom -line out
            if self.robot_y[i] < -3000 or self.robot_y[i] > 3000:
                return True

            # Right -line out
            if self.robot_x[i] > 4500:
                return True

            # Left -line out(upper/goal area)
            if self.robot_x[i] < -4500 and self.robot_y[i] < -750:
                return True

            # Left -line out(under/goal area)
            if self.robot_x[i] < -4500 and self.robot_y[i] > 750:
                return True

        return False

    def get_distance_target_goal(self):
        target_location = np.array([self.target_x, self.target_y])
        goal_location = np.array([self.goal_x, self.goal_y])
        return np.linalg.norm(goal_location - target_location)
    
    def check_facing_ball(self, robot_index):
        # Normalize angle to 0-360
        robot_angle = math.degrees(self.robot_angle[robot_index]) % 360

        # Find the angle between the robot and the ball
        angle_to_ball = math.degrees(
            math.atan2(self.target_y - self.robot_y[robot_index], self.target_x - self.robot_x[robot_index])
        )

        # Check if the robot is facing the ball
        if abs(angle_to_ball - robot_angle) < 30:
            return True
        else:
            return False

    '''
    Reward for keepaway is 1 for every timestep the adversary does not touch the ball
    '''
    def calculate_reward(self):
        if self.adversary_contacted_ball:
            return 0
        if self.contacted_ball:
            return 1
        return 0.001

    class Point:
        def __init__(self, x, y):
            self.x = x
            self.y = y

    # Return true if line segments AB and CD intersect, for goal line
    def intersect(self, A, B, C, D):
        def ccw(A,B,C):
            return (C.y-A.y) * (B.x-A.x) > (B.y-A.y) * (C.x-A.x)

        return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)

    def kick_ball(self, robot_index):
        if self.check_facing_ball(robot_index):
            robot_location = np.array([self.robot_x[robot_index], self.robot_y[robot_index]])
            target_location = np.array([self.target_x, self.target_y])

            # Find distance between robot and target
            distance_robot_target = np.linalg.norm(target_location - robot_location)

            # If robot is close enough to ball, kick ball
            if distance_robot_target < (self.robot_radius + self.target_radius) * 10:
                self.contacted_ball = True

                # Update Relative Angle
                self.delta_x = self.target_x - self.robot_x[robot_index]
                self.delta_y = self.target_y - self.robot_y[robot_index]
                self.theta_radians = np.arctan2(self.delta_y, self.delta_x)

                self.target_x = (
                    self.target_x
                    + np.cos(self.theta_radians)
                    * (self.robot_radius + self.target_radius)
                    * 80
                )
                self.target_y = (
                    self.target_y
                    + np.sin(self.theta_radians)
                    * (self.robot_radius + self.target_radius)
                    * 80
                )

                # Check if line of ball intersects goal line
                A = self.Point(self.target_x, self.target_y)
                B = self.Point(self.robot_x[robot_index], self.robot_y[robot_index])
                C = self.Point(-4500, -750)
                D = self.Point(-4500, 750)

                # Put ball at goal location (-4501, 0)
                if self.intersect(A, B, C, D):
                    self.target_x = -4501
                    self.target_y = 0

    def move_robot(self, action, robot_index):
            # Find location policy is trying to reach
        policy_target_x = self.robot_x[robot_index] + (
            (
                (np.cos(self.robot_angle[robot_index]) * np.clip(action[1], -1, 1))
                + (np.cos(self.robot_angle[robot_index] + np.pi / 2) * np.clip(action[2], -1, 1))
            )
            * 200
        )  # the x component of the location targeted by the high level action
        policy_target_y = self.robot_y[robot_index] + (
            (
                (np.sin(self.robot_angle[robot_index]) * np.clip(action[1], -1, 1))
                + (np.sin(self.robot_angle[robot_index] + np.pi / 2) * np.clip(action[2], -1, 1))
            )
            * 200
        )  # the y component of the location targeted by the high level action

        # Update robot position
        self.robot_x[robot_index] = (
            self.robot_x[robot_index] * (1 - self.displacement_coef)
            + policy_target_x * self.displacement_coef
        )  # weighted sums based on displacement coefficient
        self.robot_y[robot_index] = (
            self.robot_y[robot_index] * (1 - self.displacement_coef)
            + policy_target_y * self.displacement_coef
        )  # the idea is we move towards the target position and angle

        self.position_rule(robot_index)

        # Find distance between robot and target
        robot_location = np.array([self.robot_x[robot_index], self.robot_y[robot_index]])
        target_location = np.array([self.target_x, self.target_y])
        distance_robot_target = np.linalg.norm(target_location - robot_location)

        # push ball, if collision with robot
        if distance_robot_target < (self.robot_radius + self.target_radius) * 5:
            # changed to reached reward mode
            self.contacted_ball = True

            # Update Relative Angle
            self.delta_x = self.target_x - self.robot_x[robot_index]
            self.delta_y = self.target_y - self.robot_y[robot_index]
            self.theta_radians = np.arctan2(self.delta_y, self.delta_x)

            self.target_x = (
                self.target_x
                + np.cos(self.theta_radians)
                * (self.robot_radius + self.target_radius)
                * 6
            )
            self.target_y = (
                self.target_y
                + np.sin(self.theta_radians)
                * (self.robot_radius + self.target_radius)
                * 5
            )

                # Check if line of ball intersects goal line
            A = self.Point(self.target_x, self.target_y)
            B = self.Point(self.robot_x[robot_index], self.robot_y[robot_index])
            C = self.Point(-4500, -750)
            D = self.Point(-4500, 750)

            # Put ball at goal location (-4500, 0)
            if self.intersect(A, B, C, D):
                self.target_x = -4501
                self.target_y = 0

        # Turn towards the target as suggested by the policy
        self.robot_angle[robot_index] = self.robot_angle[robot_index] + self.displacement_coef * action[0]

    '''
    Adversary always moves towards the ball
    '''
    def step_adversary(self, adversary_index):
        # Find distance between adversary and target
        adversary_location = np.array([self.adversary_x[adversary_index], self.adversary_y[adversary_index]])
        target_location = np.array([self.target_x, self.target_y])
        distance_adversary_target = np.linalg.norm(target_location - adversary_location)

        # Find angle between adversary and target
        self.delta_x = self.target_x - self.adversary_x[adversary_index]
        self.delta_y = self.target_y - self.adversary_y[adversary_index]
        self.theta_radians = np.arctan2(self.delta_y, self.delta_x)

        # Move adversary towards ball
        self.adversary_x[adversary_index] = self.adversary_x[adversary_index] + np.cos(self.theta_radians) * 25 * ADVERSARY_SPEED
        self.adversary_y[adversary_index] = self.adversary_y[adversary_index] + np.sin(self.theta_radians) * 25 * ADVERSARY_SPEED

        # If adversary touches ball, set reward for robots to 0
        if distance_adversary_target < (self.robot_radius + self.target_radius) * 5:
            self.adversary_contacted_ball = True
        
    '''
    Actions for robots:
    [0] - angular velocity
    [1] - x velocity
    [2] - y velocity
    [3] - kick ball
    '''
    def step(self, action):
        self.time += 1

        for i in range(self.num_agents):
            
            # Get current robot action
            current_action = action[0 + i * 4 : 4 + i * 4]
            
            # Attempt to kick ball
            if current_action[3] > 0.95:
                self.kick_ball(i)       
            # If not kicking, move robot
            else:
                self.move_robot(current_action, i)

        # Move adversary
        for i in range(self.num_adversaries):
            self.step_adversary(i)

        self.update_target_value()
        self.update_goal_value()

        done = self.time > BaseEnv.EPISODE_LENGTH

        reward = self.calculate_reward()

        new_obs = self._observe_state()

        return (new_obs, reward, done, {})

    def render_robot(self, robot_index):
        Field_length = 1200
        render_robot_x = (self.robot_x[robot_index] / 5200 + 1) * (Field_length / 2)
        render_robot_y = (self.robot_y[robot_index] / 3700 + 1) * (Field_length / 3)

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
                render_robot_x + self.robot_radius * np.cos(self.robot_angle[robot_index]),
                render_robot_y + self.robot_radius * np.sin(self.robot_angle[robot_index]),
            ),
            width=5,
        )

    def render_adversary(self):
        # Pink = 255 51 255
        # Blue = 63 154 246

        color = (63, 154, 246)
        if self.adversary_contacted_ball:
            color = (255, 51, 255)

        for i in range(self.num_adversaries):
            Field_length = 1200
            render_robot_x = (self.adversary_x[i] / 5200 + 1) * (Field_length / 2)
            render_robot_y = (self.adversary_y[i] / 3700 + 1) * (Field_length / 3)

            pygame.draw.circle(
                self.field,
                pygame.Color(color[0], color[1], color[2]),
                (render_robot_x, render_robot_y),
                self.robot_radius,
                width=5,
            )


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

        # Render robots
        for i in range(self.num_agents):
            self.render_robot(i)

        # Render adversary
        self.render_adversary()

        # Render target
        render_target_x = (self.target_x / 5200 + 1) * (Field_length / 2)
        render_target_y = (self.target_y / 3700 + 1) * (Field_length / 3)

        pygame.draw.circle(
            self.field,
            pygame.Color(40, 40, 40),
            (render_target_x, render_target_y),
            self.target_radius,
        )

        pygame.display.update()
        self.clock.tick(60)

    def position_rule(self, robot_index):
        # Ball out : 3 case
        # 1) corner kick : right side of field
        if self.target_x > 4500:
            if self.target_y > 750:
                self.target_x = 4500
                self.target_y = 3000
            elif self.target_y < -750:
                self.target_x = 4500
                self.target_y = -3000
        # 2) corner kick :checking ball out of bounds on left side of field
        if self.target_x < -4500:
            if self.target_y > 750:
                self.target_x = -4500
                self.target_y = 3000
            elif self.target_y < -750:
                self.target_x = -4500
                self.target_y = -3000

        # 3) ball out of bounds on top/bottom of field
        if self.target_y > 3000:
            self.target_y = 3000
        if self.target_y < -3000:
            self.target_y = -3000


        # Robot movement : 2 case
        # checking robot out of bounds (700 additional units past field line)
        if self.robot_x[robot_index] > 4500 + 700:
            self.robot_x[robot_index] = 4500
        if self.robot_x[robot_index] < -4500 - 700:
            self.robot_x[robot_index] = -4500
        if self.robot_y[robot_index] > 3000 + 700:
            self.robot_y[robot_index] = 3000
        if self.robot_y[robot_index] < -3000 - 700:
            self.robot_y[robot_index] = -3000

        # disallow robot pass through goal-net
        if self.robot_x[robot_index] < -4500 or self.robot_x[robot_index] > 4500:
            if abs(-750 - self.robot_y[robot_index]) < 20:
                if self.robot_y[robot_index] < -750:
                    self.robot_y[robot_index] = -900
                else:
                    self.robot_y[robot_index] = -700
            elif abs(750 - self.robot_y[robot_index]) < 20:
                if self.robot_y[robot_index] > 750:
                    self.robot_y[robot_index] = 900
                else:
                    self.robot_y[robot_index] = 700

    def update_target_value(self):
        for i in range(self.num_agents):
            # Update Relative Angle
            self.delta_x = self.target_x - self.robot_x[i]
            self.delta_y = self.target_y - self.robot_y[i]
            self.theta_radians = np.arctan2(self.delta_y, self.delta_x)

            if self.theta_radians >= 0:
                self.relative_angle[i] = self.theta_radians
            else:
                self.relative_angle[i] = self.theta_radians + 2 * np.pi

    def update_goal_value(self):
        for i in range(self.num_agents):
            # Update Relative Angle to goal
            self.goal_delta_x = self.goal_x - self.robot_x[i]
            self.goal_delta_y = self.goal_y - self.robot_y[i]
            self.goal_theta_radians = np.arctan2(self.goal_delta_y, self.goal_delta_x)

            if self.goal_theta_radians >= 0:
                self.goal_relative_angle[i] = self.goal_theta_radians
            else:
                self.goal_relative_angle[i] = self.goal_theta_radians + 2 * np.pi

if __name__ == "__main__":
    if not len(sys.argv) == 2:
        print("usage: python ./keepaway.py <policy and vector path folder>")
        exit(1)

    policy_path = sys.argv[1] + "/policy.zip"
    normalization_path = sys.argv[1] + "/vector_normalize"

    env = VecNormalize.load(
        normalization_path, make_vec_env(KeepawayEnv, n_envs=12)
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


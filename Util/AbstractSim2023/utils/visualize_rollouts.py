# derived from https://stable-baselines3.readthedocs.io/en/master/modules/ppo.html

import gym
import torch
import numpy as np
import json
import torch
import torch.nn as nn
import torch.nn.functional as functional
from torch.utils.data import TensorDataset, DataLoader
import torch.optim as optim
from tensorflow import keras
import tensorflow as tf
from scipy.stats import multivariate_normal
import math
from threading import Thread
import socket
import os.path
import time
import sys
sys.path.append(sys.path[0] + "/..")
import copy

import matplotlib.pyplot as plt

from tqdm import tqdm
from pytorch2keras import pytorch_to_keras
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env.base_vec_env import VecEnv
from stable_baselines3.common.buffers import RolloutBuffer
from stable_baselines3.common.preprocessing import get_action_dim, get_obs_shape
from stable_baselines3.common.utils import obs_as_tensor
from matplotlib.collections import LineCollection

from utils.train_grounding_function import NETWORK as Network


from envs.push_ball_to_goal import PushBallToGoalEnv
from utils.utils import get_string_from_file, get_json_from_file_name
from envs.grounded_env import GroundedEnv

BATCH_SIZE = 10000
TRAIN_EPOCHS = 2000

HISTORY_LENGTH = 5
EPISODE_LENGTH = 200


def get_coords(abstract_state, action):
    ball_agent_x_offset = abstract_state[0] * 9000
    ball_agent_y_offset = abstract_state[1] * 6000
    goal_ball_x_offset = abstract_state[2] * 9000
    goal_ball_y_offset = abstract_state[3] * 6000

    ball_robot_angle_offset_sin = abstract_state[4]
    ball_robot_angle_offset_cos = abstract_state[5]
    goal_robot_angle_offset_sin = abstract_state[6]
    goal_robot_angle_offset_cos = abstract_state[7]

    target_x = -4500 - goal_ball_x_offset
    target_y = 0 - goal_ball_y_offset

    robot_x = target_x - ball_agent_x_offset
    robot_y = target_y - ball_agent_y_offset

    ball_robot_angle_offset = np.arctan2(
        ball_robot_angle_offset_sin, ball_robot_angle_offset_cos
    )
    goal_robot_angle_offset = np.arctan2(
        goal_robot_angle_offset_sin, goal_robot_angle_offset_cos
    )

    robot_angle_to_ball = np.arctan2(ball_agent_y_offset, ball_agent_x_offset)

    robot_angle_to_ball = robot_angle_to_ball

    robot_angle = robot_angle_to_ball - ball_robot_angle_offset

    policy_target_x = robot_x + (
        (
            (np.cos(robot_angle) * np.clip(action[1], -1, 1))
            + (np.cos(robot_angle + np.pi / 2) * np.clip(action[2], -1, 1))
        )
        * 200
    )  # the x component of the location targeted by the high level action
    policy_target_y = robot_y + (
        (
            (np.sin(robot_angle) * np.clip(action[1], -1, 1))
            + (np.sin(robot_angle + np.pi / 2) * np.clip(action[2], -1, 1))
        )
        * 200
    )  # the y component of the location targeted by the high level action

    return robot_x, robot_y, target_x, target_y, policy_target_x, policy_target_y


def visualize_abstract_rollout(env, model):

    model.set_env(env)

    agent_x_list = []
    agent_y_list = []
    action_x_list = []
    action_y_list = []
    ball_x_list = []
    ball_y_list = []

    obs = env.reset()

    done = False
    while not done:
        action = model.predict(obs, deterministic=False)[0]
        robot_x, robot_y, target_x, target_y, action_x, action_y = get_coords(
            obs, action
        )
        agent_x_list.append(robot_x)
        agent_y_list.append(robot_y)
        action_x_list.append(action_x)
        action_y_list.append(action_y)
        ball_x_list.append(target_x)
        ball_y_list.append(target_y)
        obs, _, done, _ = env.step(action)

    print(agent_x_list)
    print(agent_y_list)
    print(action_x_list)
    print(action_y_list)
    print(ball_x_list)
    print(ball_y_list)
    fig, ax = plt.subplots()

    agent_points = [(x, y) for x, y in zip(agent_x_list, agent_y_list)]
    action_points = [(x, y) for x, y in zip(action_x_list, action_y_list)]
    lines = [
        [agent_point, action_point]
        for agent_point, action_point in zip(agent_points, action_points)
    ]

    col = LineCollection(lines)
    ax.add_collection(col)

    plt.scatter(
        agent_x_list,
        agent_y_list,
        c=[i for i in range(len(agent_x_list))],
        cmap="Blues",
    )
    plt.scatter(
        ball_x_list, ball_y_list, c=[i for i in range(len(ball_x_list))], cmap="Greens"
    )
    plt.scatter(
        action_x_list,
        action_y_list,
        c=[i for i in range(len(action_x_list))],
        cmap="Reds",
    )

    plt.show()


def visualize_recorded_rollout(trajectories, episode_length):

    agent_x_list = []
    agent_y_list = []
    action_x_list = []
    action_y_list = []
    ball_x_list = []
    ball_y_list = []

    for i in range(episode_length):
        observation = trajectories["observations"][i]
        action = trajectories["actions"][i]

        print(action)
        print(observation)
        robot_x, robot_y, target_x, target_y, action_x, action_y = get_coords(
            observation, action
        )
        agent_x_list.append(robot_x)
        agent_y_list.append(robot_y)
        action_x_list.append(action_x)
        action_y_list.append(action_y)
        ball_x_list.append(target_x)
        ball_y_list.append(target_y)
    fig, ax = plt.subplots()

    agent_points = [(x, y) for x, y in zip(agent_x_list, agent_y_list)]
    action_points = [(x, y) for x, y in zip(action_x_list, action_y_list)]
    lines = [
        [agent_point, action_point]
        for agent_point, action_point in zip(agent_points, action_points)
    ]

    col = LineCollection(lines)
    ax.add_collection(col)

    plt.scatter(
        agent_x_list,
        agent_y_list,
        c=[i for i in range(len(agent_x_list))],
        cmap="Blues",
    )
    plt.scatter(
        ball_x_list, ball_y_list, c=[i for i in range(len(ball_x_list))], cmap="Greens"
    )
    plt.scatter(
        action_x_list,
        action_y_list,
        c=[i for i in range(len(action_x_list))],
        cmap="Reds",
    )

    plt.show()


if __name__ == "__main__":

    trajectories = get_json_from_file_name("standard_trajectories.json")

    # print(trajectories)
    # visualize_recorded_rollout(trajectories,EPISODE_LENGTH)
    # exit()

    net = Network(HISTORY_LENGTH)
    net.load_state_dict(torch.load("./sim_grounding_function"))

    env_lambda = lambda: GroundedEnv(PushBallToGoalEnv(), net, HISTORY_LENGTH)

    env = PushBallToGoalEnv()
    # env = env_lambda()
    model = PPO.load("Models/pushBallToTarget_1000000.zip")
    # model = PPO.load("pushBallToTarget_grounded_1500000.zip")

    visualize_abstract_rollout(env, model)

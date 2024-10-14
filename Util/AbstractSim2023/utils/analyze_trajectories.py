# derived from https://stable-baselines3.readthedocs.io/en/master/modules/ppo.html

import gym
import torch
import numpy as np
import json
import torch
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

from pytorch2keras import pytorch_to_keras
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.vec_env.base_vec_env import VecEnv
from stable_baselines3.common.buffers import RolloutBuffer
from stable_baselines3.common.preprocessing import get_action_dim, get_obs_shape
from stable_baselines3.common.utils import obs_as_tensor
from walkForward import PointTargetingEnv

# todo minify this file to only necessary code


def get_string_from_file(file_name):
    with open(file_name, "r") as data_file:
        data = data_file.read()
        return data


"""
This is a tool provided for ensuring that the policies ported to the Robosoccer code are matching the behavior of the PyTorch and Keras Neural networks with the same architecture 
and parameters
"""


def analyze_trajectories(file_path, model_path):

    env = DummyVecEnv([PointTargetingEnv])

    model = PPO.load(model_path, env)

    with open("metadata.json", "w") as metadata_file:
        json.dump(
            {
                "log_stds": model.policy.log_std.data.detach().numpy().tolist(),
                "observation_length": sum(model.observation_space.shape),
                "action_length": sum(model.action_space.shape),
            },
            metadata_file,
        )

    total_model = pytorch_to_keras(
        model.policy.mlp_extractor,
        torch.zeros((1, model.observation_space.shape[0])),
        verbose=False,
        name_policy="short",
    )
    action_model = pytorch_to_keras(
        model.policy.action_net,
        torch.zeros((1, 64)),
        verbose=False,
        name_policy="short",
    )
    value_model = pytorch_to_keras(
        model.policy.value_net, torch.zeros((1, 64)), verbose=False, name_policy="short"
    )

    with open(file_path, "r") as trajectory_file:
        trajectories = json.load(trajectory_file)
        observations = trajectories["observations"]
        actions = trajectories["action_means"]

        for observation, action in zip(observations, actions):
            print("observation")
            print(observation)
            print("recorded action mean")
            print(action)
            print("stable-baselines model prediction (clipped by observation space)")
            print(list(model.predict(observation, deterministic=True)[0]))
            print(
                "keras exported policy, does not have clipping based on observation space"
            )
            print(
                list(
                    analyze_keras_model(
                        observation, total_model, action_model, value_model
                    )[0]
                )
            )


def preprocess_trajectories(file_path):
    trajectories = None
    with open(file_path, "r") as trajectory_file:
        trajectories = json.load(trajectory_file)
        trajectories["observations"] = [
            [observation] for observation in trajectories["observations"]
        ]
        trajectories["rewards"] = [
            [observation[0][0]] for observation in trajectories["observations"]
        ]
        trajectories["episode_starts"] = [
            [episode_start] for episode_start in trajectories["episode_starts"]
        ]
        trajectories["log_probs"] = [
            [log_prob] for log_prob in trajectories["log_probs"]
        ]
        trajectories["values"] = [[value] for value in trajectories["values"]]
        trajectories["last_values"] = [
            [last_value] for last_value in trajectories["last_values"]
        ]
        trajectories["actions"] = [[action] for action in trajectories["actions"]]
    with open(file_path, "w") as trajectory_file:
        json.dump(trajectories, trajectory_file)


def analyze_keras_model(observation, shared_model, action_model, value_model):

    log_stds = None
    observation_length = None
    with open("metadata.json", "r") as metadata_file:
        metadata = json.load(metadata_file)
        log_stds = np.array(metadata["log_stds"])
        observation_length = int(metadata["observation_length"])

    # observation = np.array([469.956,326.441,-0.0422514,0.999107])
    observation = np.reshape(observation, (1, observation_length))
    # observation = np.reshape(np.array([0 for i in range(observation_length)]), (1,observation_length))
    # print("OBSERVATION")
    # print(observation)
    output = shared_model.predict(observation)
    # print(output)
    # print(output[0].shape)

    # print("SHARED OUTPUT 1")
    # print(output[0])
    # print("SHARED OUTPUT 2")
    # print(output[1])

    latent_action = output[0]
    latent_value = output[1]

    # print(latent_action)
    action_data = action_model.predict(latent_action)
    action_length = len(action_data[0])
    # print("action mean")
    # print(action_data)
    value_data = value_model.predict(latent_value)
    # print(value_data)
    cov_mat = np.zeros((action_length, action_length))
    # print(cov_mat)
    for i in range(action_length):
        cov_mat[i][i] = math.exp(log_stds[i])
    # print(cov_mat)

    if len(action_data) == 1:
        cov_mat = cov_mat[0][
            0
        ]  # in this case cov_mat is a 1x1 matrix which has a scalar data type

    # print(cov_mat)
    # print(action_data)
    mvn = multivariate_normal(mean=action_data[0], cov=cov_mat, allow_singular=False)
    sample = mvn.rvs(size=1)
    # print(sample)
    prob = multivariate_normal.pdf(
        sample, mean=action_data[0], cov=cov_mat, allow_singular=False
    )
    # print(prob)
    log_prob = math.log(prob)
    # print(log_prob)
    return (action_data[0], value_data[0], log_prob)


def save_as_H5(model, file_path="./"):

    print(model.observation_space.shape)
    total_model = pytorch_to_keras(
        model.policy.mlp_extractor,
        torch.zeros((1, model.observation_space.shape[0])),
        verbose=True,
        name_policy="short",
    )
    action_model = pytorch_to_keras(
        model.policy.action_net, torch.zeros((1, 64)), verbose=True, name_policy="short"
    )
    value_model = pytorch_to_keras(
        model.policy.value_net, torch.zeros((1, 64)), verbose=True, name_policy="short"
    )

    total_model.save(file_path + "shared_policy.h5", save_format="h5")
    action_model.save(file_path + "action_policy.h5", save_format="h5")
    value_model.save(file_path + "value_policy.h5", save_format="h5")

    print(model.policy.log_std)

    with open(file_path + "metadata.json", "w") as metadata_file:
        json.dump(
            {
                "log_stds": model.policy.log_std.data.detach().numpy().tolist(),
                "observation_length": sum(model.observation_space.shape),
                "action_length": sum(model.action_space.shape),
            },
            metadata_file,
        )


def serialize_model(model):
    save_as_H5(model)
    shared_model = get_string_from_file("shared_" + file_path)
    action_model = get_string_from_file("action_" + file_path)
    value_model = get_string_from_file("value_" + file_path)
    metadata = None
    with open("metadata.json", "r") as metadata_file:
        metadata = json.load("metadata.json")
    result_data = {
        "shared_model": shared_model,
        "action_model": action_model,
        "value_model": value_model,
        "metadata": metadata,
    }
    return json.dumps(result_data)


def save_as_torch_model(model, file_path="policy.mdl"):
    torch.save(model, file_path)


def test_ppo_integration(epochs=10, batch_size=2000):
    n_envs = 1  # hardcoded for now

    env = make_vec_env(DummyWrapper, n_envs=n_envs)
    model = PPO("MlpPolicy", env, verbose=1, n_steps=batch_size // n_envs)

    save_as_H5(model, file_path="./BHumanCodeRelease/Config/")

    for i in range(epochs):
        while not os.path.isfile(f"./BHumanCodeRelease/Config/trajectories_{i}.json"):
            time.sleep(0.05)

        time.sleep(0.05)  # this is a bandaid instead of using a lock

        preprocess_trajectories(f"./BHumanCodeRelease/Config/trajectories_{i}.json")

        model.learn(
            batch_size,
            load_saved_trajectories=True,
            trajectory_file_name=f"./BHumanCodeRelease/Config/trajectories_{i}.json",
        )
        save_as_H5(model, file_path="./BHumanCodeRelease/Config/")
    model.save("best_model")


def test_PPO_trajectory_loading(epochs=10, batch_size=1000, n_envs=1):

    # env = make_vec_env("BipedalWalker-v3", n_envs=n_envs)

    env = make_vec_env(DummyWrapper, n_envs=n_envs)

    print(env.observation_space.shape)
    print(env.envs[0].observation_space.shape)
    model = PPO("MlpPolicy", env, verbose=1, n_steps=batch_size // n_envs)

    for i in range(epochs):
        # save_as_H5(model)
        save_trajectories(
            model, env, batch_size, output_file_name=f"trajectories_{i}.json"
        )
        model.learn(
            batch_size,
            load_saved_trajectories=True,
            trajectory_file_name=f"trajectories_{i}.json",
        )

    obs = env.reset()

    save_as_H5(model)
    # save_as_torch_model(model)

    while True:
        action, _states = model.predict(obs)
        obs, rewards, dones, info = env.step(action)
        env.render()


def env_server_mockup(env, batch_size=1000, port=7891, epochs=10):

    # set up TCP socket
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    host = "127.0.0.1"
    s.bind((host, port))
    s.listen()
    connection, address = s.accept()

    for i in range(epochs):
        data = ""
        new_data = connection.recv(1024)
        while new_data:
            data = data + new_data
            new_data = s.recv(1024)
        print("data")
        connection.sendall(data)

        # recieve model through socket
        # deploy model to collect trahectories
        # send trajectories through tcp socket


def test_PPO_trajectory_socket(epochs=10, batch_size=1000, n_envs=1):

    env = make_vec_env("BipedalWalker-v3", n_envs=n_envs)

    thread = Thread(target=env_server_mockup, args=(env,))
    thread.start()
    print("reached")

    print(env.observation_space.shape)
    print(env.envs[0].observation_space.shape)
    model = PPO("MlpPolicy", env, verbose=1, n_steps=batch_size // n_envs)

    for i in range(epochs):
        # send policy
        # save_trajectories(model,env,batch_size,output_file_name = f"trajectories_{i}.json")

        model.learn(
            batch_size,
            load_saved_trajectories=True,
            trajectory_file_name=f"trajectories_{i}.json",
        )

    obs = env.reset()

    save_as_H5(model)
    # save_as_torch_model(model)

    while True:
        action, _states = model.predict(obs)
        obs, rewards, dones, info = env.step(action)
        env.render()


def test_PPO_standard(epochs=10, batch_size=100, n_envs=2):
    env = make_vec_env("BipedalWalker-v3", n_envs=n_envs)
    model = PPO("MlpPolicy", env, verbose=1, n_steps=batch_size // n_envs)

    model.learn(batch_size * epochs, load_saved_trajectories=False)
    obs = env.reset()

    while True:
        action, _states = model.predict(obs)
        obs, rewards, dones, info = env.step(action)
        env.render()


def save_trajectories(model, env, n_timesteps, output_file_name="trajectories.json"):

    n_envs = env.num_envs

    assert isinstance(env, VecEnv), "only VecEnv supported for now"
    assert (n_timesteps % n_envs) == 0

    obs_shape = get_obs_shape(env.observation_space)
    action_dim = get_action_dim(env.action_space)

    saved_observations = np.zeros(
        (n_timesteps // n_envs, n_envs) + obs_shape, dtype=np.float32
    )
    saved_actions = np.zeros(
        (n_timesteps // n_envs, n_envs, action_dim), dtype=np.float32
    )

    saved_rewards = np.zeros((n_timesteps // n_envs, n_envs), dtype=np.float32)
    # saved_returns = np.zeros((n_timesteps // n_envs, n_envs), dtype=np.float32)
    saved_episode_starts = np.zeros((n_timesteps // n_envs, n_envs), dtype=np.float32)

    saved_values = np.zeros((n_timesteps // n_envs, n_envs), dtype=np.float32)
    saved_log_probs = np.zeros((n_timesteps // n_envs, n_envs), dtype=np.float32)
    # saved_advantages = np.zeros((n_timesteps // n_envs, n_envs), dtype=np.float32)

    obs = env.reset()
    prev_dones = np.array(
        [True for i in range(n_envs)]
    )  # the after calling reset for the first time done is implicitly false.

    for i in range(n_timesteps // env.num_envs):

        with torch.no_grad():
            obs_tensor = torch.Tensor(obs)
            actions, values, log_probs = model.policy.forward(obs_tensor)
        actions = actions.cpu().numpy()
        clipped_actions = actions
        if isinstance(env.action_space, gym.spaces.Box):
            clipped_actions = np.clip(
                actions, env.action_space.low, env.action_space.high
            )
        new_obs, rewards, dones, infos = env.step(clipped_actions)

        # update buffer
        saved_observations[i, :, :] = obs
        saved_actions[i, :, :] = actions.reshape(
            n_envs, action_dim
        )  # this may not be robust
        saved_episode_starts[i, :] = prev_dones
        saved_rewards[i, :] = rewards
        saved_values[i, :] = values.view(n_envs)
        saved_log_probs[i, :] = log_probs

        with torch.no_grad():
            _, last_values, _ = model.policy.forward(torch.Tensor(new_obs))

        obs = new_obs
        prev_dones = dones
    with open(output_file_name, "w") as output_file:
        trajectories = {
            "last_values": last_values.tolist(),
            "length": n_timesteps // n_envs,
            "observations": saved_observations.tolist(),
            "actions": saved_actions.tolist(),
            "episode_starts": saved_episode_starts.tolist(),
            "rewards": saved_rewards.tolist(),
            "values": saved_values.tolist(),
            "log_probs": saved_log_probs.tolist(),
        }
        json.dump(trajectories, output_file)


if __name__ == "__main__":

    if not len(sys.argv) == 3:
        print("usage: ./export_model.py <trajectory_file_path> <model_file_path>")
        exit(1)

    analyze_trajectories(sys.argv[1], sys.argv[2])

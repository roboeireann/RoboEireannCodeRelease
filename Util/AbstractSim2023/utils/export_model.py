# derived from https://stable-baselines3.readthedocs.io/en/master/modules/ppo.html

# Example usage:
# python utils/export_model.py Models/kick_to_goal/policy.zip Models/kick_to_goal/vector_normalize.json

import gym
import torch
import numpy as np
import json
from scipy.stats import multivariate_normal
import math
from threading import Thread
import sys
sys.path.append(sys.path[0] + "/..")

from pytorch2keras import pytorch_to_keras
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env.base_vec_env import VecEnv
from stable_baselines3.common.vec_env import VecNormalize, DummyVecEnv

from stable_baselines3.common.buffers import RolloutBuffer
from stable_baselines3.common.preprocessing import get_action_dim, get_obs_shape
from stable_baselines3.common.utils import obs_as_tensor


# todo minify this file to only necessary code


def get_string_from_file(file_name):
    with open(file_name, "r") as data_file:
        data = data_file.read()
        return data


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


def analyze_keras_model(shared_model, action_model, value_model):

    log_stds = None
    observation_length = None
    with open("metadata.json", "r") as metadata_file:
        metadata = json.load(metadata_file)
        log_stds = np.array(metadata["log_stds"])
        observation_length = int(metadata["observation_length"])

    observation = np.array([469.956, 326.441, -0.0422514, 0.999107])
    observation = np.reshape(observation, (1, observation_length))
    # observation = np.reshape(np.array([0 for i in range(observation_length)]), (1,observation_length))
    print("OBSERVATION")
    print(observation)
    output = shared_model.predict(observation)
    # print(output)
    # print(output[0].shape)

    print("SHARED OUTPUT 1")
    print(output[0])
    print("SHARED OUTPUT 2")
    print(output[1])

    latent_action = output[0]
    latent_value = output[1]

    print(latent_action)
    action_data = action_model.predict(latent_action)
    action_length = len(action_data[0])
    print("action mean")
    print(action_data)
    value_data = value_model.predict(latent_value)
    print(value_data)
    cov_mat = np.zeros((action_length, action_length))
    print(cov_mat)
    for i in range(action_length):
        cov_mat[i][i] = math.exp(log_stds[i])
    print(cov_mat)

    if len(action_data) == 1:
        cov_mat = cov_mat[0][
            0
        ]  # in this case cov_mat is a 1x1 matrix which has a scalar data type

    print(cov_mat)
    print(action_data)
    mvn = multivariate_normal(mean=action_data[0], cov=cov_mat, allow_singular=False)
    sample = mvn.rvs(size=1)
    print(sample)
    prob = multivariate_normal.pdf(
        sample, mean=action_data[0], cov=cov_mat, allow_singular=False
    )
    print(prob)
    log_prob = math.log(prob)
    print(log_prob)
    return (action_data[0], value_data[0], log_prob)


def save_as_H5(model, vec_normalize, file_path="./"):

    print(model.observation_space.shape)
    total_model = pytorch_to_keras(
        model.policy.mlp_extractor,
        torch.zeros((1, model.observation_space.shape[0])),
        verbose=True,
        name_policy="renumerate",
    )
    action_model = pytorch_to_keras(
        model.policy.action_net, torch.zeros((1, 64)), verbose=True, name_policy="renumerate"
    )
    value_model = pytorch_to_keras(
        model.policy.value_net, torch.zeros((1, 64)), verbose=True, name_policy="renumerate"
    )

    total_model.save(file_path + "shared_policy.h5", save_format="h5")
    action_model.save(file_path + "action_policy.h5", save_format="h5")
    value_model.save(file_path + "value_policy.h5", save_format="h5")

    print(model.policy.log_std)

    metadata = {
        "log_stds": model.policy.log_std.data.detach().numpy().tolist(),
        "observation_length": sum(model.observation_space.shape),
        "action_length": sum(model.action_space.shape),
    }
    metadata["mean"] = vec_normalize["mean"]
    metadata["var"] = vec_normalize["var"]
    metadata["clip"] = vec_normalize["clip"]
    metadata["epsilon"] = vec_normalize["epsilon"]
    metadata["policy_type"]= "STABLE_BASELINES3_PPO"

    with open(file_path + "metadata.json", "w") as metadata_file:
        json.dump(metadata, metadata_file)


if __name__ == "__main__":
    if not len(sys.argv) == 2:
        print("usage: ./export_model.py <model folder path>")
        exit(1)

    model_path = sys.argv[1] + "/policy.zip"
    vec_normalize_path = sys.argv[1] + "/vector_normalize.json"

    model = PPO.load(model_path)
    vec_normalize = json.load(open(vec_normalize_path, "r"))
    save_as_H5(model, vec_normalize)

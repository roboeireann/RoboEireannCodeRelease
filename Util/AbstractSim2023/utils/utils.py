import json
import sys
sys.path.append(sys.path[0] + "/..")
from stable_baselines3.common.vec_env import VecNormalize


def get_string_from_file(file_name):
    with open(file_name, "r") as data_file:
        data = data_file.read()
        return data


def get_json_from_file_name(file_name):
    with open(file_name, "r") as json_file:
        return json.load(json_file)


def save_vec_normalize_data(vec_normalize, path):
    with open(path, "w") as output_file:
        data = {}
        data["mean"] = list(vec_normalize.obs_rms.mean)
        data["var"] = list(vec_normalize.obs_rms.var)
        data["clip"] = vec_normalize.clip_obs
        data["epsilon"] = vec_normalize.epsilon
        json.dump(data, output_file)

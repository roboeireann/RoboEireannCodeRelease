# Robocup 2023
This repository was used to train the goalie policy used by BadgerBots in Robocup 2023, and to train the defender policy used by RoboEireann in RoboCup 2024.

## Set up a python environment
Setting up a python environment is recommended as it ensures all dependencies are installed and using the correct versions.
Python 3.9 is required to run the code in this repository, ensure you have it installed before running the following commands.

````
python3.9 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
````

<!-- Some quick commands that are relevant to retraining/trying out this policy:


retrain policy

````
python generate_baselines.py kicking_goalkeeper_tune

````

visualize rollouts
````
python envs/kicking_goalkeeper_tune.py Models/kicking_goalkeeper_tune/

````
export policy to format for BadgerRLSystem2023 or RoboEireann2024 CodeRelease
````
python utils/export_model.py Models/kicking_goalkeeper_tune/

```` -->

To use this policy with BadgerRLSystem2023 or RoboEireann2024 CodeRelease, copy the created .h5 files and metadata.json to `Config/Policies/GoalKeeperKickPolicy` in the desired CodeRelease.

# AbstractSim

This repository contains the BadgerRL lab's abstract robosoccer environment simulators, as well as reference policies trained using these simulators. 
You can configure the main function of walk_to_goal.py, walk_to_ball.py, and push_ball_to_goal.py to either train a new policy, or load and demonstrate rollouts with an existing policy. 

## Pretrained Policies

Pre-trained Policies are available in the Models directory.


## Tools


### Trying out a policy

You can try out a policy with the PushBallToGoalEnv environment for example with the following command

````
python ./envs/push_ball_to_goal.py <path to models folder for policy>

specifically
python ./envs/push_ball_to_goal.py /Models/push_ball_to_goal

````

### export_model.py

export_model.py can be used to export a stable-baselines3 saved policy into a format that can be read by the C++ code base. 

When export_model.py is executed, it will create action_policy.h5, vale_policy.h5, shared_policy.h5, and metadata.json. All of these files need to be moved
to the Config directory of the C++ repository to try them out in the C++ environments. 

Note that export model needs both the vector normalize json and the policy file specified. 


### analyze_trajectories.py

This is provided for checking the similarity between the action_means recorded from the C++ policy during rollouts, and the 
action mean predictions made by the stable-baselines3 policy, and the converted keras models. An example trajectory file is provided in the Examples directory. 

### generate_baselines.py

This allows the baselines to be automatically trained. In the future, the abstract environemnts will be instrumented with psuedorandom seeding, so this process should be possible to exactly reproduce. Right now, there is some variation in the quality of the policies. 

To generate a policy run:
````
python generate_baselines.py <name of policy>

for example:
python generate_baselines.py push_ball_to_goal
````


## environment
````
python3.9 -m venv env
source env/bin/activate
python -m pip install -r requirements.txt

````

It is recommended to make a pythonvirtual env using requirements.txt before using the code in this repository, this will ensure you can load the saved policies, and ensure the 
policies you train are compatible with other people's environments. You will need to use python 3.9.

## Running tests

A limited test battery is provided with this project (particularly for logic involving trigonometric calculations) To run the tests, make sure you have the virtual environment set up, then execute the command:

````
python -m pytest
````

More tests are always welcome!

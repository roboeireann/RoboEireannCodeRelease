import json
import sys
sys.path.append(sys.path[0] + "/..")
import re
import os
import re


from utils.utils import get_string_from_file
import os




#gets the length of the first episode in the batch
# this is necessary because we don't want to use the truncated additional episodes in each batch
def get_episode_length( episode_starts):
    assert(episode_starts[0] == 1)
    try:
        return episode_starts[1:].index(1) + 1
    except:
        return len(episode_starts)

def merge_trajectories(input_folder_path, output_file_path, train_val_split):
    pattern = r"^trajectories_0_[0-9]+.json$"
    
    names = []
    for (path, _, file_names) in os.walk(input_folder_path):
        print(path)
        for file_name in file_names:
            print(file_name)
            names.append((path, file_name))
    print(names)


    names = sorted(names, key=lambda x: int(re.sub("[^0-9]", "", x[1])))

    train_names = []
    val_names = []

    for index in range(len(names)):
        if index < len(names)*train_val_split:
            train_names.append(names[index])
        else:
            val_names.append(names[index])

    #train_names.sort()
    #val_names.sort()

    #train_names = sorted(train_names, key=lambda x: int(re.sub("[^0-9]", "", x[1])))
    #val_names = sorted(val_names, key=lambda x: int(re.sub("[^0-9]", "", x[1])))

    merged_trajectories_train = {"length" : 0}
    merged_trajectories_val = {"length" : 0}

    ## Train names ##
    for name in train_names:
        print(name)
        if re.match(pattern, name[1]):
            trajectories = json.loads(
                get_string_from_file(os.path.join(name[0], name[1]))
            )


            episode_length = get_episode_length(trajectories['episode_starts'])
            assert episode_length <= 800

            print(episode_length)

            merged_trajectories_train["length"] += episode_length

            for key in trajectories.keys():
                if isinstance(trajectories[key], list):
                    if key in merged_trajectories_train:
                        merged_trajectories_train[key] += trajectories[key][:episode_length]
                    else:
                        merged_trajectories_train[key] = trajectories[key][:episode_length]

    with open(output_file_path, "w") as merged_trajectories_file:
        json.dump(merged_trajectories_train, merged_trajectories_file)

    # #Validation names ##
    for name in val_names:
        print(name)
        if re.match(pattern, name[1]):
            #print(name)
            trajectories = json.loads(
                get_string_from_file(os.path.join(name[0], name[1]))
            )


            episode_length = get_episode_length(trajectories['episode_starts'])
            
            print(episode_length)

            merged_trajectories_val["length"] += episode_length

            for key in trajectories.keys():

                # if isinstance(trajectories[key], list) and isinstance(trajectories[key][0],list):
                #    trajectories[key] = [entry[0] for entry in trajectories[key]]

                if isinstance(trajectories[key], list):
                    if key in merged_trajectories_val:
                        merged_trajectories_val[key] += trajectories[key][:episode_length]
                    else:
                        merged_trajectories_val[key] = trajectories[key][:episode_length]

    output_file_path = output_file_path[0:-5] + '_val.json'

    with open(output_file_path, "w") as merged_trajectories_file:
        json.dump(merged_trajectories_val, merged_trajectories_file)



if __name__ == "__main__":


    if not len(sys.argv) == 4:
        print("usage: merge_trajectories.py <path to folder> <output file name> <train_val_split>")

    merge_trajectories(sys.argv[1], sys.argv[2], sys.argv[3])

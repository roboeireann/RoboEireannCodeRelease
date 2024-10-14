/**
 * @file RLConfig.h
 * 
 * This file is used to configure the RL Algorithm and Environment.
 * This file reads the configuration from a json file and sets the parameters.
 * 
 * @author BadgerBots
 */

#ifndef RL_CONFIG
#define RL_CONFIG


#include <stdio.h>
#include <string>
#include <unistd.h>
#include <fstream>
#include "Tools/json.h"
#include <libgen.h>         // dirname
#include "Platform/File.h"

#define PATH_MAX 4096




#include <linux/limits.h>
#include <mutex>

#define PATH_MAX 4096

static std::string getConfigDirectory()
{
  std::string filePrefix = File::getBHDir();
  filePrefix += "/Config/rl_config.json";
  std::cout << "Config Directory: " << filePrefix << std::endl;
  return filePrefix;
}


namespace RLConfig
{
 static std::string configPath = getConfigDirectory();
 static std::ifstream configFile(configPath);
 static json::value configData = json::parse(configFile);
 static std::string mode = to_string(configData["mode"]);
 static bool train_mode = to_bool(configData["train_mode"]);
 static bool deterministic = to_bool(configData["deterministic"]);
 static bool referee_enabled = to_bool(configData["referee_enabled"]);
 static int action_steps = std::stoi(to_string(configData["action_steps"]));
 static int episode_length = std::stoi(to_string(configData["episode_length"]));
 static int batch_size = std::stoi(to_string(configData["batch_size"]));
 static int epoch_count = std::stoi(to_string(configData["epoch_count"]));
 static int seed = std::stoi(to_string(configData["seed"]));
 static bool normalization = to_bool(configData["normalization"]);
 static bool shieldEnabled = to_bool(configData["shield_enabled"]);
 static bool debug_print = to_bool(configData["debug_print"]);
 static bool visualization_mode = to_bool(configData["visualization_mode"]);
 static bool logging = to_bool(configData["logging"]);
//  extern std::mutex resetLock;
 extern bool resetting;
}

#endif

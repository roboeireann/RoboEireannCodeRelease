/** 
 *	@file ObsTools.h
 *
 *	This file implements tools for getting observations from the environment, to be used for observation inputs to RL policies.
 *
 *	@author BadgerBots
*/
#ifndef ObsTools_h
#define ObsTools_h

#include "RLConfig.h"

#include <vector>

std::vector<float> get_relative_observation(std::vector<float> agent_loc, std::vector<float> object_loc);
bool isFacingPoint(float x, float y, float angle);
bool isFacingMidfield(float x, float y, float angle);

#endif


/**
 * @file RLEnv.h
 * 
 * This file implements the Environment class which is used to create the environment for the RL algorithm.
 * The environment is modelled on the robots perception of its surroundings.
 * 
 * @author BadgerBots
 * @author Conal Hughes
 */

#ifndef RLEnv_h
#define RLEnv_h

#include "Representations/Modeling/RobotPose.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/Configuration/FieldDimensions.h"

#include "Tools/json.h"
#include "Tools/RL/RLConfig.h"

extern bool RLConfig::debug_print;
extern std::string RLConfig::mode;

struct FieldPositions {
  float virtual_ball_X_position;
  float virtual_ball_Y_position;
  float dummy_defender_1_X_position;
  float dummy_defender_1_Y_position;
  float dummy_defender_2_X_position;
  float dummy_defender_2_Y_position;

  FieldPositions(const float virtual_ball_X_position, const float virtual_ball_Y_position,
                  const float dummy_defender_1_X_position, const float dummy_defender_1_Y_position,
                  const float dummy_defender_2_X_position, const float dummy_defender_2_Y_position):
  virtual_ball_X_position (virtual_ball_X_position), virtual_ball_Y_position (virtual_ball_Y_position),
  dummy_defender_1_X_position (dummy_defender_1_X_position), dummy_defender_1_Y_position (dummy_defender_1_Y_position),
  dummy_defender_2_X_position (dummy_defender_2_X_position), dummy_defender_2_Y_position (dummy_defender_2_Y_position) {}

  FieldPositions(const FieldPositions &field_positions) {
    virtual_ball_X_position = field_positions.virtual_ball_X_position;
    virtual_ball_Y_position = field_positions.virtual_ball_Y_position;
    dummy_defender_1_X_position = field_positions.dummy_defender_1_X_position;
    dummy_defender_1_Y_position = field_positions.dummy_defender_1_Y_position;
    dummy_defender_2_X_position = field_positions.dummy_defender_2_X_position;
    dummy_defender_2_Y_position = field_positions.dummy_defender_2_Y_position;
  }
};    // struct FieldPositions

class Environment {
private:
// says unused but needs to be here
  unsigned int observation_length;
  unsigned int action_length;

  FieldPositions field_positions;

  std::vector<float> observation_vector;
  std::vector<float> current_action;


public:
  Environment (const FieldPositions field_positions, const int observation_size, const int action_size);

  std::vector<float> getFloatVectorFromJSONArray(const json::value &json_value);
  bool shouldReset(GroundTruthRobotPose pose);

  // std::vector<float> getObservation(GroundTruthRobotPose pose);
  std::vector<float> getObservation(RobotPose pose, FieldBall ball, FieldDimensions fieldDimensions);


  std::vector<float> getPredictedPosition(RobotPose theRobotPose, std::vector<float> action);


  void setFieldPositions(const float virtual_ball_X_position, const float virtual_ball_Y_position,
                         const float dummy_defender_1_X_position, const float dummy_defender_1_Y_position,
                         const float dummy_defender_2_X_position, const float dummy_defender_2_Y_position);

  unsigned int getActionLength() { return action_length; }

  void setCurrentAction(std::vector<float> currentAction) {
    current_action = std::vector<float>(currentAction);
  }

  std::vector<float> getCurrentAction() { return current_action; }
  std::vector<float> getCurrentObservation() { return observation_vector; }
};    // class Environment

#endif /* RLEnv_h */

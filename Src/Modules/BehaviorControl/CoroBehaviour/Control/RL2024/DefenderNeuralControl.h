/**
 * @file NeuralControl.h
 *
 * This file implements an implementation for the NeuralControl skill.
 *
 * @author Arne Hasselbring (the actual behavior is older)
 * @author John Balis
 * @author Chen Li
 * @author Benjamin Hong
 * @author Conal Hughes
 */

#pragma once

#include "CompiledNN/CompiledNN.h"
#include "CompiledNN/Model.h"
#include "CompiledNN/SimpleNN.h"
#include "CompiledNN/Tensor.h"

#include "Tools/RL/RLConfig.h"
#include "Tools/RL/RLAlg.h"
#include "Tools/RL/RLData.h"
#include "Tools/RL/RLEnv.h"
#include "Tools/RL/ObsTools.h"

#include "Tools/json.h"

#include "Representations/BehaviorControl/Skills.h"

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Configuration/FieldDimensions.h"

#include <cmath>
#include <stdio.h>
#include <iostream>

#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/ColorRGBA.h"
#include "Tools/Debugging/DebugDrawings3D.h"

// my additions
#include "Modules/BehaviorControl/CoroBehaviour/CoroBehaviourCommon.h"
#include "Modules/BehaviorControl/CoroBehaviour/2024/CoroBehaviour2024.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/CommonSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/BallSkills.h"
#include "Representations/BehaviorControl/PlayerRole.h"
#include "Platform/File.h"

#define PI 3.14159265f

namespace CoroBehaviour
{
namespace RL2024
{
  // =====================================================================
  CRBEHAVIOUR(DefenderSearchForBallTask)
  {
    CRBEHAVIOUR_INIT(DefenderSearchForBallTask) {}

    void operator()(bool shouldBackUp = false)
    {
      CRBEHAVIOUR_BEGIN();

      if (shouldBackUp)
      {
        // try backing up a few steps in case we lost the ball in a duel
        CR_CHECKPOINT(backUp);
        while (getCheckpointDuration() < paramBackupDurationMs)
        {
          headSkills.lookActive();
          motionSkills.walkAtRelativeSpeed(Pose2f(0, -0.8f, 0)); // walk backwards at full speed
          if (theFieldBall.ballWasSeen())
            CR_EXIT_SUCCESS();
          else
            CR_YIELD();
        }
      }

      chooseTurnDirection();
      CR_CHECKPOINT(turn);
      while (getCheckpointDuration() < paramTurnDurationMs)
      {
        lookLeftAndRightTask(turnDirection);
        motionSkills.walkAtRelativeSpeed(Pose2f(turnDirection, 0.f, 0)); // turn on the spot
        if (theFieldBall.ballWasSeen())
          CR_EXIT_SUCCESS();
        else
          CR_YIELD();
      }

      CR_CHECKPOINT(gotoTacticPose);
      while (true)
      {
        // if we still haven't found the ball, go to a tactic pose
        headSkills.lookActive();
        walkToPoseAutoAvoidanceTask(getTacticPose(), Pose2f(0.8f, 1.0f, 1.0f));
        if (theFieldBall.ballWasSeen())
          CR_EXIT_SUCCESS();
        else
          CR_YIELD();
      }

    }

  private:
    unsigned paramBackupDurationMs = 4000;
    unsigned paramTurnDurationMs = 2000;

    READS(FieldBall);
    READS(RobotPose);

    HeadSkills headSkills {env};
    MotionSkills motionSkills {env};

    WalkToPoseAutoAvoidanceTask walkToPoseAutoAvoidanceTask {env};

    LookLeftAndRightTask lookLeftAndRightTask {env};

    int turnDirection; // 1 or -1

    void chooseTurnDirection()
    {
      // Always turn in-field (towards the centre of the field) first
      // - theRobotPose is the robot relative to the field origin 
      //   and theRobotPose.inversePose is the field origin relative to the robot
      // - get the bearing to the field origin point (theRobotPose.inversePose.translation) 
      //   from the robot. (It will lie between -180 and 180 degrees)
      // - if it is between 0 and -180 the field origin is clockwise from the robot
      //   otherwise it is anticlockwise
      Angle originBearing = theRobotPose.inversePose.translation.angle();

      turnDirection = (originBearing > 0) ? 1 : -1;      
    }   

    Pose2f getTacticPose()
    {
      // TODO - implement this
      Pose2f tacticPose = Pose2f(0.0f, -2500.0f, 0.0f);
      // robot relative
      return tacticPose*theRobotPose.inversePose;

    }
  };

// =======================================================================================================================

  CRBEHAVIOUR(DefenderNeuralControlTask)
  {
    CRBEHAVIOUR_INIT(DefenderNeuralControlTask){}

    void operator()(void)
    {
      
      CRBEHAVIOUR_LOOP()
      {
        updatePolicies();
        CR_CHECKPOINT(pathPlanning);
        while(theFieldBall.ballWasSeen(params.ballSeenTimeoutMs))
        {
          updateModel();
          distance = std::sqrt(std::pow(theFieldBall.recentBallEndPositionOnField().x() - theRobotPose.translation.x(), 2.f) + std::pow(theFieldBall.recentBallEndPositionOnField().y() - theRobotPose.translation.y(), 2.f));
          // move the robot based off the model outputs
          if (def_algorithm.getActionMeans()[3] > 0.75 && kicking)// 4 model outputs contains a kick threshold, if it is above 0.7, kick
          {
            gotoBallAndKickBestOptionTask();
          }

          else if (def_algorithm.getActionMeans()[3] > 0.75)
          {
            commonSkills.lookActive();
            walkToPoseAutoAvoidanceTask(getTargetPose(), Pose2f(0.8f, 1.0f, 1.0f));
            kicking = (distance < 1500);
          }
          else // either move or dribble
          {
            kicking = false;
            commonSkills.lookActive();
            walkToPoseAutoAvoidanceTask(getTargetPose(), Pose2f(0.8f, 1.0f, 1.0f));
          }
          CR_YIELD();
        }        
      }
    }
  private:
  DEFINES_PARAMS(DefenderNeuralControlTask, 
    {, 
      (unsigned)(5000) ballSeenTimeoutMs, // if the ball is not seen for this time we consider it lost
      (unsigned)(6000) ballLostRecentlyTimeoutMs, // if the ball *was* seen within this period we consider the ball to be recently lost
      (int)(14) def_observation_size,
      (int)(4) def_action_size,
    });
    READS(RobotPose);
    READS(TeamBehaviorStatus);
    READS(FieldBall);
    READS(FieldDimensions);
    READS(ObstacleModel);
    READS(ActiveTactic);

    CommonSkills commonSkills {env};

    WalkToPoseAutoAvoidanceTask walkToPoseAutoAvoidanceTask {env};
    DefenderSearchForBallTask defenderSearchForBallTask {env};
    RE2024::GotoBallAndKickBestOptionTask gotoBallAndKickBestOptionTask {env};

    bool kicking = false;
    float distance = 0;

    std::string getPolicyDirectory()
    {
      std::string filePrefix = File::getBHDir();
      filePrefix += "/Config/Policies/";
      std::cout << "Config Directory: " << filePrefix << std::endl;
      return filePrefix;
    }

    std::string def_policy_path = getPolicyDirectory();

    FieldPositions field_positions{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    Environment environment{field_positions, params.def_observation_size, params.def_action_size};
    Algorithm def_algorithm = Algorithm(def_policy_path, "PositionalDefenderPolicy");
    
    void updateModel()
    {
      const std::vector<NeuralNetwork::TensorLocation> &shared_input = def_algorithm.getSharedModel()->getInputs();
      std::vector<NeuralNetwork::TensorXf> observation = updateInputObservation(shared_input, shared_input.size());
      std::vector<NeuralNetwork::TensorXf> action_output = def_algorithm.inference(observation);
      std::vector<float> tempCurrentAction = std::vector<float>(def_algorithm.computeCurrentAction(action_output, def_algorithm.getActionLength()));
    }

    // updates the algorithm models
    void updatePolicies()
    {
      if (def_algorithm.getCollectNewPolicy())
      {
        def_algorithm.waitForNewPolicy();

        if (!def_algorithm.getUndefinedPolicy())
        {
          def_algorithm.deleteModels();
        }
        
        def_algorithm.updateModels();

        if (RLConfig::train_mode) {
            def_algorithm.deletePolicyFiles();
        }

        def_algorithm.setCollectNewPolicy(false);
        def_algorithm.setUndefinedPolicy(false);
      }
    }

    // return max or min if number is outside the range
    float clampOutput(float number, float low, float high)
    {
        return number <= low ? low : number >= high ? high : number;
    }

    // Use the ObstacleModel to determine if any robots should be considered obstacles
    std::vector<Vector2f> getObstaclePositions()
    {
        std::vector<Vector2f> obstacles;
        for (auto obstacle : theObstacleModel.obstacles)
        {
          // check if the obstacle is a robot
          if (obstacle.type != Obstacle::someRobot)
            continue;

          // get distance from robot to obstacle
          double distance = std::sqrt(std::pow(obstacle.center.x() - theRobotPose.translation.x(), 2) + std::pow(obstacle.center.y() - theRobotPose.translation.y(), 2));
          if(distance < 2000)
          {
            obstacles.push_back(Vector2f(obstacle.center.x(), obstacle.center.y()));
          }
        }
        // sort from closest to farthest
        std::sort(obstacles.begin(), obstacles.end(), [this](Vector2f a, Vector2f b) {
          double distanceA = std::sqrt(std::pow(a.x() - theRobotPose.translation.x(), 2) + std::pow(a.y() - theRobotPose.translation.y(), 2));
          double distanceB = std::sqrt(std::pow(b.x() - theRobotPose.translation.x(), 2) + std::pow(b.y() - theRobotPose.translation.y(), 2));
          return std::abs(distanceA) < std::abs(distanceB);
        });

        if (obstacles.size() > 2)
        {
          return std::vector<Vector2f>(obstacles.begin(), obstacles.begin() + 2);
        }
        else
        {
          return obstacles;
        }
    }

    // Use the model outputs to determine the target pose
    Pose2f getTargetPose()
    {
        float robotX = theRobotPose.translation.x();
        float robotY = theRobotPose.translation.y();
        float angle = theRobotPose.rotation;
        float policy_rotation = (float)(def_algorithm.getActionMeans())[0] * 0.3f;

        float policy_target_x = robotX + (
            (
                ((float)cos(angle) * clampOutput((float)(def_algorithm.getActionMeans())[1], -1.0f, 1.0f)) // translationX
                + ((float)cos(angle + PI / 2) * clampOutput((float)(def_algorithm.getActionMeans())[2], -1.0f, 1.0f)) // translationY
            )
            * 200.0f
        );
        // extract the y component of the translation
        float policy_target_y = robotY + (
            (
                ((float)sin(angle) * clampOutput((float)(def_algorithm.getActionMeans())[1], -1.0f, 1.0f))
                + ((float)sin(angle + PI / 2) * clampOutput((float)(def_algorithm.getActionMeans())[2], -1.0f, 1.0f))
            )
            * 200.0f
        );
        Vector2f target = Vector2f(policy_target_x, policy_target_y);
        // get distance from base position to target
        Pose2f tacticPoseOnField = theActiveTactic.formationPose;
        float distance = std::sqrt(std::pow(tacticPoseOnField.translation.x() - target.x(), 2.f) + std::pow(tacticPoseOnField.translation.y() - target.y(), 2.f));

        // FIXME
        // GUARD RAILS IDEALLY WANT THESE GONE

        // if the distance is greater than 1000, split the difference with weighted average towards base position
        if (distance > 500)
        {
            target = (target + target + tacticPoseOnField.translation) / 3;
        }

        Vector2f relativeTarget = theRobotPose.inversePose * target;
        return Pose2f(policy_rotation, relativeTarget[0], relativeTarget[1]);
    }

    // refresh the input observation vector
    std::vector<NeuralNetwork::TensorXf> updateInputObservation(const std::vector<NeuralNetwork::TensorLocation> &shared_input, unsigned long size)
    {
      std::vector<NeuralNetwork::TensorXf> observation_input{size};

      // get field dimensions
      float fieldLength = abs(theFieldDimensions.xPosOwnGroundLine) + abs(theFieldDimensions.xPosOpponentGroundLine);
      float fieldWidth = abs(theFieldDimensions.yPosRightSideline) + abs(theFieldDimensions.yPosLeftSideline);


      for (std::size_t i = 0; i < observation_input.size(); ++i)
      {
          observation_input[i].reshape(shared_input[i].layer->nodes[shared_input[i].nodeIndex].outputDimensions[shared_input[i].tensorIndex]);
      }
      std::vector<float> rawObservation = def_algorithm.normalizeObservation(environment.getObservation(theRobotPose, theFieldBall, theFieldDimensions));

      for (int i = 2; i < params.def_observation_size; i++) {
          observation_input[0][i] = rawObservation[i-2];
      }

      // if an defender -- will need 2 obstacles, or teammate positions(ish) 
      std::vector<Vector2f> obstacles = getObstaclePositions();
      std::vector<float> opponent_positions;
      for (std::size_t i = 0; i < obstacles.size(); i++)
      {
        // get relative x and y
        opponent_positions.push_back((obstacles[i].x() - theRobotPose.translation.x()) / fieldLength);
        opponent_positions.push_back((obstacles[i].y() - theRobotPose.translation.y()) / fieldWidth);
      }
      for (std::size_t i = 2; i < opponent_positions.size(); i++)
      {
        observation_input[0][i] = opponent_positions[i-2];
      }

      // base position as the first 2 elements
      Pose2f tacticPoseOnField = theActiveTactic.formationPose;
      observation_input[0][0] = (tacticPoseOnField.translation.x() - theRobotPose.translation.x()) / fieldLength;
      observation_input[0][1] = (tacticPoseOnField.translation.y() - theRobotPose.translation.y()) / fieldWidth;

      return observation_input;
    }
  };
}
}
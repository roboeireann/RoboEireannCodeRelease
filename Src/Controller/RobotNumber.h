/**
 * @file RobotNumber.h
 * 
 * Simple class to manage mapping of robot numbers to/from scene numbers
 * 
 * @author Rudi Villing
 */

#pragma once

#include "Platform/BHAssert.h"


class RobotNumber
{
public:
  static const int numSubstitutePlayersPerTeam = 1;  
  static const int numPlayersPerTeam = 20;//7 + numSubstitutePlayersPerTeam; // incl keeper
  static const int numFieldPlayersPerTeam = numPlayersPerTeam - 1 - numSubstitutePlayersPerTeam;
  static const int numRobots = numPlayersPerTeam * 2; // 2 teams

  static const int team2Offset = 20;

  static void validateSceneNumber(int sceneNumber)
  {
    if (sceneNumber > numPlayersPerTeam)
      sceneNumber -= team2Offset;

    ASSERT(1 <= sceneNumber && sceneNumber <= numPlayersPerTeam);
  }


  static bool isFirstTeamFromScene(int sceneNumber)
  {
    return sceneNumber <= numPlayersPerTeam;
  }

  static bool isFirstTeamFromIndex(int index)
  {
    return index < numPlayersPerTeam;
  }

  static bool isSubstituteFromScene(int sceneNumber)
  {
    if (sceneNumber > numPlayersPerTeam)
      sceneNumber -= team2Offset;
    
    return numPlayersPerTeam + 1 - numSubstitutePlayersPerTeam <= sceneNumber && sceneNumber <= numPlayersPerTeam;
  }

  static bool isSubstituteFromIndex(int index)
  {
    if (index >= numPlayersPerTeam)
      index -= numPlayersPerTeam;
    
    return numPlayersPerTeam - numSubstitutePlayersPerTeam <= index && index < numPlayersPerTeam;
  }

  /**
   * get the player number (as it would appear on the robot's jersey)
   */
  static int getPlayerNumberFromScene(int sceneNumber)
  {
    int player = (sceneNumber > team2Offset) ? sceneNumber - team2Offset : sceneNumber;

    return player;
  }

  /**
   * get the zero-based array index from the scene number of the robot
   * i.e. robot1 -> index 0. Robots for team 2 occupy array slots immediately
   * following those for team 1.
   */
  static int getRobotIndexFromScene(int sceneNumber)
  {
    int index = ((sceneNumber > team2Offset) ? sceneNumber - team2Offset + numPlayersPerTeam : sceneNumber) - 1;

    return index;
  }

  /**
   * get the scene number of the robot from the array index
   */
  static int getSceneNumberFromIndex(int index)
  {
    int sceneNumber = (index < numPlayersPerTeam) ? index + 1 : team2Offset + index + 1;

    return sceneNumber;
  }

  /**
   * get the player number of the robot from the array index
   */
  static int getPlayerNumberFromIndex(int index)
  {
    int player = (index < numPlayersPerTeam) ? index + 1 : index - numPlayersPerTeam + 1;

    return player;
  }

};
/**
 * @file BehaviourParams.h
 * 
 * Common parameters for Behaviour to simplify loading from one place and
 * to provide some loadable parameters separate from parameters defined
 * in code that are not loadable
 * 
 * @author Rudi Villing
 */

#pragma once

#include "KickInfo.h"

#include "Streaming/AutoStreamable.h"


STREAMABLE(REBehaviourParams,
{
  /***** separate header from streamable members, note comma at end *****/,

  // ballPlayer specific
  (unsigned)(4000) ballPlayerSearchBackupMs, ///< how long to back up when searching for the ball before turning

  // kick at goal
  // NOTE: that manipulating the ranges here can also determine whether a kick is enabled at all and also
  //       how far away the robot will attempt a shot on goal.
  //       Ensure that at least one walk kick is enabled - usually the walkForwardsKick.
  (Rangef)(2000.f, 6000.f) forwardFastLongKickAtGoalRange,  ///< enabled over this range, (0,0) means disabled always
  (Rangef)(1600.f, 2500.f) forwardFastKickAtGoalRange,      ///< enabled over this range, (0,0) means disabled always
  (Rangef)(800.f, 2000.f) walkForwardsLongKickAtGoalRange,  ///< enabled over this range, (0,0) means disabled always
  (Rangef)(0.f, 1000.f) walkForwardsKickAtGoalRange,        ///< enabled over this range, (0,0) means disabled always
  (float)(750.f) longShotAtGoalMinX, ///< long shots at goal only allowed forward of this point

  // penalty shots
  (float)(1.f) penaltyShotApproachSpeed, ///< the approach speed as the robot walks from distance to the ball
  (float)(0.7f) penaltyShotFinalLineUpSpeed, ///< the walk speed during the final line up to kick

  // goalie
  (Vector2f)(1600.f, 1500.f) goalieClearanceBox, // goalie will try to kick away within this size of goal center

  // demo related params
  (int)(0) demoDuellingDelayMs, ///< a delay in taking actions while duelling, only applied to the weaker team for demos, 0 is disabled
  (bool)(false) demoGoalieArmsOutstretchedPenalty, ///< whether the goalie should have its arms outstretched during a penalty shot
  (bool)(false) demoSlowerStrikerWalkSpeed, ///< whether the striker should walk slower during a penalty shot
});

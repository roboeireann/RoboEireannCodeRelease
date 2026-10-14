/**
 * @file Representations/Infrastructure/RobotSafetyState.h
 *
 * Represents whether the robot in an emergency stop state or not.
 *
 * @author Rudi Villing
 */

#pragma once

STREAMABLE(RobotSafetyState,
{
  ENUM(State,
  {,
    safetyTripped, // unstiff / motor torques disabled
    resetPending, // reset from emergencyStop mode - awaiting button release
    operational, // motors may be actuated normally
  });

  /**** separate header from streamable fields - comma at end of comment ****/,

  // we need the defaults here to be suitable for use on any platform (whether
  // the platform supports e-stop, e.g. Booster K1, or not, e.g. Nao). Each
  // robot platform will set its own appropriate initialisation values
  (State)(operational) state,
  (bool)(true) hardwareConnected, ///< is the interface to hardware working fine?
  (unsigned)(0) timeWhenStateStarted,
});

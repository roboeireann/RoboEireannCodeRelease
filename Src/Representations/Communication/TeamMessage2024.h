/**
 * @file TeamMessage2024.h
 * 
 * A representation of the data that would be exchanged between robots.
 * 
 * We expect this to diverge further and further from simply compressing
 * representations as they are used internally, so the information here is custom
 * and will be copied piece by piece to/from representations. Moreover the line
 * coding/compression will be customized per member so the only reason we use
 * the streamable form is so that it can be logged and exchanged with SimRobot.
 *
 * @author Rudi Villing
 */

#pragma once

#include "Representations/BehaviorControl/PlayerRole.h"
#include "Representations/Modeling/RobotPose.h"

#include "Tools/Function.h"
#include "Tools/Streams/AutoStreamable.h"


/**
 * A representation of TeamMessage that we send to/receive from teammates
 */
STREAMABLE(TeamMessage2024,
{
  using PlayerNum = int; /* generally 1..numPlayers, 0 or -1 means not used/set  */

  STREAMABLE(MinimalRobotPose,
  {,
    (Pose2f) pose,
    (float) translationSD, /* std dev of translation */
    (float) rotationSD,
    (RobotPose::LocalizationQuality) quality,
  });

  STREAMABLE(MinimalBallModel,
  {,
    /* ball estimate from sender's own observations only; sent even if the ball is not currently seen */
    (Vector2f) position,      /**< The position of the ball relative to the robot (in mm)*/
    (Vector2f) velocity,      /**< The velocity of the ball relative to the robot (in mm/s)*/
    (float) positionSD, /* std dev of position instead of covariance, same units as position */

    (Vector2f) positionLastSeen, /**< (called lastPerception in BallModel) The last seen position of the ball */
    (unsigned) frameTimeLastSeen, /**< Time stamp when the ball last observed */
  });

  STREAMABLE(MinimalBehaviourStatus,
  {,
    //(PlayerNum)(-1) passTarget, /* -1 means none */
    (Vector2f)(Vector2f::Zero()) walkingTo, ///< robot relative coords
    (float)(0.f) speed, ///< abs speed in mm/sec
    /* shootingTo unused for now */
  });

  STREAMABLE(MinimalWhistle,
  {,
    // (bool)(false) detectionActive,
    // (float)(0.f) confidenceLastWhistleDetection,
    (unsigned)(0) frameTimeLastWhistleDetection,
  });

  STREAMABLE(MinimalStandbyReadyGesture,
  {,
    (unsigned)(0) frameTimeLastDetection,
  });

  // we don't transmit TeamBehaviorStatus any more, but instead reconstruct the TeamBehaviorStatus from the ActiveTacticStatus

  // we transmit the ActiveTacticStatus as is since it is a small representation

  // virtual unsigned toLocalTimestamp(unsigned remoteTimestamp) const { return 0u; }
  

  /* ----------------- separation from header to streamable members ------------------ */,

  /* header */ /* not stored as a member, but included in the formatted transmission packet */
  /* version */ /* not stored as a member, but included in the formatted transmission packet */
  /* teamId - as provided by SPL organizers */ /* not stored as a member, but included in the formatted transmission packet */

  (PlayerNum) playerNum,

  (uint8_t) seqNumGC, /* last seq num received from GC by sending player */
  (unsigned) timeReceivedGC, /* local timestamp at sending player when last packet received from GC */

  // (uint8_t) seqNum, /* TODO - NOT USED CURRENTLY - the sending player's sequence number for this message (wrap at 15, i.e. 0,1...13,14,0...) */
  (unsigned) timestamp, /* local timestamp at sending player when this packet was generated - the reference for all relative timestamps in codec */

  /* robot status */
  (bool) isPenalized, /* TODO: get this from GC? */
  (bool) isUpright, /* not fallen, lifted, or otherwise off feet */
  // (unsigned) frameTimeWhenLastUpright, - TODO probably not used anywhere so not needed

  (bool) kickedSinceRestart, ///< if the robot has touched the ball since the last restart in play

  (MinimalRobotPose) robotPose,
  (MinimalBallModel) ballModel,

  /* frame info */
  (unsigned) frameTime,

  /* TODO: obstacleModel is excluded for now */

  (MinimalBehaviourStatus) behaviourStatus,
  // (MinimalTeamBehaviourStatus) teamBehaviourStatus,
  (ActiveTacticStatus) activeTacticStatus,
  (MinimalWhistle) whistle,
  (MinimalStandbyReadyGesture) standbyReadyGesture,
});


/**
 * This is the representation that saves the last team message sent by this
 * robot. This is useful so that we can detect how much the current state has
 * changed from what we last sent (to determine if we should send a new message)
 */
STREAMABLE(SentTeamMessage2024,
{
  /* ----------------- separation from header to streamable members (note comma at end) ------------------ */,

  (TeamMessage2024) teamMessage, //< the team message to be sent (only updated if send was actually needed)
  (unsigned)(0) frameTime,  //< the frame time when this message was sent
});


/**
 * This is the representation that can generate the message bytes,
 * that this robot wants to send to its teammates.
 */
STREAMABLE(TeamMessage2024OutputGenerator,
{
  FUNCTION(void ()) sendIfNeeded; //< check if message needs to be sent and send it if it does
  
  /* ----------------- separation from header to streamable members (note comma at end) ------------------ */,

  (TeamMessage2024) teamMessage, //< the team message to be sent (only updated if send was actually needed)
  (unsigned)(0) sentMessages,  //< count of sent messages
  // (bool)(false) shouldSendThisFrame, //< should send this frame (determined by provider)
});

/**
 * @file Representations/BehaviorControl/BehaviorStatus.h
 * The file declares a struct that contains data about the current behavior state.
 * @author Andreas Stolpmann
 */

#pragma once

#include "Tools/Communication/BHumanTeamMessageParts/BHumanMessageParticle.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Enum.h"

/**
 * @struct BehaviorStatus
 * A struct that contains data about the current behavior state.
 */
STREAMABLE(BehaviorStatus, COMMA public BHumanCompressedMessageParticle<BehaviorStatus>
{
  ENUM(Activity,
  {,
    unknown,

    afterInterceptBall,
    fallen,
    finished,
    initial,
    lookAroundAfterPenalty,
    set,
    generalPlay,

    codeReleaseKickAtGoal,
    codeReleasePositionForKickOff,

    calibrationFinished,

    rePlaceholder,
    reStandStill,
    rePositionForKickOff,
    reKickoff,
    reKickAtGoal,
    reSearchForBall,

    reGoalieWaiting,
    reGoalieSave,
    reKickBestOption,
    reGotoPose,
    rePenaltyShot,

    reStandAndLookActive,
    reWalkToTacticPose,
    reStandAtTacticPose,

    reSetPlayBackoffAndStand,
    reIndirectKick,

    reIntercept,
    
    // IMPORTANT - add additional statuses to the end - don't insert in the list above
  });

  /**
   * Checks whether the activity indicates that the robot is searching for the ball.
   * @return Whether the activity indicates that the robot is searching for the ball.
   */
  bool searchesForBall() const
  {
    return false;
  },

  (Activity)(unknown) activity, /**< What is the robot doing in general? */
  (int)(-1) passTarget,
  (Vector2f)(Vector2f::Zero()) walkingTo,
  (float) speed, /**< The absolute speed in mm/s. */
  (Vector2f)(Vector2f::Zero()) shootingTo,
});

typedef BehaviorStatus BehaviourStatus; // alias with "correct" spelling
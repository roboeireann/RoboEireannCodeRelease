/**
 * Request for the key frame engine.
 * @author <a href="mailto:simont@tzi.de>Simon Taddiken</a>
 */
#pragma once
#include "Tools/RobotParts/Arms.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/EnumIndexedArray.h"

/**
 * Class that represents the possible arm motions that can be requested from
 * the robot.
 */
STREAMABLE(ArmKeyFrameRequest,
{
  /** Existing arm motions. Ordering must match ordering in armMotionEngine.cfg */
  ENUM(ArmKeyFrameId,
  {,
    // IF U TOUCH THIS MAKE SURE U TAKE CARE TO SAVE THE ARMS CORRECTLY IN A FALLING CASE
    useDefault,     /**< No explicit arm motion, so WalkingEngine's arm angles will be used */
    back,           /**< Arms positioned to the back */
    raiseArm,       /**< Raise the arm straight up. */
    lowerArm,       /**< Lowers the arm straight down safely and slowly */
    lowerArmRelaxed,       /**< hold the arm in the lowered position with reduced stiffness (hack instead of releasing keyframe) */
    lowerArmSafely,       /**< Lowers the arm straight down safely and slowly */
    pointSideUp,        /**< Arm is pointed upwards to the side at a 45deg angle */
    pointSideDown,      /**< Arm is pointed downwards to the side at a 45deg angle */
    pointSideStraight,    /**< Holds arm horizontally to the side */
    pointToGroundAhead,  /**< Points to the ground (Straight arm 45deg in front of robot) */
    pointAcrossChest,    /**< Bends the arm and points across the robot's chest */
    keeperStand,    /**< Arm position for the keeper when guarding the goal */
    fullTimeWide,       /**< full time arms wide apart */
    fullTimeTogether,   ///< full time arms together
    // Add any new states before this line. Do not alter the next lines and make sure these are the last motions
    repeat,         ///< repeat the most recently requested motion. - FIXME (RV) this doesn't work correctly yet
    reverse,        /**< Reverse current arm keyframe motion. MAKE SURE THIS IS THE LAST STATE!!!!*/
  });

  STREAMABLE(Arm,
  {,
    (ArmKeyFrameRequest::ArmKeyFrameId)(useDefault) motion, /**< Motion to execute */
    (bool)(false) fast, /**< Whether states should not be interpolated */
  }),

  (ENUM_INDEXED_ARRAY(Arm, Arms::Arm)) arms,
});

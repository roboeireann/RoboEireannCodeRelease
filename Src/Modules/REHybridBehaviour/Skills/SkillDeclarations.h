/**
 * @file SkillDeclarations.h
 *
 * declares the Skill tasks that will be included in the behaviour
 * and possibly called from other tasks. The behaviour impl/definitions are found
 * in the .cpp files.
 *
 * This file should only be included inside the container/executor class for
 * the tasks. 
 */


// ============================================================================
// ArmMotionRequestSkills
// ============================================================================
CORO_DECLARE(pointAtTask, ARGS((const Vector3f&, localPoint), (Arms::Arm, arm, Arms::anyArm)));

CORO_DECLARE(keyFrameArmsTask, ARGS((ArmKeyFrameRequest::ArmKeyFrameId, motion), (Arms::Arm, arm), (bool, fast, false)));

CORO_DECLARE(armContactTask, ARGS((Arms::Arm, arm, Arms::bothArms)));

HSM_DECLARE_DELEGATOR(armContactLeftTask);
HSM_DECLARE_DELEGATOR(armContactRightTask);
HSM_DECLARE_DELEGATE(armContactArmDelegate, ARGS((Arms::Arm, arm)));

bool keyFrameArmHelper(ArmKeyFrameRequest::ArmKeyFrameId motion, Arms::Arm arm, bool fast = false);
bool keyFrameArmsHelper(ArmKeyFrameRequest::ArmKeyFrameId motion, Arms::Arm arm, bool fast = false);


// ============================================================================
// HeadMotionRequestSkills
// ============================================================================

CORO_DECLARE(headPlayDeadTask);

CORO_DECLARE(lookActiveTask,
    ARGS((bool, withBall, false),
        (bool, ignoreBall, false), // ignoreBall overrides withBall - only one should be true
        (bool, onlyOwnBall, false), // ignore team ball if true
        (bool, fixTilt, false), // use default fixed tilt value
        (float, slowdownFactor, 1.f))); // [0-1] lower values slow down the head

CORO_DECLARE(lookAtAnglesTask, 
    ARGS((Angle, pan),
        (Angle, tilt),
        (float, speed, 180_deg),
        (HeadMotionRequest::CameraControlMode, camera, HeadMotionRequest::autoCamera),
        (bool, calibrationMode, false)));

HSM_DECLARE(lookAtBallTask,
       ARGS((HeadMotionRequest::CameraControlMode, camera, HeadMotionRequest::autoCamera)));

CORO_DECLARE(lookAtPointTask,
       ARGS((const Vector3f&, target),
            (HeadMotionRequest::CameraControlMode, camera, HeadMotionRequest::autoCamera),
            (Angle, speed = 180_deg)));

CORO_DECLARE(lookForwardTask);

HSM_DECLARE(lookLeftAndRightTask,
       ARGS((bool, startLeft, true),
            (Angle, maxPan, 50_deg),
            (Angle, tilt, 49_deg),
            (Angle, speed, 100_deg)));


// ============================================================================
// InterceptSkills
// ============================================================================

CORO_DECLARE(keyFrameInterceptTask,
    ARGS((MotionRequest::Dive::Request, interceptMotion), (unsigned, getUpDelay, 0)));

CORO_DECLARE(interceptTask,
    ARGS((unsigned, interceptsEnabled), (bool, aggressive, false), (unsigned, getUpDelay, 0)));


/// ============================================================================
// MotionRequestSkills
// ============================================================================

HSM_DECLARE(playDeadTask);
HSM_DECLARE(prepareTask);

HSM_DECLARE(standTask, ARGS((bool, high, false), (bool, energySavingWalk, true)));

HSM_DECLARE(diveTask, ARGS((MotionRequest::Dive::Request, request)));

HSM_DECLARE(walkAtAbsoluteSpeedTask,
           ARGS((const Pose2f &, speed), (bool, energySavingWalk, false)));

HSM_DECLARE(walkAtRelativeSpeedTask,
           ARGS((const Pose2f &, speed), (bool, energySavingWalk, false)));

CORO_DECLARE(walkToBallAndKickTask,
       ARGS((Angle, targetDirection),
            (KickInfo::KickType, kickType),
            (KickPrecision, alignPrecisely, KickPrecision::notPrecise),
            (float, kickLength, std::numeric_limits<float>::max()),
            (const Pose2f&, speed, {1.f, 1.f, 1.f}),
            (const MotionRequest::ObstacleAvoidance&, obstacleAvoidance, {}),
            (PreStepType, preStepType, PreStepType::allowed),
            (bool, turnKickAllowed, true),
            (bool, shiftTurnKickPose, false),
            (const Rangea&, directionPrecision, {0_deg, 0_deg})));

HSM_DECLARE(walkToPoseBasicTask,
       ARGS((const Pose2f&, target),
            (const Pose2f&, speed, {1.f, 1.f, 1.f}),
            (const MotionRequest::ObstacleAvoidance&, obstacleAvoidance, {}),
            (bool, keepTargetRotation, false),
            (const std::optional<Vector2f>&, targetOfInterest, {}),
            (SideWalkingRequest::SideWalkingRequest, sideWalkingRequest, SideWalkingRequest::allowed),
            (bool, energySavingWalk, false)))

ENUM(Avoidance,
{,
  AVOIDANCE_NONE, // rough = true, disableObstacleAvoidance = true, toBall = true
  AVOIDANCE_ROUGH, // rough = true, disableObstacleAvoidance = true (i.e. libwalk avoidance), toBall = true
  AVOIDANCE_AUTO, // rough = false, disableObstacleAvoidance = false (path planner or libwalk as needed), toBall = false
  AVOIDANCE_AUTO_RELAXED, // rough = false, disableObstacleAvoidance = false, toBall = true
});

CORO_DECLARE(walkToPoseTask,
    ARGS((const Pose2f &, target), (float, speed), (Avoidance, avoidance, AVOIDANCE_AUTO),
        (bool, keepTargetRotation, false), (bool, standAtTarget, true),
        (Rangef, positionThreshold, {20.f, 100.f}), (Rangea, angleThreshold, {3_deg, 10_deg})));

// ============================================================================
// SoundsSkills
// ============================================================================

HSM_DECLARE(playSoundTask, ARGS((const std::string &, soundName), (bool, force, false)));
HSM_DECLARE(sayTask, ARGS((const std::string &, text), (bool, force, false), (float, speed, 1.f)));

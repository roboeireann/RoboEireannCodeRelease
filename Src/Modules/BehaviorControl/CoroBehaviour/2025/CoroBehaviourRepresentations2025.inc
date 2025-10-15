/**
 * @file CoroBehaviourRepresentations.inc
 *
 * Parts of this file are included in different places, hence .inc instead of .h
 * 
 * The idea is that every time we add/remove a representation we just edit
 * this file instead of editing multiple files.
 * 
 * YOU MUST DO THE FOLLOWING FOR EVERY REPRESENTATION YOU ADD
 * 
 * 1. Add a header file for the representation
 * 2. Add it to the behaviour representations using X_REQUIRES, X_USES, etc.
 *    This will add it to the module and will make it accessible from coro behaviours.
 *
 * FIXME
 * 2. Add a REQUIRES/PROVIDES for the representation
 * 3. Add an ADD_READABLE or ADD_MODIFIABLE for the representation
 *
 * @author Rudi Villing
 */

#ifdef CORO_REPRESENTATION_INCLUDES

  // ------------------------------------------------------------------------
  // Group the header files by source code folder This makes it easier to 
  // track if something is missed at any stage. There needs to be a #include
  // for every representation used by the module (further below)
  // ------------------------------------------------------------------------

  // REQUIRED by CoroBehaviourModule (and the coros it calls)

  #include "Representations/Configuration/BallSpecification.h"
  #include "Representations/Configuration/FieldDimensions.h"
  #include "Representations/Configuration/KickInfo.h"
  #include "Representations/Configuration/RobotDimensions.h"
  #include "Representations/Configuration/Formations.h"
  #include "Representations/Configuration/Tactics.h"
  #include "Representations/Configuration/BehaviourParams.h"
  #include "Representations/Configuration/GameConfig.h"

  #include "Representations/Infrastructure/CameraStatus.h"
  #include "Representations/Infrastructure/ExtendedGameInfo.h"
  #include "Representations/Infrastructure/FrameInfo.h"
  #include "Representations/Infrastructure/JointAngles.h"
  #include "Representations/Infrastructure/RobotHealth.h"
  #include "Representations/Infrastructure/SensorData/JointSensorData.h"
  #include "Representations/Infrastructure/SensorData/KeyStates.h"

  #include "Representations/MotionControl/HeadMotionInfo.h"
  #include "Representations/MotionControl/MotionInfo.h"
  #include "Representations/MotionControl/WalkingEngineOutput.h"
  #include "Representations/MotionControl/ArmMotionInfo.h"
  #include "Representations/MotionControl/WalkStepData.h"

  #include "Representations/Communication/GameInfo.h"
  
  #include "Representations/Communication/TeamData.h"

  #include "Representations/Sensing/FallDownState.h"
  #include "Representations/Sensing/ArmContactModel.h"
  #include "Representations/Sensing/FootBumperState.h"
  #include "Representations/Sensing/FootSupport.h"

  #include "Representations/Modeling/BallModel.h"
  #include "Representations/Modeling/ObstacleModel.h"
  #include "Representations/Modeling/RobotPose.h"
  #include "Representations/Modeling/TeamBallModel.h"
  #include "Representations/Modeling/BallDropInModel.h"
  #include "Representations/Modeling/TeammatesLocationModel.h"

  #include "Representations/BehaviorControl/FieldBall.h"
  #include "Representations/BehaviorControl/Libraries/LibCheck.h"
  #include "Representations/BehaviorControl/Libraries/LibLookActive.h"
  #include "Representations/BehaviorControl/Libraries/LibWalk.h"
  #include "Representations/BehaviorControl/PathPlanner.h"
  #include "Representations/BehaviorControl/TeamBehaviorStatus.h"
  #include "Representations/BehaviorControl/ActiveTactic.h"

  #include "Representations/MotionControl/OdometryData.h"

  #include "Representations/Perception/StandbyReadyGesture.h"

  // PROVIDED by CoroBehaviourModule (and the coros it calls)

  #include "Representations/BehaviorControl/ActivationGraph.h"
  #include "Representations/BehaviorControl/BehaviorStatus.h"

  #include "Representations/MotionControl/ArmMotionRequest.h"
  #include "Representations/MotionControl/HeadMotionRequest.h"
  #include "Representations/MotionControl/MotionRequest.h"

  #include "Representations/Infrastructure/TeamTalk.h"

  #include "Representations/Configuration/CalibrationRequest.h"

#endif // CORO_REPRESENTATION_INCLUDES



#if defined(CORO_REPRESENTATIONS_MODULE)
  #define X_REQUIRES(cls_) REQUIRES(cls_),
  #define X_USES(cls_) USES(cls_),
  #define X_REQUIRES_PROVIDES(cls_) REQUIRES(cls_), PROVIDES(cls_),
  #define X_PROVIDES(cls_) PROVIDES(cls_),
#elif defined(CORO_REPRESENTATIONS_FOR_BEHAVIOUR)
  #define X_REQUIRES(cls_) representationRegistry.add(the##cls_);
  #define X_USES(cls_) representationRegistry.add(the##cls_);
  #define X_REQUIRES_PROVIDES(cls_) representationRegistry.addModifiable(const_cast<cls_&>(the##cls_));
  #define X_PROVIDES(cls_) representationRegistry.addModifiable(the##cls_);
#endif

#if defined(CORO_REPRESENTATIONS_MODULE) || defined(CORO_REPRESENTATIONS_FOR_BEHAVIOUR)

  #ifdef CORO_REPRESENTATIONS_MODULE
  // don't forget to update the year as necessary
  MODULE(CoroBehaviourModule2025,
  {,
  #endif

    // ----------------------- begin edit representations list ----------------------------------

    X_REQUIRES(FieldDimensions)
    X_REQUIRES(KickInfo)
    X_REQUIRES(BallSpecification)
    X_REQUIRES(RobotDimensions)
    // X_REQUIRES(BehaviourFormations)
    X_REQUIRES(Formations)
    X_REQUIRES(Tactics)
    X_REQUIRES(BehaviourParams)
    X_REQUIRES(GameConfig)

    X_REQUIRES(CameraStatus)
    X_REQUIRES(RobotHealth)
    X_REQUIRES(FrameInfo)
    X_REQUIRES(EnhancedKeyStates)
    X_REQUIRES(JointSensorData)
    X_REQUIRES(JointAngles)
    X_REQUIRES(ExtendedGameInfo)

    X_REQUIRES(MotionInfo)
    X_REQUIRES(HeadMotionInfo)
    X_REQUIRES(WalkingEngineOutput)
    X_REQUIRES(ArmMotionInfo)

    X_REQUIRES(GameInfo)
    X_REQUIRES(ReceivedGameControlData)
    X_REQUIRES(TeamData)

    X_REQUIRES(FallDownState)
    X_REQUIRES(ArmContactModel)
    X_REQUIRES(FootBumperState)

    X_REQUIRES(RobotPose)
    X_REQUIRES(BallModel)
    X_REQUIRES(TeamBallModel)
    X_REQUIRES(ObstacleModel)
    X_REQUIRES(BallDropInModel)
    X_REQUIRES(TeammatesLocationModel)

    X_REQUIRES(FieldBall)
    X_REQUIRES(LibCheck)
    X_REQUIRES(LibLookActive)
    X_REQUIRES(LibWalk)
    X_REQUIRES(PathPlanner)
    X_REQUIRES(TeamBehaviorStatus)
    X_REQUIRES(ActiveTactic)

    X_REQUIRES(OdometryData)

    X_REQUIRES(StandbyReadyGesture)

    X_REQUIRES_PROVIDES(ActivationGraph)

    X_PROVIDES(ArmMotionRequest)
    X_PROVIDES(BehaviorStatus)
    X_PROVIDES(HeadMotionRequest)
    X_PROVIDES(MotionRequest)
    X_PROVIDES(TeamTalk)
    X_PROVIDES(CalibrationRequest)

    // ----------------------- end edit representations list ----------------------------------

  #ifdef CORO_REPRESENTATIONS_MODULE

    LOADS_PARAMETERS(
    {,
      /// this names the top level (coro) behaviour to run - it must have been 
      /// registered already in the registerBehaviours function
      (std::string) topLevelBehaviour, 
    }),
  });
  
  #endif // CORO_REPRESENTATIONS_MODULE

  #undef X_REQUIRES
  #undef X_USES
  #undef X_REQUIRES_PROVIDES
  #undef X_PROVIDES

#endif // CORO_REPRESENTATIONS_MODULE || CORO_REPRESENTATIONS_FOR_BEHAVIOUR

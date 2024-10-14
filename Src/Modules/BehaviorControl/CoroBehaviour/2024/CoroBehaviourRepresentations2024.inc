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

  #include "Representations/Communication/GameInfo.h"
  
  #include "Representations/Communication/TeamData.h"

  #include "Representations/Sensing/FallDownState.h"
  #include "Representations/Sensing/ArmContactModel.h"
  #include "Representations/Sensing/FootBumperState.h"

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

#ifdef CORO_REPRESENTATIONS_MODULE

  // don't forget to update the year if necessary
  MODULE(CoroBehaviourModule2024, 
  {,
    REQUIRES(FieldDimensions),
    REQUIRES(KickInfo),
    REQUIRES(BallSpecification),
    REQUIRES(RobotDimensions),
    // REQUIRES(BehaviourFormations),
    REQUIRES(Formations),
    REQUIRES(Tactics),
    REQUIRES(BehaviourParams),
    REQUIRES(GameConfig),

    REQUIRES(CameraStatus),
    REQUIRES(RobotHealth),
    REQUIRES(FrameInfo),
    REQUIRES(EnhancedKeyStates),
    REQUIRES(JointSensorData),
    REQUIRES(JointAngles),
    REQUIRES(ExtendedGameInfo),

    REQUIRES(MotionInfo),
    REQUIRES(HeadMotionInfo),
    REQUIRES(WalkingEngineOutput),
    REQUIRES(ArmMotionInfo),

    REQUIRES(GameInfo),
    REQUIRES(ReceivedGameControlData),
    REQUIRES(TeamData),

    REQUIRES(FallDownState),
    REQUIRES(ArmContactModel),
    REQUIRES(FootBumperState),

    REQUIRES(RobotPose),
    REQUIRES(BallModel),
    REQUIRES(TeamBallModel),
    REQUIRES(ObstacleModel),
    REQUIRES(BallDropInModel),
    REQUIRES(TeammatesLocationModel),

    REQUIRES(FieldBall),
    REQUIRES(LibCheck),
    REQUIRES(LibLookActive),
    REQUIRES(LibWalk),
    REQUIRES(PathPlanner),
    REQUIRES(TeamBehaviorStatus),
    REQUIRES(ActiveTactic),

    REQUIRES(OdometryData),

    REQUIRES(StandbyReadyGesture),

    PROVIDES(ActivationGraph),
    REQUIRES(ActivationGraph),

    PROVIDES(ArmMotionRequest),
    PROVIDES(BehaviorStatus),
    PROVIDES(HeadMotionRequest),
    PROVIDES(MotionRequest),
    PROVIDES(TeamTalk),
    PROVIDES(CalibrationRequest),

    LOADS_PARAMETERS(
    {,
      /// this names the top level (coro) behaviour to run - it must have been 
      /// registered already in the registerBehaviours function
      (std::string) topLevelBehaviour, 
    }),
  });

#endif // CORO_REPRESENTATIONS_MODULE

#ifdef CORO_REPRESENTATIONS_FOR_BEHAVIOUR

  #define ADD_READABLE(cls) representationRegistry.add(the##cls)
  #define ADD_MODIFIABLE(cls) representationRegistry.addModifiable(the##cls)

  // register representations that will be used by (non-team) coro behaviours.
  // Use ADD_READABLE for any REQUIRES/USES line in the module above
  // Use ADD_MODIFIABLE for any PROVIDES line in the module above

  ADD_READABLE(FieldDimensions);
  ADD_READABLE(KickInfo);
  ADD_READABLE(BallSpecification);
  ADD_READABLE(RobotDimensions);
  // ADD_READABLE(BehaviourFormations);
  ADD_READABLE(Formations);
  ADD_READABLE(Tactics);
  ADD_READABLE(BehaviourParams);
  ADD_READABLE(GameConfig);

  ADD_READABLE(CameraStatus);
  ADD_READABLE(RobotHealth);
  ADD_READABLE(FrameInfo);
  ADD_READABLE(EnhancedKeyStates);
  ADD_READABLE(JointSensorData);
  ADD_READABLE(JointAngles);
  ADD_READABLE(ExtendedGameInfo);

  ADD_READABLE(MotionInfo);
  ADD_READABLE(HeadMotionInfo);
  ADD_READABLE(WalkingEngineOutput);
  ADD_READABLE(ArmMotionInfo);

  ADD_READABLE(GameInfo);
  ADD_READABLE(ReceivedGameControlData);
  ADD_READABLE(TeamData);

  ADD_READABLE(FallDownState);
  ADD_READABLE(ArmContactModel);
  ADD_READABLE(FootBumperState);

  ADD_READABLE(RobotPose);
  ADD_READABLE(BallModel);
  ADD_READABLE(TeamBallModel);
  ADD_READABLE(ObstacleModel);
  ADD_READABLE(BallDropInModel);
  ADD_READABLE(TeammatesLocationModel);

  ADD_READABLE(FieldBall);
  ADD_READABLE(LibCheck);
  ADD_READABLE(LibLookActive);
  ADD_READABLE(LibWalk);
  ADD_READABLE(PathPlanner);
  ADD_READABLE(TeamBehaviorStatus);
  ADD_READABLE(ActiveTactic);

  ADD_READABLE(OdometryData);

  ADD_READABLE(StandbyReadyGesture);

  // modifiables
  
  representationRegistry.addModifiable(const_cast<ActivationGraph &>(theActivationGraph));
  ADD_MODIFIABLE(ArmMotionRequest);
  ADD_MODIFIABLE(BehaviorStatus);
  ADD_MODIFIABLE(HeadMotionRequest);
  ADD_MODIFIABLE(MotionRequest);
  ADD_MODIFIABLE(TeamTalk);

  #undef ADD_READABLE
  #undef ADD_MODIFIABLE

#endif // CORO_REPRESENTATIONS_FOR_BEHAVIOUR
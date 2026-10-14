/**
 * @file REHybridBehaviourModule.h
 *
 * This is the module that runs the behaviour of a robot using the
 * HybridBehaviourSystem
 *
 * @author Rudi Villing
 */

#pragma once

// required representations - behaviour inputs

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/FieldInterceptBall.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/Libraries/LibLookActive.h"
#include "Representations/BehaviorControl/Libraries/LibWalk.h"
#include "Representations/BehaviorControl/PathPlanner.h"

#include "Representations/Communication/RETeamData.h"
#include "Representations/Communication/GameControllerData.h"
#include "Representations/Communication/RETeamMessage2025.h"

#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/BehaviorParameters.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/KickInfo.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Configuration/REBehaviourParams.h"
#include "Representations/Configuration/REGameConfig.h"
#include "Representations/Configuration/REFormations.h"
#include "Representations/Configuration/RETactics.h"

#include "Representations/Infrastructure/CameraStatus.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/RobotHealth.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"

#include "Representations/Modeling/BallDropInModel.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/RETeammatesLocationModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/TeamBallModel.h"

#include "Representations/MotionControl/ArmMotionInfo.h"
#include "Representations/MotionControl/HeadMotionInfo.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"

#include "Representations/Perception/RefereeGestures/REStandbyReadyGesture.h"

#include "Representations/Sensing/ArmContactModel.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/FootBumperState.h"
#include "Representations/Sensing/GroundContactState.h"


// PROVIDED representations - behaviour outputs

#include "Representations/BehaviorControl/ActivationGraph.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/REActiveTactic.h"
#include "Representations/BehaviorControl/TeamBehaviorStatus.h"

#include "Representations/Configuration/CalibrationRequest.h"

#include "Representations/MotionControl/ArmMotionRequest.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Representations/MotionControl/MotionRequest.h"

#include "Tools/BehaviorControl/Interception.h"

#include "Tools/REBehaviour/HybridBehaviour.h"
#include "Tools/RECommon/OnceFlag.h"
#include "Tools/RECommon/Hysteresis.h"
#include "Tools/RECommon/EdgeTrigger.h"

#include "Framework/Module.h"
#include "Streaming/Enum.h"

#include "Platform/SystemCall.h"
#include "Debugging/Annotation.h"
#include "TextLogging/TextLogging.h" // include here to make it easily accessible to tasks
#include "Tools/RECommon/FmtCommonTypes.h"

#include <string>


#define HBS_ANNOTATION(...)  ANNOTATION_FMT("HbsBehaviour", __VA_ARGS__) 


MODULE(REHybridBehaviourModule,
{,
  REQUIRES(FieldBall),
  REQUIRES(FieldInterceptBall),
  REQUIRES(LibCheck),
  // REQUIRES(LibDemo),
  REQUIRES(LibLookActive),
  // REQUIRES(LibPosition),
  REQUIRES(LibWalk),
  REQUIRES(PathPlanner),

  REQUIRES(RETeamData),
  REQUIRES(GameControllerData),
  // REQUIRES(RETeamMessage2025),

  REQUIRES(BallSpecification),
  REQUIRES(BehaviorParameters), // B-Human's
  REQUIRES(FieldDimensions),
  REQUIRES(KickInfo),
  REQUIRES(RobotDimensions),

  REQUIRES(REBehaviourParams),
  REQUIRES(REGameConfig),
  REQUIRES(REFormations),
  REQUIRES(RETactics),

  REQUIRES(CameraStatus),
  REQUIRES(GameState),
  REQUIRES(ExtendedGameState),
  REQUIRES(FrameInfo),
  REQUIRES(JointAngles),
  REQUIRES(RobotHealth),
  REQUIRES(JointSensorData),
  REQUIRES(EnhancedKeyStates),

  REQUIRES(BallDropInModel),
  REQUIRES(BallModel),
  REQUIRES(ObstacleModel),
  REQUIRES(RETeammatesLocationModel),
  REQUIRES(RobotPose),
  REQUIRES(TeamBallModel),


  REQUIRES(ArmMotionInfo),
  REQUIRES(HeadMotionInfo),
  REQUIRES(OdometryData),
  REQUIRES(MotionInfo),
  REQUIRES(WalkingEngineOutput),

  REQUIRES(REStandbyReadyGesture),

  REQUIRES(FallDownState),
  REQUIRES(FootBumperState),
  REQUIRES(ArmContactModel),
  REQUIRES(GroundContactState),

  // provided outputs

  REQUIRES(ActivationGraph),
  PROVIDES(ActivationGraph),

  PROVIDES(REActiveTactic),
  PROVIDES(TeamBehaviorStatus),

  PROVIDES(CalibrationRequest),
  PROVIDES(BehaviorStatus),
  PROVIDES(ArmMotionRequest),
  PROVIDES(HeadMotionRequest),
  PROVIDES(MotionRequest),

  LOADS_PARAMETERS(
  {,
    (std::string) teamTask, ///< the one and only coordination task
    (std::vector<std::string>) fallbackPlayerTasks, ///< one or more tasks that will be tried in order until one progresses beyond INITIAL
  }),
});


class REHybridBehaviourModule : public REHybridBehaviourModuleBase
{
public:
  /** Constructor. */
  REHybridBehaviourModule();

  // needed only because we want to continue supporting cards and skills
  /**
   * Creates extended module info (union of this module's info and requirements of all behavior parts (cards and skills)).
   * @return The extended module info.
   */
  // static std::vector<ModuleBase::Info> getExtModuleInfo();


protected:
  TeamBehaviorStatus theTeamBehaviorStatus;     ///< The team behavior status that is modified by the behavior.
  REActiveTactic theREActiveTactic;     ///< the Active Tactic modified by behaviour

  BehaviorStatus theBehaviorStatus;     ///< The behavior status that is modified by the behavior.
  CalibrationRequest theCalibrationRequest; ///< The camera calibration request that is modified by the behavior.
  ArmMotionRequest theArmMotionRequest; ///< The arm motion request that is modified by the behavior.
  HeadMotionRequest theHeadMotionRequest; ///< The head motion request that is modified by the behavior.
  MotionRequest theMotionRequest;         ///< The motion request that is modified by the behavior.

  OnceFlag startupAnnouncement;

  /**
   * Updates the activation graph and executes the main behaviour
   * @param activationGraph The provided activation graph.
   */
  void update(ActivationGraph &activationGraph) override;

  void update(REActiveTactic &activeTactic) override { activeTactic = theREActiveTactic; }
  void update(TeamBehaviorStatus &teamBehaviorStatus) override { teamBehaviorStatus = theTeamBehaviorStatus; }


  // simply copy the values that were filled in when update(ActivationGraph...) was executed
  void update(CalibrationRequest& CalibrationRequest) override { CalibrationRequest = theCalibrationRequest; }
  void update(BehaviorStatus &behaviorStatus) override { behaviorStatus = theBehaviorStatus; }
  void update(ArmMotionRequest &armMotionRequest) override { armMotionRequest = theArmMotionRequest; }
  void update(HeadMotionRequest &headMotionRequest) override { headMotionRequest = theHeadMotionRequest; }
  void update(MotionRequest &motionRequest) override { motionRequest = theMotionRequest; }

  void draw();

  // -------------------------------------------------------------------------
  // Hybrid behaviour integration
  // -------------------------------------------------------------------------
protected: // everything that should be shared with tasks should be protected
  Hbs::TickTrace tickTrace; // Our tasks will write to tickTrace and we'll convert to ActivationGraph after

  HBS_ENV(&tickTrace);
  HBS_TASK_REGISTRY(REHybridBehaviourModule); 

  // include task declarations (or inline definitions) below this line
  #include "CommonControl/CommonControlDeclarations.h"
  #include "Skills/SkillDeclarations.h"
  #include "Control/ControlDeclarations.h"
};

using BehaviourContainer = REHybridBehaviourModule;
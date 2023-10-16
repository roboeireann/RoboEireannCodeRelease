/*
 * @file: CoroBehaviour.h
 *
 * General include file for all (single robot) Coroutine based behaviour tasks
 *
 * @author: Rudi Villing
 */


#pragma once

#include "CoroBehaviourRegistry.h"

// common representations used by shared skills

// // REQUIRED by CoroBehaviourModule (and the coros it calls)

#include "Representations/Configuration/BallSpecification.h"
// #include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/KickInfo.h"
// #include "Representations/Configuration/RobotDimensions.h"

// #include "Representations/Infrastructure/CameraStatus.h"
// #include "Representations/Infrastructure/ExtendedGameInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
// #include "Representations/Infrastructure/RobotHealth.h"
// #include "Representations/Infrastructure/SensorData/JointSensorData.h"
// #include "Representations/Infrastructure/SensorData/KeyStates.h"
// #include "Representations/Infrastructure/WhistleProcessed.h"

#include "Representations/MotionControl/HeadMotionInfo.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/MotionControl/ArmMotionInfo.h"

#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/Communication/TeamInfo.h"
#include "Representations/Communication/TeamData.h"

#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/ArmContactModel.h"

// #include "Representations/Perception/RefereeGesture.h"

#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/RobotPose.h"
// #include "Representations/Modeling/TeamBallModel.h"

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/Libraries/LibLookActive.h"
#include "Representations/BehaviorControl/Libraries/LibWalk.h"
#include "Representations/BehaviorControl/PathPlanner.h"
#include "Representations/BehaviorControl/TeamBehaviorStatus.h"
// #include "Representations/BehaviorControl/KickoffState.h"

// #include "Representations/MotionControl/OdometryData.h"

// // PROVIDED by CoroBehaviourModule (and the coros it calls)

// #include "Representations/BehaviorControl/ActivationGraph.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"

#include "Representations/MotionControl/ArmMotionRequest.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Representations/MotionControl/MotionRequest.h"

// #include "Representations/Infrastructure/TeamTalk.h"

// #include "Representations/Configuration/CalibrationRequest.h"


// cards and skills compatibility
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Skill/Skill.h"

// -----------------------------------------------------------
// macros
// -----------------------------------------------------------


// NOTE: remember to register any coro behaviours you will be calling
// like this in CoroBehaviourModuleYYYY::registerEntryPointBehaviours
#define CALL_BASIC_CORO(name) CoroBehaviourRegistry::theInstance->getBasic(name)()
#define DECLARE_CALLS_BASIC_CORO(varName, name)                                                                        \
  CoroBehaviourRegistry::Basic &varName { CoroBehaviourRegistry::theInstance->getBasic(name) }

// more cards and skills compatibility
// We rely on the SkillRegistry and CardRegistry being created in coro
// behaviour module before any coro behaviour attempts to use these macros

/// call a skill without declaring it first
/// example: CALL_SKILL(Say)("Hello world");
#define CALL_SKILL(name) (*SkillRegistry::theInstance->getSkill<Skills::name##Skill>(#name))

/// call a card without declaring it first
/// example: CALL_CARD(KickoffCard);
#define CALL_CARD(name) (CardRegistry::theInstance->getCard(#name)->call())

/// declare a card or skill variable that can be accessed without looking it up
/// on each tick. (Tiny performance gain)
#define DECLARE_CALLS_CARD(name)  CardBase& the##name = *(CardRegistry::theInstance->getCard(#name));
#define DECLARE_CALLS_SKILL(name) Skills::name##Skill& the##name##Skill = *(SkillRegistry::theInstance->getSkill<Skills::name##Skill>(#name))
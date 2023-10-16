/**
 * @file CoroBehaviourModule2022.h
 *
 * This is the module that runs the behaviour for a single robot.
 * Because of its dependencies, it expects a team behaviour module
 * to run first.
 *
 * From this file, the behaviour uses the coro behaviour architecture.
 *
 * The implementation is based in part on the BH2021 code release but
 * modified for the coro behaviour architecture
 *
 * @author Rudi Villing
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Tools/Streams/Enum.h"

#include <string>

#define CORO_REPRESENTATION_INCLUDES
#include "CoroBehaviourRepresentations2023.inc"
#undef CORO_REPRESENTATION_INCLUDES

#define CORO_REPRESENTATIONS_MODULE
#include "CoroBehaviourRepresentations2023.inc"
#undef CORO_REPRESENTATIONS_MODULE


// Include here so module macros do not dismantle themself

// CoroBehaviour.h also includes all the representations required/provided by this module
#include "CoroBehaviour2023.h"

// skills we use in the top level behaviour

#include "Modules/BehaviorControl/CoroBehaviour/Skills/BehaviorStatusSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/CommonSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/MotionSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/SoundSkills.h"

class CoroBehaviourModule2023 : public CoroBehaviourModule2023Base
{
public:
  ENUM(Status, 
  {,
    inactive,
    sittingDown,
    gettingUp,
    cameraStatusFailed,
    lowBattery,
    penalized,
    playing,
  });

  /** Constructor. */
  CoroBehaviourModule2023();

  // needed only because we want to continue supporting cards and skills
  /**
   * Creates extended module info (union of this module's info and requirements of all behavior parts (cards and skills)).
   * @return The extended module info.
   */
  static std::vector<ModuleBase::Info> getExtModuleInfo();


private:
  Status status = inactive;
  Status prevStatus = inactive;

  bool firstFrame = true;
  unsigned int statusStartTime = 0;

  ArmMotionRequest theArmMotionRequest; /**< The arm motion request that is modified by the behavior. */
  BehaviorStatus theBehaviorStatus;     /**< The behavior status that is modified by the behavior. */
  HeadMotionRequest theHeadMotionRequest; /**< The head motion request that is modified by the behavior. */
  MotionRequest theMotionRequest;         /**< The motion request that is modified by the behavior. */
  TeamTalk theTeamTalk;                   /**< The team talk that is modified by the behavior. */
  CalibrationRequest theCalibrationRequest; /**< The camera calibration request that is modified by the behavior. */

  CoroBehaviour::BehaviourEnv env;
  CoroBehaviour::CoroBehaviourRegistry theCoroBehaviourRegistry; // there can only be one instance in this thread

  CoroBehaviour::HeadSkills headSkills{env};
  CoroBehaviour::MotionSkills motionSkills{env};
  CoroBehaviour::CommonSkills commonSkills{env};

  // allow some compatibility with cards and skills based behaviours

  SkillRegistry theSkillRegistry; /**< The manager of all skills. */
  CardRegistry theCardRegistry;   /**< The manager of all cards. */

  /**
   *  helper function to ensure all representations registered with BehaviourEnv
   * prior to any skills or behaviours constructing and trying to access the
   * representations
   */
  CoroBehaviour::BehaviourEnv::RepresentationRegistry initRegisteredRepresentations();

  /**
   * helper to register all entry point behaviours that may be specified using config files
   */
  void registerEntryPointBehaviours();

  // helper functions associated with running one tick of the behaviour engine

  void executeBehaviourTick();
  void checkUsbMountedButtonRequest();
  void checkJointTemperatureButtonRequest();


  /**
   * Updates the activation graph and executes the main behaviour
   * @param activationGraph The provided activation graph.
   */
  void update(ActivationGraph &activationGraph) override;

  // simply copy the values that were filled in when update(ActivationGraph...) was executed
  void update(ArmMotionRequest &armMotionRequest) override { armMotionRequest = theArmMotionRequest; }
  void update(BehaviorStatus &behaviorStatus) override { behaviorStatus = theBehaviorStatus; }
  void update(HeadMotionRequest &headMotionRequest) override { headMotionRequest = theHeadMotionRequest; }
  void update(MotionRequest &motionRequest) override { motionRequest = theMotionRequest; }
  void update(TeamTalk &teamTalk) override { teamTalk = theTeamTalk; }
  void update(CalibrationRequest& CalibrationRequest) override { CalibrationRequest = theCalibrationRequest; }
};

/**
 * @file CoroBehaviourRepresentations.inc
 *
 * Parts of this file are included in different places, hence .inc instead of .h
 * 
 * The idea is that every time we add/remove a representation we just edit
 * this file instead of editing multiple files. For each representation:
 * - add it to the list of includes
 * - add it to the team behaviour representations using X_REQUIRES, X_USES, etc.
 *   This will add it to the module and will make it accessible from coro behaviours.
 *
 * @author Rudi Villing
 */

#ifdef TEAM_REPRESENTATION_INCLUDES

  // ------------------------------------------------------------------------
  // Group the header files by source code folder This makes it easier to 
  // track if something is missed at any stage. There needs to be a #include
  // for every representation used by the module (further below)
  // ------------------------------------------------------------------------

  // REQUIRED by TeamBehaviourModule (and the coros it calls)

  #include "Representations/Configuration/FieldDimensions.h"
  #include "Representations/Configuration/RobotDimensions.h"
  #include "Representations/Configuration/Formations.h"
  #include "Representations/Configuration/Tactics.h"

  #include "Representations/Infrastructure/FrameInfo.h"

  #include "Representations/MotionControl/MotionInfo.h"
  #include "Representations/MotionControl/MotionRequest.h"
  #include "Representations/MotionControl/WalkingEngineOutput.h"

  #include "Representations/Communication/GameInfo.h"
  #include "Representations/Communication/TeamData.h"
  #include "Representations/Communication/TeamMessage2025.h"

  #include "Representations/Infrastructure/ExtendedGameInfo.h"

  #include "Representations/Sensing/FallDownState.h"
  #include "Representations/Sensing/GroundContactState.h"

  #include "Representations/Modeling/RobotPose.h"
  #include "Representations/Modeling/TeammatesLocationModel.h"


  // #include "Representations/BehaviorControl/BehaviorStatus.h"
  #include "Representations/BehaviorControl/FieldBall.h"
  #include "Representations/BehaviorControl/Libraries/LibCheck.h"

  // PROVIDED/MODIFIED - behaviour outputs

  #include "Representations/BehaviorControl/ActivationGraph.h"
  #include "Representations/BehaviorControl/TeamBehaviorStatus.h"
  #include "Representations/BehaviorControl/ActiveTactic.h"

#endif // TEAM_REPRESENTATION_INCLUDES

//====================================================================================================================

#if defined(TEAM_REPRESENTATIONS_MODULE)
  #define X_REQUIRES(cls_) REQUIRES(cls_),
  #define X_USES(cls_) USES(cls_),
  #define X_REQUIRES_PROVIDES(cls_) REQUIRES(cls_), PROVIDES(cls_),
  #define X_PROVIDES(cls_) PROVIDES(cls_),
#elif defined(TEAM_REPRESENTATIONS_FOR_BEHAVIOUR)
  #define X_REQUIRES(cls_) representationRegistry.add(the##cls_);
  #define X_USES(cls_) representationRegistry.add(the##cls_);
  #define X_REQUIRES_PROVIDES(cls_) representationRegistry.addModifiable(const_cast<cls_&>(the##cls_));
  #define X_PROVIDES(cls_) representationRegistry.addModifiable(the##cls_);
#elif defined(TEAM_REPRESENTATIONS_PROVIDED)
  #define X_REQUIRES(cls_)
  #define X_USES(cls_)
  #define X_REQUIRES_PROVIDES(cls_)
  #define X_PROVIDES(cls_)                                                                                               \
    cls_ the##cls_;                                                                                                      \
    void update(cls_ &out##cls_) override                                                                                \
    {                                                                                                                    \
      out##cls_ = the##cls_;                                                                                             \
    }
#endif

#if defined(TEAM_REPRESENTATIONS_MODULE) || defined(TEAM_REPRESENTATIONS_FOR_BEHAVIOUR) || defined(TEAM_REPRESENTATIONS_PROVIDED)

  #ifdef TEAM_REPRESENTATIONS_MODULE
  MODULE(TeamBehaviourModule2025,
  {,
  #endif // TEAM_REPRESENTATIONS_MODULE

    // ----------------------- begin edit representations list ----------------------------------

    X_REQUIRES(FieldDimensions)
    X_REQUIRES(RobotDimensions)
    X_REQUIRES(Formations)
    X_REQUIRES(Tactics)

    X_REQUIRES(FrameInfo)

    X_REQUIRES(FallDownState)
    X_REQUIRES(GroundContactState)

    X_REQUIRES(MotionInfo)
    X_USES(MotionRequest)
    X_REQUIRES(WalkingEngineOutput)

    X_REQUIRES(GameInfo)
    X_REQUIRES(TeamData)
    X_REQUIRES(SentTeamMessage2025)

    X_REQUIRES(ExtendedGameInfo)

    X_REQUIRES(RobotPose)
    X_REQUIRES(TeammatesLocationModel)

    X_USES(BehaviorStatus) // TODO: RV check if needed
    X_REQUIRES(FieldBall)
    X_REQUIRES(LibCheck)
    
    X_REQUIRES_PROVIDES(TeamActivationGraph)
    X_PROVIDES(TeamBehaviorStatus)
    X_PROVIDES(ActiveTactic)

    // ----------------------- end edit representations list ----------------------------------

  #ifdef TEAM_REPRESENTATIONS_MODULE

    LOADS_PARAMETERS(
    {,
      /// this names the top level (coro) behaviour to run - it must have been 
      /// registered already in the registerBehaviours function
      (std::string) topLevelBehaviour, 
    }),
  });
  
  #endif // TEAM_REPRESENTATIONS_MODULE

  #undef X_REQUIRES
  #undef X_USES
  #undef X_REQUIRES_PROVIDES
  #undef X_PROVIDES

#endif // TEAM_REPRESENTATIONS_MODULE || TEAM_REPRESENTATIONS_FOR_BEHAVIOUR || TEAM_REPRESENTATIONS_PROVIDED

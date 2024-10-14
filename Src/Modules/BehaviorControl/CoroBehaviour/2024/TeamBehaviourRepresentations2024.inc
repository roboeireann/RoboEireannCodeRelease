/**
 * @file CoroBehaviourRepresentations.inc
 *
 * Parts of this file are included in different places, hence .inc instead of .h
 * 
 * The idea is that every time we add/remove a representation we just edit
 * this file instead of editing multiple files. For each representation:
 * - add it to the list of includes
 * - add it to the team behaviour module REQUIRES/PROVIDES
 * - add it to the representations that behaviours can access ADD_READABLE/ADD_MODIFIABLE
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
  #include "Representations/Communication/TeamMessage2024.h"

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

#ifdef TEAM_REPRESENTATIONS_MODULE

  MODULE(TeamBehaviourModule2024,
  {,
    REQUIRES(FieldDimensions),
    REQUIRES(RobotDimensions),
    REQUIRES(Formations),
    REQUIRES(Tactics),

    REQUIRES(FrameInfo),

    REQUIRES(FallDownState),
    REQUIRES(GroundContactState),

    REQUIRES(MotionInfo),
    USES(MotionRequest),
    REQUIRES(WalkingEngineOutput),

    REQUIRES(GameInfo),
    REQUIRES(TeamData),
    REQUIRES(SentTeamMessage2024),

    REQUIRES(ExtendedGameInfo),

    REQUIRES(RobotPose),
    REQUIRES(TeammatesLocationModel),

    // USES(BehaviorStatus),
    REQUIRES(FieldBall),
    REQUIRES(LibCheck),
    
    PROVIDES(TeamActivationGraph),
    REQUIRES(TeamActivationGraph),

    // NOTE: ensure there is an entry in TEAM_REPRESENTATIONS_PROVIDED for each of these
    PROVIDES(TeamBehaviorStatus),
    PROVIDES(ActiveTactic),

    LOADS_PARAMETERS(
    {,
      /// this names the top level (coro) behaviour to run - it must have been 
      /// registered already in the registerBehaviours function
      (std::string) topLevelBehaviour, 
    }),
  });

#endif // TEAM_REPRESENTATIONS_MODULE

//====================================================================================================================

#ifdef TEAM_REPRESENTATIONS_PROVIDED

#define BEHAVIOUR_PROVIDES(typ_)                                                                                     \
    typ_ the##typ_;                                                                                                    \
    void update(typ_ &out##typ_) override                                                                              \
    {                                                                                                                  \
      out##typ_ = the##typ_;                                                                                           \
    }

  BEHAVIOUR_PROVIDES(TeamBehaviorStatus);
  BEHAVIOUR_PROVIDES(ActiveTactic);

#undef BEHAVIOUR_PROVIDES

#endif // TEAM_REPRESENTATIONS_PROVIDED

//====================================================================================================================

#ifdef TEAM_REPRESENTATIONS_FOR_BEHAVIOUR

  #define ADD_READABLE(cls) representationRegistry.add(the##cls)
  #define ADD_MODIFIABLE(cls) representationRegistry.addModifiable(the##cls)

  // register representations that will be used by team behaviours.
  // Use ADD_READABLE for any REQUIRES/USES line in the module above
  // Use ADD_MODIFIABLE for any PROVIDES line in the module above

  ADD_READABLE(FieldDimensions);
  ADD_READABLE(RobotDimensions);
  ADD_READABLE(Formations);
  ADD_READABLE(Tactics);

  ADD_READABLE(FrameInfo);

  ADD_READABLE(MotionInfo);
  ADD_READABLE(MotionRequest);
  ADD_READABLE(WalkingEngineOutput);

  ADD_READABLE(FallDownState);
  ADD_READABLE(GroundContactState);

  ADD_READABLE(GameInfo);
  ADD_READABLE(TeamData);
  ADD_READABLE(SentTeamMessage2024);

  ADD_READABLE(ExtendedGameInfo);

  ADD_READABLE(RobotPose);
  ADD_READABLE(TeammatesLocationModel);

  // ADD_READABLE(BehaviorStatus);
  ADD_READABLE(LibCheck);
  ADD_READABLE(FieldBall);

  representationRegistry.addModifiable(const_cast<TeamActivationGraph &>(theTeamActivationGraph));
  ADD_MODIFIABLE(TeamBehaviorStatus);
  ADD_MODIFIABLE(ActiveTactic);

  #undef ADD_READABLE
  #undef ADD_MODIFIABLE

#endif // TEAM_REPRESENTATIONS_FOR_BEHAVIOUR
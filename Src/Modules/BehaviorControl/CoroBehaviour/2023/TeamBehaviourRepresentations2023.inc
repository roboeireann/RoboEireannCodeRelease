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

  #include "Representations/Infrastructure/FrameInfo.h"

  #include "Representations/MotionControl/MotionInfo.h"
  #include "Representations/MotionControl/WalkingEngineOutput.h"

  #include "Representations/Communication/GameInfo.h"
  #include "Representations/Communication/RobotInfo.h"
  #include "Representations/Communication/TeamInfo.h"
  #include "Representations/Communication/TeamData.h"

  #include "Representations/BehaviorControl/BehaviorStatus.h"
  #include "Representations/BehaviorControl/FieldBall.h"
  #include "Representations/BehaviorControl/Libraries/LibCheck.h"

  // PROVIDED/MODIFIED - behaviour outputs

  #include "Representations/BehaviorControl/ActivationGraph.h"
  #include "Representations/BehaviorControl/TeamBehaviorStatus.h"

#endif // TEAM_REPRESENTATION_INCLUDES


#ifdef TEAM_REPRESENTATIONS_MODULE

  MODULE(TeamBehaviourModule2023,
  {,
    REQUIRES(FrameInfo),

    REQUIRES(MotionInfo),
    REQUIRES(WalkingEngineOutput),

    REQUIRES(GameInfo),
    REQUIRES(RobotInfo),
    REQUIRES(OwnTeamInfo),
    REQUIRES(TeamData),

    REQUIRES(RobotPose),

    USES(BehaviorStatus),
    REQUIRES(FieldBall),
    REQUIRES(LibCheck),
    
    PROVIDES(TeamActivationGraph),
    REQUIRES(TeamActivationGraph),

    PROVIDES(TeamBehaviorStatus),

    LOADS_PARAMETERS(
    {,
      /// this names the top level (coro) behaviour to run - it must have been 
      /// registered already in the registerBehaviours function
      (std::string) topLevelBehaviour, 
    }),
  });

#endif // TEAM_REPRESENTATIONS_MODULE

#ifdef TEAM_REPRESENTATIONS_FOR_BEHAVIOUR

  #define ADD_READABLE(cls) representationRegistry.add(the##cls)
  #define ADD_MODIFIABLE(cls) representationRegistry.addModifiable(the##cls)

  // register representations that will be used by team behaviours.
  // Use ADD_READABLE for any REQUIRES/USES line in the module above
  // Use ADD_MODIFIABLE for any PROVIDES line in the module above

  ADD_READABLE(FrameInfo);

  ADD_READABLE(MotionInfo);
  ADD_READABLE(WalkingEngineOutput);

  ADD_READABLE(GameInfo);
  ADD_READABLE(RobotInfo);
  ADD_READABLE(OwnTeamInfo);
  ADD_READABLE(TeamData);
  
  ADD_READABLE(RobotPose);

  ADD_READABLE(BehaviorStatus);
  ADD_READABLE(LibCheck);
  ADD_READABLE(FieldBall);

  representationRegistry.addModifiable(const_cast<TeamActivationGraph &>(theTeamActivationGraph));
  ADD_MODIFIABLE(TeamBehaviorStatus);

  #undef ADD_READABLE
  #undef ADD_MODIFIABLE

#endif // TEAM_REPRESENTATIONS_FOR_BEHAVIOUR
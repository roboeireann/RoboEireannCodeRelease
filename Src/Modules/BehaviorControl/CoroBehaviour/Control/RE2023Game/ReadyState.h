/**
 * @file ReadyStateTask.h
 *
 * This task implements the ready state behaviour (proceeding to kickoff positions).
 * It has been adapted to the 2021 rules regarding default start positions
 * for the robots.
 *
 * @author Rudi Villing
 */


#pragma once

#include "Modules/BehaviorControl/CoroBehaviour/2023/CoroBehaviour2023.h"

#include "PenaltyShotGoalie.h"
#include "PenaltyShotStriker.h"
#include "PenaltyShotOther.h"

#include "Modules/BehaviorControl/CoroBehaviour/Skills/CommonSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/HeadSkills.h"

#include "Representations/BehaviorControl/Skills.h"

#include "Tools/Math/Pose2f.h"


namespace CoroBehaviour
{
namespace RE2023
{
  CRBEHAVIOUR(ReadyStateKickoffTask)
  {
    CRBEHAVIOUR_INIT(ReadyStateKickoffTask) {}

    void operator()(void)
    {
      CRBEHAVIOUR_BEGIN();

      initReadyPose();

      CR_CHECKPOINT(lookAroundBeforeWalkToKickoff);
      // contine looking around while time is not up, while we don;t have great localisation, and while we're in our own half
      while (((getCoroDuration() < getLookAroundMs()) && (theRobotPose.quality != RobotPose::superb)) &&
             (theRobotPose.translation.x() < 0))
      {
        // commonSkills.activityStatus(BehaviorStatus::rePositionForKickOff);
        headSkills.lookActive(/* withBall: */ false, /* ignoreBall: */ true);
        commonSkills.stand();
        CR_YIELD();
      }

      CR_CHECKPOINT(walkToKickoff);
      while (true)
      {
        // commonSkills.activityStatus(BehaviorStatus::rePositionForKickOff);

        CALL_SKILL(WalkToKickoffPose)(readyPoseOnField);
        CR_YIELD();
      }
    }

  private:
    READS(RobotPose);
    READS(RobotInfo);
    READS(BehaviourFormations);
    READS(FieldDimensions);
    READS(GameInfo);
    
    CommonSkills commonSkills {env};
    HeadSkills headSkills {env};

    unsigned lookAroundMs = 3000;
    unsigned lookAroundLongerMs = lookAroundMs + 3000;

    Pose2f readyPoseOnField;

    unsigned getLookAroundMs()
    {
      switch (theRobotInfo.number)
      {
        case 1:
        case 5:
        case 4:
          return lookAroundMs;

        default:
          return lookAroundLongerMs;
      }
    }

    // for now just a simple implementation based on robot number
    // (we try to set positions that will scale to different field sizes)
    void initReadyPose()
    {
      ASSERT(theBehaviourFormations.readyStateDefaultKickoffOurs.has_value() &&
             theBehaviourFormations.readyStateDefaultKickoffTheirs.has_value());

      const BehaviourFormations::Formation &formation = env.isOurTeamKick()
                                                            ? theBehaviourFormations.readyStateDefaultKickoffOurs.value()
                                                            : theBehaviourFormations.readyStateDefaultKickoffTheirs.value();

      const std::vector<Pose2f>& poses = formation.poses;

      int idx = theRobotInfo.playerIndex();
      readyPoseOnField = poses[idx];
    }
  };




  // this is the main entry point for the ready state and it delegates to the appropriate
  // specialized behaviour to do the real work

  CRBEHAVIOUR(ReadyStateTask)
  {
    CRBEHAVIOUR_INIT(ReadyStateTask) {}

    void operator()(void)
    {
      CRBEHAVIOUR_LOOP()
      {
        if (theGameInfo.setPlay == SET_PLAY_PENALTY_KICK)
        {
          if (!commonSkills.isOurTeamKick() && theTeamBehaviorStatus.role.isGoalkeeper())
            penaltyShotGoalieReadyTask();
          else if (commonSkills.isOurTeamKick() && theTeamBehaviorStatus.role.playsTheBall())
            penaltyShotStrikerReadyTask();
          else
            penaltyShotOtherReadyTask();
        }
        else
          readyStateKickoffTask();

        CR_YIELD();
      }
    }

  private:
    READS(TeamBehaviorStatus);
    READS(GameInfo);

    CommonSkills commonSkills {env};

    PenaltyShotGoalieReadyTask penaltyShotGoalieReadyTask {env};
    PenaltyShotStrikerReadyTask penaltyShotStrikerReadyTask {env};
    PenaltyShotOtherReadyTask penaltyShotOtherReadyTask {env};
    ReadyStateKickoffTask readyStateKickoffTask {env};
  };


} // RE2023
} // CoroBehaviour2023

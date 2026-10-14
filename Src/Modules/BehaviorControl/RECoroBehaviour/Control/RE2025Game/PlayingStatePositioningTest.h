/**
 * @file PlayingState.h
 *
 * This task implements the playing state behaviour for all robots.
 * It has been adapted from the RoboEireann 2019 behaviour 
 * to fit the BH2021 code release framework and coro behaviour 
 * engine.
 *
 * @author Rudi Villing
 */


#pragma once

#include "Modules/BehaviorControl/RECoroBehaviour/2025/CoroBehaviour2025.h"

#include "Striker.h"
#include "Goalie.h"
#include "Supporter.h"
#include "PenaltyShotStriker.h"
#include "PenaltyShotGoalie.h"
#include "PenaltyShotOther.h"
#include "SetPlay.h"


#include "Modules/BehaviorControl/RECoroBehaviour/Skills/CommonSkills.h"
#include "Modules/BehaviorControl/RECoroBehaviour/Skills/HeadSkills.h"

#include "Math/Pose2f.h"


namespace RE
{
namespace v2025
{
  CRBEHAVIOUR(GotoTacticPoseTestTask)
  {
    CRBEHAVIOUR_INIT(GotoTacticPoseTestTask) 
    { 
      DECLARE_CRBEHAVIOUR_DRAWING("behaviour:GotoTacticPoseTest:tacticPose", "drawingOnField");
    }

    void operator()(void)
    {
      updateTacticPose();

      CRBEHAVIOUR_LOOP()
      {
        CR_CHECKPOINT(walk_to_tactic_pose);
        while (!walkToPoseAutoAvoidanceTask.isSuccess())
        {
          headSkills.lookActive(/* withBall: */ false);
          walkToPoseAutoAvoidanceTask(tacticPose, Pose2f(params.walkSpeed, params.walkSpeed, params.walkSpeed),
                                      /* keepTargetRotation */ false, params.distanceThreshold, params.angleThreshold);
          CR_YIELD();
        }

        CR_CHECKPOINT(stand_at_tactic_pose);
        do
        {
          headSkills.lookActive(/* withBall: */ false);
          commonSkills.stand();
          CR_YIELD();
        }
        while (!outOfPosition());
      }
    }

  private:
    DEFINES_PARAMS(GotoTacticPoseTestTask,
    {,
      (float)(1.f) walkSpeed,
      (float)(50.f) distanceThreshold, // mm
      (float)(5_deg) angleThreshold,
      (float)(100.f) outOfPositionDistance, // mm
      (float)(10_deg) outOfPositionAngle,
      (float)(30_deg) maxBallWatchingAngle, // max angle to ball when robot trying to watch ball and also face somewhat upfield
    });

    READS(RobotPose);
    // READS(REGameInfo);
    // READS(TeamBehaviorStatus);
    READS(FieldBall);
    READS(FieldDimensions);
    READS(REActiveTactic);

    CommonSkills commonSkills {env};
    HeadSkills headSkills {env};
    ObstacleSkills obstacleSkills {env};
    WalkToPoseAutoAvoidanceTask walkToPoseAutoAvoidanceTask {env};

    Pose2f tacticPose;

    bool outOfPosition()
    {
      return !commonSkills.isPoseClose(tacticPose, params.outOfPositionDistance, params.outOfPositionAngle);
    }

    void updateTacticPose()
    {
      // FIXME: naive approach - playerIndex maps directly to pose in formation
      tacticPose = theRobotPose.toRobotCoordinates(theREActiveTactic.formationPose);

      // drawings
      // CIRCLE("behaviour:GotoTacticPoseTest:tacticPose", tacticPosOnField.x(), tacticPosOnField.y(), 150, 30,
      //        Drawings::solidPen, ColorRGBA::violet, Drawings::noBrush, ColorRGBA::black);
    }
  };


  /**
   * This is the entry point task for the playing state. It decides which
   * specialized behaviour to run
   */
  CRBEHAVIOUR(PlayingStatePositioningTestTask)
  {
    CRBEHAVIOUR_INIT(PlayingStatePositioningTestTask) {}

    void operator()(void)
    {
      CRBEHAVIOUR_LOOP()
      {
        gotoTacticPoseTestTask();
        CR_YIELD();
      }
    }

  private:    
    READS(TeamBehaviorStatus);
    READS(GameState);

    GotoTacticPoseTestTask gotoTacticPoseTestTask {env};
  };

} // namespace v2025
} // namespace RE
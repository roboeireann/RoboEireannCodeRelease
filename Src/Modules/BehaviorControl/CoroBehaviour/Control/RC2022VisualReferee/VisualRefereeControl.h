/**
 * @file VisualRefereeControl.h
 *
 * This task implements the top level behaviour for the RoboCup 2022
 * Visual Referee technical challenge.
 *
 * @author Rudi Villing
 */


#pragma once

#include "Modules/BehaviorControl/CoroBehaviour/2022/CoroBehaviour2022.h"

#include "Modules/BehaviorControl/CoroBehaviour/Skills/CommonSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/SoundSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/HeadSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/ArmSkills.h"
#include "Modules/MotionControl/ArmKeyFrameEngine/ArmKeyFrameEngine.h"

namespace CoroBehaviour
{
namespace RC2022
{

  CRBEHAVIOUR(VisualRefereeControlTask)
  {
    CRBEHAVIOUR_INIT(VisualRefereeControlTask) {}

    void operator()(void)
    {
      // do this regardless of place within coro body
      commonSkills.activityStatus(BehaviorStatus::unknown);

      CRBEHAVIOUR_LOOP()
      {
        CR_CHECKPOINT(wait_for_whistle);
        while (!checkForWhistle())
        {
          headSkills.lookAtAngles(0_deg, params.headTilt); // tilt the head up to see referee gestures at 3m distance
          commonSkills.stand(/* high */ true);
          armSkills.keyFrameBothArms(ArmKeyFrameRequest::ArmKeyFrameId::lowerArmRelaxed);
          CR_YIELD();
        }

        CR_CHECKPOINT(wait_for_stable_gesture);
        while (getCheckpointDuration() < params.waitForGestureStableMs)
        {
          headSkills.lookAtAngles(0_deg, params.headTilt);
          commonSkills.stand(/* high */ true);
          armSkills.keyFrameBothArms(ArmKeyFrameRequest::ArmKeyFrameId::lowerArmRelaxed);
          CR_YIELD();
        }

        CR_CHECKPOINT(wait_for_button);
        speechRepeatTime = theFrameInfo.time;
        gestureRepeatTime = theFrameInfo.time;
        latchedGesture = theRefereeGesture.gesture;
        latchedTeam = theRefereeGesture.team;

        while (!(theEnhancedKeyStates.pressedDuration[KeyStates::headFront] >= params.buttonPressThreshold))
        {
          headSkills.lookAtAngles(0_deg, params.headTilt);
          commonSkills.stand(/* high */ true);
          
          performMirroredRefereeGesture(latchedGesture, latchedTeam);

          if (theFrameInfo.getTimeSince(speechRepeatTime) < params.speechRepeatMs)
            sayRefereeGesture(latchedGesture, latchedTeam);
          else
          {
            speechRepeatTime = theFrameInfo.time;
            soundSkills.reset(); // reset the speech so it will say the gesture name again next time
          }
          
          CR_YIELD();
        }

        CR_CHECKPOINT(lower_hands_safely);
        while (getCheckpointDuration() < params.lowerArmMs)
        {
          headSkills.lookAtAngles(0_deg, params.headTilt);
          commonSkills.stand(/* high */ true);
          armSkills.keyFrameBothArms(ArmKeyFrameRequest::ArmKeyFrameId::lowerArm);
          // CALL_SKILL(KeyFrameArms)(ArmKeyFrameRequest::ArmKeyFrameId::lowerArmSafely);
          CR_YIELD();
        }
      }
    }

  private:
    DEFINES_PARAMS(VisualRefereeControlTask,
    {,
      (int)(4000) repeatThreshold,      ///< Used to determine the time interval for repeating the state 
      (unsigned)(1000) buttonPressThreshold, ///< Used as a comparator to how much the front head button was pressed for
      (int)(1500) defaultGestureTimeout,///< Insert example
      (Angle)(-4.0_deg) headTilt, ///< tilt the head up so that robot can see the full ref 3m away
      (unsigned)(2500) waitForGestureStableMs, ///< we allow this time between the whistle blow and checking for a stable pose
      (unsigned)(4000) lowerArmMs, ///< we allow this time to lower the arm safely
      (unsigned)(4000) useDefaultArmsMs, ///< we allow this time to lower the arm safely
      (int)(5000) speechRepeatMs, ///< repeat the speech every time this period elapses until button press
      (int)(1600) fullTimeWideMs, ///< approx duration of fulltime wide gesture
      (int)(3200) fullTimeTogetherMs, ///< approx duration of fulltime wide + together gesture
    });
    
    READS(FrameInfo);
    READS(ExtendedGameInfo);
    READS(RefereeGesture);
    READS(EnhancedKeyStates);
    READS(WhistleProcessed);

    CommonSkills commonSkills {env};
    SoundSkills soundSkills; // unlike the others, soundSkills doesn't need the env
    HeadSkills headSkills {env};
    ArmSkills armSkills {env};

    unsigned speechRepeatTime;
    unsigned gestureRepeatTime;
    RefereeGesture::Gesture latchedGesture = RefereeGesture::NONE;
    RefereeGesture::Team latchedTeam = RefereeGesture::NO_TEAM;

    /**
     * check if the whistle has been blown
     */
    bool checkForWhistle()
    {
      return theWhistleProcessed.detected;
    }
    
    /**
     * perform a mirrored version of the referee gesture
     */
    void performMirroredRefereeGesture(RefereeGesture::Gesture gesture, RefereeGesture::Team team) 
    {      
      if (gesture == RefereeGesture::NONE)
        mapGestureToArms(team, ArmKeyFrameRequest::ArmKeyFrameId::lowerArm, ArmKeyFrameRequest::ArmKeyFrameId::lowerArm);

      else if (gesture == RefereeGesture::KICK_IN)
        mapGestureToArms(team, ArmKeyFrameRequest::ArmKeyFrameId::pointSideStraight, ArmKeyFrameRequest::ArmKeyFrameId::lowerArm);

      else if (gesture == RefereeGesture::GOAL_KICK)
        mapGestureToArms(team, ArmKeyFrameRequest::ArmKeyFrameId::pointSideUp, ArmKeyFrameRequest::ArmKeyFrameId::lowerArm);

      else if (gesture == RefereeGesture::CORNER_KICK)
        mapGestureToArms(team, ArmKeyFrameRequest::ArmKeyFrameId::pointSideDown, ArmKeyFrameRequest::ArmKeyFrameId::lowerArm);

      else if (gesture == RefereeGesture::GOAL)
        mapGestureToArms(team, ArmKeyFrameRequest::ArmKeyFrameId::pointSideStraight, ArmKeyFrameRequest::ArmKeyFrameId::pointToGroundAhead);

      else if (gesture == RefereeGesture::PUSHING_FREE_KICK)
        mapGestureToArms(team, ArmKeyFrameRequest::ArmKeyFrameId::pointSideStraight, ArmKeyFrameRequest::ArmKeyFrameId::pointAcrossChest);

      else if (gesture == RefereeGesture::FULL_TIME)
      {
        // if (getCheckpointDuration() < params.fullTimeGestureMs)
        //   armSkills.keyFrameBothArms(ArmKeyFrameRequest::ArmKeyFrameId::fullTime);
        // else
        //   armSkills.keyFrameBothArms(ArmKeyFrameRequest::ArmKeyFrameId::repeat);


        if (theFrameInfo.getTimeSince(gestureRepeatTime) < params.fullTimeWideMs)
          armSkills.keyFrameBothArms(ArmKeyFrameRequest::ArmKeyFrameId::fullTimeWide);
          // mapGestureToArms(team, ArmKeyFrameRequest::ArmKeyFrameId::lowerArm, ArmKeyFrameRequest::ArmKeyFrameId::fullTimeWide);
        else
        {
          armSkills.keyFrameBothArms(ArmKeyFrameRequest::ArmKeyFrameId::fullTimeTogether);
          // mapGestureToArms(team, ArmKeyFrameRequest::ArmKeyFrameId::lowerArm, ArmKeyFrameRequest::ArmKeyFrameId::fullTimeTogether);
        if (theFrameInfo.getTimeSince(gestureRepeatTime) >= params.fullTimeTogetherMs)
            gestureRepeatTime = theFrameInfo.time;
        }
      }
    }

    /**
     * map arm1 and arm2 requests to left and right arm as appropriate for
     * blue team or red team.
     * 
     * arm1 is left and arm2 is right for the BLUE_TEAM case.
     */
    void mapGestureToArms(RefereeGesture::Team team, ArmKeyFrameRequest::ArmKeyFrameId arm1, ArmKeyFrameRequest::ArmKeyFrameId arm2)
    {
      if (team == RefereeGesture::BLUE_TEAM)
      {
        armSkills.keyFrameArm(arm1, Arms::left);
        armSkills.keyFrameArm(arm2, Arms::right);

        // CALL_SKILL(KeyFrameSingleArm)(arm1, Arms::left);
        // CALL_SKILL(KeyFrameSingleArm)(arm2, Arms::right);
      }
      else
      {
        armSkills.keyFrameArm(arm1, Arms::right);
        armSkills.keyFrameArm(arm2, Arms::left);

        // CALL_SKILL(KeyFrameSingleArm)(arm1, Arms::right);
        // CALL_SKILL(KeyFrameSingleArm)(arm2, Arms::left);
      }
    }

    /// Simply says the gesture and the team that the referee is indicating
    void sayRefereeGesture(RefereeGesture::Gesture gesture, RefereeGesture::Team team)
    {
      if (gesture != RefereeGesture::NONE && gesture != RefereeGesture::FULL_TIME)
        soundSkills.say(getSpeakableGestureName(gesture) + getSpeakableTeam(team));
      else
        soundSkills.say(getSpeakableGestureName(gesture));
    }


    std::string getSpeakableGestureName(RefereeGesture::Gesture gesture)
    {
      switch (gesture)
      {
      case RefereeGesture::KICK_IN:
        return "Kick In ";
      case RefereeGesture::GOAL_KICK:
        return "Goal Kick ";
      case RefereeGesture::CORNER_KICK:
        return "Corner Kick ";
      case RefereeGesture::GOAL:
        return "Goal ";
      case RefereeGesture::PUSHING_FREE_KICK:
        return "Pushing Free Kick ";
      case RefereeGesture::FULL_TIME:
        return "Full time";
      default:
        return "No gesture";
      }
    }

    std ::string getSpeakableTeam(RefereeGesture::Team team)
    {
      switch (team)
      {
      case RefereeGesture ::BLUE_TEAM:
        return "Blue team";
      case RefereeGesture ::RED_TEAM:
        return "Ref team";
      default:
        return "No team";
      }
    }
  };

} // RE2022
} // CoroBehaviour2022
/**
 * @file predictMovingBall.h
 *
 * A simple behaviour that will prdict when the ball will be in kicking distance
 *
 * @author Nathan Ewnetu
 */

#pragma once

#include "Modules/BehaviorControl/CoroBehaviour/CoroBehaviourCommon.h"
#include "Modules/BehaviorControl/CoroBehaviour/Coroutine.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/CommonSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/HeadSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/BallSkills.h"

#include "Representations/Sensing/FootSupport.h" 


#include "Tools/Modeling/BallPhysics.h"
#include "Tools/TextLogging.h"
#include "Tools/Math/Random.h"

namespace CoroBehaviour
{
namespace RE2025
{
  CRBEHAVIOUR(RampLocationDeclarer)
  {
    CRBEHAVIOUR_INIT(RampLocationDeclarer) {}

    void operator()(void)
    {
      CRBEHAVIOUR_BEGIN();
      CR_CHECKPOINT(wait_for_tap);

      commonSkills.say("When ready, tap robot hand");
      debugRampLocation();

      //debugRampLocation();

      while(/*(debugRampLocationDeclarer == false)|| */  !(params.leftFOHPressed || params.rightBOHPressed))
      {
        commonSkills.standLookForward();
        params.leftFOHPressed = theEnhancedKeyStates.pressed[KeyStates::headFront] && 
          theEnhancedKeyStates.pressedDuration[KeyStates::headFront] >= params.buttonPressThreshold;
          
        params.rightBOHPressed = theEnhancedKeyStates.pressed[KeyStates::headRear] && 
          theEnhancedKeyStates.pressedDuration[KeyStates::headRear] >= params.buttonPressThreshold;
        
        // Used to simulate a specific sensor pressed in SimRobot
        /*
        if(debugRampLocationDeclarer)
          params.rightBOHPressed = true;
        */
        CR_YIELD();
      }

      // OK button pressed, go and kick

      //FIND: will the ball be set on the ramp? ramp set up alone? -> needs to be checked
      //Set robot to look at max degree to find ball? - the do search functionality?
      CR_CHECKPOINT(TurnHead);
      loopStartTime = getCoroDuration();
      while (getCoroDuration()-loopStartTime < 2000)
      {
        OptimalPosition();
        motionSkills.walkToPose(Pose2f(targetRotation, 0,targetOffset.y()), Pose2f(0.5f, 0.0f, 1.f), obstacleAvoidance, true);
        if(params.leftFOHPressed)
        {
          CR_CHECKPOINT(seen Left);
          headSkills.lookAtAngles(60_deg, 0_deg);  // Add tilt parameter (0_deg = level)
        }
        
        else
        {
          CR_CHECKPOINT(seen Right);
          headSkills.lookAtAngles(-60_deg, 0_deg);  // Add tilt parameter (0_deg = level)

        }
          
        CR_YIELD();
      }
      loopfailTime = getCoroDuration();
      CR_CHECKPOINT(finish);
      while(getCoroDuration()-loopfailTime < 6000)
      {
        if(theFieldBall.ballWasSeen() && abs(theBallModel.estimate.velocity.y()) > 500.0f)
        {
          CR_CHECKPOINT(seen);
          params.leftFOHPressed = false;
          params.rightBOHPressed = false;
          CR_EXIT_SUCCESS();
        }
        headSkills.lookAtAngles(lookAngle(),0_deg);
        motionSkills.stand();
        CR_YIELD();   
      }
      commonSkills.standLookActive();
      params.leftFOHPressed = false;
      params.rightBOHPressed = false;
      commonSkills.say("RESET");
      CR_EXIT_FAILED(); 
              
    }

  private:
    DEFINES_PARAMS(RampLocationDeclarer,
    {,
      (unsigned)(700) buttonPressThreshold, ///< Used as a comparator to how much the head sensor was pressed for
      (bool)(false) leftFOHPressed, ///< Used to determine which button was pressed - FOH -> front of head sensor
      (bool)(false) rightBOHPressed, ///< Used to determine which button was pressed - BOH -> back of head sensor
      (bool)(false) hasLooked, ///< Used to determine has the robot turned its head
      
    });

    READS(EnhancedKeyStates);
    READS(FieldBall);
    READS(RobotPose);
    READS(LibWalk);
    READS(BallModel);

    bool debugRampLocationDeclarer = false;

    Vector2f targetOffset;
    Angle targetRotation;

    unsigned loopStartTime;
    unsigned loopfailTime;

    void debugRampLocation()
    {
      debugRampLocationDeclarer = true;
    }

    void OptimalPosition()
    {
      if(params.rightBOHPressed)
      {
        targetOffset = Vector2f(0.f, 300.f);  // Step back and slightly right
        targetRotation = -4_deg;         // Turn toward ramp (optimal angle)
      }
      else
      {
        targetOffset = Vector2f(0.f, -300.f);   // Step back and slightly left
        targetRotation = 4_deg;          // Turn toward ramp (optimal angle)
      }
    }

    Angle lookAngle()
    {
      if(params.leftFOHPressed)
        return 60_deg;
      else
        return -60_deg;
    }


    CommonSkills commonSkills {env};
    MotionSkills motionSkills {env};
    HeadSkills headSkills {env};
    LookLeftAndRightTask lookLeftAndRightTask {env};

    MotionRequest::ObstacleAvoidance obstacleAvoidance;
  };

  // =========================================================================
  CRBEHAVIOUR(SearchForBallTask)
  {
    CRBEHAVIOUR_INIT(SearchForBallTask) {}

    void operator()(void)
    {
      CRBEHAVIOUR_BEGIN();
      CR_CHECKPOINT(SearchForBall);
      while (true)
      {
        lookLeftAndRightTask(/* startLeft */ true, /* maxPan */ 90_deg);
        motionSkills.walkAtRelativeSpeed(Pose2f(0.f, 0.f, 0.f));
        if (theFieldBall.ballWasSeen())
        {
          CR_EXIT_SUCCESS();
        }
        else
        {
          CR_YIELD();
        }
      }   
      
    }

  private:
    DEFINES_PARAMS(SearchForBallTask,
    {,
      (unsigned)(5000) searchTimeoutMs, //time to search for ball
    });

    DECL_TLOGGER_FN(tlogger, "KickMovingBall", TextLogging::INFO);
    READS(FieldBall);
    READS(BallModel);
    READS(RobotPose);
    HeadSkills headSkills {env};
    MotionSkills motionSkills {env};
    LookLeftAndRightTask lookLeftAndRightTask {env};
  };

  // =========================================================================

  CRBEHAVIOUR(PassingTheBallTask)
  {
    CRBEHAVIOUR_INIT(PassingTheBallTask) {}
    void operator()(void)
    {     
      CRBEHAVIOUR_BEGIN();
      CR_CHECKPOINT(Passing_the_ball);
      while(true)
      {
        calcKickoffAngleAndType();
        gotoBallAndKickTask(/* angle */ 0, kickType, /* kickAlignPrecisely */ false, kickLength, /* preStepAllowed */ false,/* turnKickAllowed */ false);
        if(gotoBallAndKickTask.isSuccess()) //ball is kicked
        {
          CR_EXIT_SUCCESS();
        }
        else{
          CR_YIELD();
        }
      }   
        
    }
  private:
    DECL_TLOGGER_FN(tlogger, "KickMovingBall", TextLogging::INFO);
    READS(RobotPose); 
    READS(FieldBall);
    HeadSkills headSkills {env};

    GotoBallAndKickTask gotoBallAndKickTask {env};

    float kickLength;
    KickInfo::KickType kickType;
    Vector2f ballVector;

    void calcKickoffAngleAndType()
    {
      Vector2f target = Vector2f(2000.f,0.f);
      ballVector = target - theFieldBall.positionRelative;
      kickLength = target.norm();
      kickType = KickInfo::KickType::forwardFastLeft;
    }

  };

  // =========================================================================

  CRBEHAVIOUR(PredictionTask)
  {
    CRBEHAVIOUR_INIT(PredictionTask) {}
    void operator()(void)
    {
      CRBEHAVIOUR_LOOP()
      {
        CR_CHECKPOINT(standing_at_pose);
        while (true)
        {   
          headSkills.lookAtBall();
          motionSkills.stand();
          timeForBallInFront();
          //0.475F is the time it takes to execute kick -> Func call to max kick extent
          if(timeTOIntersect  < 0.475f)  
          {
            CR_EXIT_SUCCESS();
          }

          else
          {
            CR_YIELD();
          } 
            
        }
      }
      
    }
  private:
    DECL_TLOGGER_FN(tlogger, "KickMovingBall", TextLogging::INFO);
    READS(RobotPose); 
    READS(FieldBall);
    READS(BallModel);
    READS(BallSpecification);
    READS(FrameInfo);

    HeadSkills headSkills {env};
    MotionSkills motionSkills {env};

    
    //Parameters needed for wal,king when out of range
    Pose2f walkSpeed = Pose2f(1.f, 1.f, 1.f);
    Pose2f target = Pose2f(0.f, 0.f, 0.f);
    MotionRequest::ObstacleAvoidance obstacleAvoidance;

    Vector2f kt = {4500,0};

    const Vector2f& ballVel = theBallModel.estimate.velocity;
    Vector2f kickTarget;  //Pose where the robot should kick from
    float timeTOIntersect;


    void timeForBallInFront()
    {
      if(theFieldBall.positionRelative.y() > 0.f)
        kickTarget = Vector2f(theFieldBall.positionOnField.x(), 50.f);
      else
        kickTarget = Vector2f(theFieldBall.positionOnField.x() ,-50.f);
      timeTOIntersect = BallPhysics::timeForDistance(ballVel, ( abs(kickTarget.y() - theFieldBall.positionRelative.y())), theBallSpecification.friction);
      // Logs to verify metrics

      TLOGI(tlogger(), "============================");
      TLOGI(tlogger(), "  - Time: {} sec", timeTOIntersect);
      TLOGI(tlogger(), "Ball Distance to Robot in y(): {} ", theRobotPose.translation.y() - theFieldBall.positionRelative.y() );
      TLOGI(tlogger(), "Ball Distance to Robot in x(): {} ", abs(theFieldBall.positionRelative.x()));
      TLOGI(tlogger(), "Ball Difference with fieldBall: {} ", kickTarget.y() - theFieldBall.positionRelative.y());
      TLOGI(tlogger(), "Ball Pos with fieldBall: {} ", theFieldBall.positionRelative.y());
      TLOGI(tlogger(), "============================");
      
    }

    Angle getKickAngle()
    {
      // get vector from ball to target
      Vector2f ballVector = theRobotPose.toRobotCoordinates(kt) - theFieldBall.positionRelative;
      return ballVector.angle();
    }

  };

  // =========================================================================

  CRBEHAVIOUR(KickMovingBallTask)
  {
    CRBEHAVIOUR_INIT(KickMovingBallTask) {}
    void operator()(void)
    {
      CRBEHAVIOUR_LOOP()
      {
        CR_CHECKPOINT(kick_at_moving_pose);
        while (true)
        {   
          detectKickStrikePoint();
          checkPredicitionFunction();
          calcKickoffAngleAndType();    
          // Perform the kick
          startTime();
          gotoBallAndKickTask(
            /* angle */ kickAngle, 
            kickType, 
            /* kickAlignPrecisely */ false, 
            kickLength, 
            /* preStepAllowed */ true,
            /* turnKickAllowed */ false,
            /* speed */ Pose2f(1.f, 1.f, 1.f),
            /* ignoreObstacles */ true,
            /* directionPrecision */ Rangea(0_deg, 0_deg),
            /* allowKickWhileBallMoving */ true  
            );
          if(gotoBallAndKickTask.isSuccess()) //ball is kicked
          {
            CR_EXIT_SUCCESS();
          }
          else
          {
            CR_YIELD();
          }
        }
      } 
    }
  private:
    DECL_TLOGGER_FN(tlogger, "KickMovingBall", TextLogging::INFO);
    READS(RobotPose); 
    READS(FieldBall);
    READS(MotionInfo); 
    READS(JointAngles);
    READS(BallModel);
    READS(BallSpecification);
    READS(FrameInfo);
    GotoBallAndKickTask gotoBallAndKickTask {env};
    WalkToBallAndKickTask walkToBallAndKickTask {env};
    MotionSkills motionSkills {env};
    CommonSkills commonSkills {env};

    float kickLength;
    Vector2f target = Vector2f(4500.f,0.f);
    KickInfo::KickType kickType;
    Angle kickAngle;
    bool trueStart = true;
    


    Vector2f kickDirection; 
    Vector2f ballInterceptPoint;

    float rHipPitch;
    float prevRHipPitch = 0.f;

    const Vector2f& ballVel = theBallModel.estimate.velocity;
    Vector2f kickTarget;  //Pose where the robot should kick from
    float timeTOIntersect;

    void startTime()
    {  
      if(trueStart)
        TLOGI(tlogger(), "Kicking Function started");
      trueStart = false;
    }

    Angle getKickAngle()
    {
      // get vector from ball to target
      Vector2f ballVector = theRobotPose.toRobotCoordinates(target) - theFieldBall.positionRelative;
      return ballVector.angle();
    }

    // Calculate the angle and type of kick based on the ball's position
    void calcKickoffAngleAndType()
    {  
      kickLength = 4500.f;
      if(theFieldBall.positionOnField.y() > 0.f)
        kickType = KickInfo::kickMovingBallRight;
      else
        kickType = KickInfo::kickMovingBallLeft;
      
    }
    // Find what time max extent was -> Used to meausre time between function call and kick max extension
    void detectKickStrikePoint()
    { 
      
      rHipPitch = theJointAngles.angles[Joints::rHipPitch];
      // Threshold range around -0.872665 radians (about -50 degrees)
      const float targetPitch = -0.872665f;
      const float threshold = 0.01f; // About 0.5 degrees tolerance
      if(theMotionInfo.isWalkPhaseInWalkKick)
      {
        TLOGI(tlogger(), "Current Knee Angle: {} ", rHipPitch);
        // Detect the transition through the target value (more reliable)
        if ((prevRHipPitch > targetPitch - threshold && rHipPitch < targetPitch - threshold) || (std::abs(rHipPitch - targetPitch) < threshold))
          TLOGI(tlogger(), "Strike Point Detected at hip angle: {}", rHipPitch);
      }
      prevRHipPitch = rHipPitch;
      
    }

    //Checks how good the prediction function is
    void checkPredicitionFunction()
    {
      if(theFieldBall.positionOnField.y() > 0.f)
        kickTarget = Vector2f(theFieldBall.positionOnField.x(),50.f);
      else
        kickTarget = Vector2f(theFieldBall.positionOnField.x() ,-50.f);
      timeTOIntersect = BallPhysics::timeForDistance(ballVel, ( abs(kickTarget.y() - theFieldBall.positionRelative.y())), theBallSpecification.friction);
      if(timeTOIntersect <  0.02f)
      {
        TLOGI(tlogger(), "[{}] Frame time", theFrameInfo.time);
        TLOGI(tlogger(), "[{}] Ball is here", timeTOIntersect);
        TLOGI(tlogger(), "[{}] Actual Ball is here", theFieldBall.positionRelative.y());
      }
    }
     
  };

  // =========================================================================
  CRBEHAVIOUR(MovingBallPlayerPlayingTask)
  {
    CRBEHAVIOUR_INIT(MovingBallPlayerPlayingTask) {}
    void operator()(void)
    {
      CRBEHAVIOUR_LOOP()
      {
        // Kicking moving ball robot
        if(Global::getSettings().playerNumber == 3)
        {    
          CR_CHECKPOINT(RampLocator);
          do
          {
            rampLocationDeclarer();
            CR_YIELD();
          }while(!rampLocationDeclarer.isSuccess());
          // Give head time to turn
          while(getCoroDuration() < 100)
          {
            CR_YIELD();
          }
          CR_CHECKPOINT(PredictBall);
          do
          {
            predictionTask();
            CR_YIELD();
          }while(!predictionTask.isSuccess());
          

          CR_CHECKPOINT(KickTheBall);
          do
          {
            kickMovingBallTask();
            CR_YIELD();
          }while(!kickMovingBallTask.isSuccess());
          
          CR_CHECKPOINT(LookingForwad);
          do
          {
            commonSkills.standLookForward(true);
            CR_YIELD();
          }while(true);   
        }

        // Make player 2 wait before passing the ball, allow time for robot 3 to see where ball is   
        if(Global::getSettings().playerNumber == 2)
        {
          CR_CHECKPOINT();
          while(getCoroDuration() < 2000)
          {
            commonSkills.standLookForward(true);
            CR_YIELD();
          }

          CR_CHECKPOINT(PassTheBall);
          TLOGI(tlogger(), "Passing Ball Task Begins");
          do
          {    
            passingTheBallTask();
            CR_YIELD();
          }while(!passingTheBallTask.isSuccess());

          // Get passing robot to stand still after passing
          CR_CHECKPOINT(lookingAtBallPoseTask);
          TLOGI(tlogger(), "Standing Ball Task Begins");
          do
          {
            commonSkills.standLookForward(true);
            CR_YIELD();
          }while(true);
        }
      }
    
    }
  private:
    READS(RobotPose); 
    READS(FieldBall);
    READS(BallModel);
    READS(GameInfo);
    DECL_TLOGGER_FN(tlogger, "KickMovingBall", TextLogging::INFO);
    
    HeadSkills headSkills {env};
    CommonSkills commonSkills {env};
    SearchForBallTask searchForBallTask {env};
    PassingTheBallTask passingTheBallTask {env};
    PredictionTask predictionTask {env};
    KickMovingBallTask kickMovingBallTask {env};
    RampLocationDeclarer rampLocationDeclarer {env};

  };

  // =========================================================================
     
} // RE2025
} // CoroBehaviour


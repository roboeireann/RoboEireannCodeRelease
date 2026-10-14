/**
 * Top level handling of camera, battery, etc
 *
 * Converted to HybridBehaviour syntax from HandlePhysicalRobot.cpp by B-Human
 */

#include "REHybridBehaviourModule.h"


DECL_TLOGGER(tlogger, "handlePhysicalRobotTask", TextLogging::DEBUG);


HSM_DEFINE((REHybridBehaviourModule, handlePhysicalRobotTask))
{
  HSM_GLOBAL_TRANSITIONS
  {
    if(!theCameraStatus.ok)
      HSM_GOTO(cameraBroken);
#ifndef NDEBUG
    else if(theRobotHealth.batteryLevel <= 1)
      HSM_GOTO(lowBattery);
#endif
    else
      HSM_GOTO(working);
  }

  HSM_INITIAL_STATE(working)
  {
    if (THIS_TASK.stateChanged())
      TLOGD(tlogger, "working");
  }

  HSM_STATE(cameraBroken)
  {
    if (THIS_TASK.stateChanged())
      TLOGD(tlogger, "cameraBroken");

    headPlayDeadTask();
    playDeadTask();
  }

#ifndef NDEBUG
  HSM_STATE(lowBattery)
  {
    if (THIS_TASK.stateChanged())
      TLOGD(tlogger, "lowBattery");

    headPlayDeadTask();
    playDeadTask();
  }
#endif
}

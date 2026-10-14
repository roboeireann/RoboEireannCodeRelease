#include "SkillBehaviorControl.h"


DECL_OPTION_TLOGGER(HandlePhysicalRobot, TextLogging::DEBUG);


option((SkillBehaviorControl) HandlePhysicalRobot)
{
  common_transition
  {
    if(!theCameraStatus.ok)
      goto cameraBroken;
#ifndef NDEBUG
    else if(theRobotHealth.batteryLevel <= 1)
      goto lowBattery;
#endif
    else
      goto working;
  }

  initial_state(working)
  {
    TLOGD_STATE_ENTRY(working);
  }

  state(cameraBroken)
  {
    action
    {
      TLOGD_STATE_ENTRY(cameraBroken);
      LookAtAngles({.pan = JointAngles::off,
                    .tilt = JointAngles::off});
      PlayDead();
    }
  }

#ifndef NDEBUG
  state(lowBattery)
  {
    action
    {
      TLOGD_STATE_ENTRY(lowBattery);
      LookAtAngles({.pan = JointAngles::off,
                    .tilt = JointAngles::off});
      PlayDead();
    }
  }
#endif
}

/**
 * @file Threads/Motion.cpp
 *
 * This file implements the execution unit for the motion thread.
 *
 * @author Jan Fiedler
 */

#include "Modules/Infrastructure/RobotProvider/NaoProvider.h" // include must be the first, because of Visual Studio
#include "Modules/Infrastructure/RobotProvider/BoosterK1Provider.h"
#include "Motion.h"
#include "Framework/ModulePacket.h"
#include "Framework/Settings.h"
#include "Modules/Infrastructure/LogDataProvider/LogDataProvider.h"
#include "Platform/Thread.h"
#include "Platform/Time.h"
#include "Streaming/Global.h"

REGISTER_EXECUTION_UNIT(Motion)

bool Motion::beforeFrame()
{
  return LogDataProvider::isFrameDataComplete();
}

void Motion::afterModules()
{
  if (Global::getSettings().robotType == Settings::k1)
    BoosterK1Provider::finishFrame();
  else
    NaoProvider::finishFrame();

  // BoosterProvider::finishFrame();
}

bool Motion::afterFrame()
{
  if(Blackboard::getInstance().exists("JointSensorData"))
  {
    BH_TRACE_MSG("before waitForFrameData");
    
    if (Global::getSettings().robotType == Settings::k1)
      BoosterK1Provider::waitForFrameData();
    else
      NaoProvider::waitForFrameData();

    // BoosterProvider::waitForFrameData();
  }
  else
    Thread::sleep(static_cast<unsigned>(Global::getSettings().motionCycleTime * 1000.f));

  return BHExecutionUnit::afterFrame();
}

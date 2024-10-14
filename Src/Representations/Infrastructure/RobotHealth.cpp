/**
 * @file RobotHealth.cpp
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 * @author <a href="mailto:fthielke@uni-bremen.de">Felix Thielke</a>
 */

#include "RobotHealth.h"
#include "Tools/Module/Blackboard.h"

// otherwise RobotHealthMessageID::getName() throws an unused function warning
struct WarningSuppressor
{
  ENUM(RobotHealthMessageId,
  {,
    idmotionFrameRate,
    idavgMotionTime,
    idmaxMotionTime,
    idminMotionTime,
    idcognitionFrameRate,
    idbatteryLevel,
    idtotalCurrent,
    idmaxJointTemperatureStatus,
    idjointWithMaxTemperature,
    idcpuTemperature,
    idload,
    idmemoryUsage,
    idwlan,
    idrobotName,
    idconfiguration,
    idlocation,
    idscenario,
  });
};


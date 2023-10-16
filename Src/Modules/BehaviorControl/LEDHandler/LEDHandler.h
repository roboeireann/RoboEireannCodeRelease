/**
 * @file LEDHandler.h
 * 
 * This file implements a module that generates the LEDRequest from certain representations.
 * In other words, this module maps from the state of representations such GameInfo and TeamBehaviourStatus
 * to colours on the various LEDs to provide feedback to us monitoring the robots.
 * 
 * @author jeff
 */

#pragma once

#include "Representations/BehaviorControl/TeamBehaviorStatus.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Communication/TeamInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/LEDRequest.h"
#include "Representations/Infrastructure/SensorData/SystemSensorData.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Perception/FieldFeatures/FieldFeatureOverview.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/RobotHealth.h"

MODULE(LEDHandler,
{,
  REQUIRES(BallModel),
  REQUIRES(FieldFeatureOverview),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(GroundContactState),
  REQUIRES(JointSensorData),
  REQUIRES(OwnTeamInfo),
  REQUIRES(RobotInfo),
  USES(RobotHealth),
  REQUIRES(SystemSensorData),
  REQUIRES(TeamBehaviorStatus),
  REQUIRES(TeamData),
  PROVIDES(LEDRequest),
  DEFINES_PARAMETERS(
  {,
    (int)(2000) gameControllerTimeOut,
  }),
});

class LEDHandler : public LEDHandlerBase
{
public:
  ENUM(EyeColor,
  {,
    red,
    green,
    blue,
    white,
    magenta,
    yellow,
    cyan,
  });

private:
  void update(LEDRequest& ledRequest) override;

  void setRightEar(LEDRequest& ledRequest);
  void setLeftEar(LEDRequest& ledRequest);
  void setLeftEye(LEDRequest& ledRequest);
  void setRightEye(LEDRequest& ledRequest);
  void setHead(LEDRequest& ledRequest);
  void setChestButton(LEDRequest& ledRequest);
  void setLeftFoot(LEDRequest& ledRequest);
  void setRightFoot(LEDRequest& ledRequest);
};

/**
 * @file LedProviderK1.h
 * Determines the appropriate LED requests for the LedRequestK1 representation
 * based on factors such as GameState etc.
 * @author Rudi Villing
 */

#pragma once

#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Infrastructure/LedRequestK1.h"
// #include "Representations/Infrastructure/SensorData/SystemSensorData.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/Modeling/GlobalTeammatesModel.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Infrastructure/RobotSafetyState.h"
// #include "Representations/Infrastructure/RobotHealth.h"
#include "Framework/Module.h"

MODULE(LedProviderK1,
{,
  REQUIRES(FrameInfo),
  REQUIRES(GameState),
  REQUIRES(GlobalTeammatesModel),
  REQUIRES(GroundContactState),
  REQUIRES(RobotSafetyState),
  REQUIRES(EnhancedKeyStates),
  // REQUIRES(JointSensorData),
  // USES(RobotHealth),
  // REQUIRES(SystemSensorData),
  PROVIDES(LedRequestK1),
  // DEFINES_PARAMETERS(
  // {,
  //   (int)(5) chargingLightSlowness,
  //   (int)(50) tempForHalfLEDActive,
  //   (int)(65) tempForLEDBlinking,
  //   (int)(75) tempForLEDFastBlinking,
  // }),
});

class LedProviderK1 : public LedProviderK1Base
{
private:
  static constexpr unsigned LONG_PERIOD_MS = 6400;
  static constexpr unsigned GC_OFFSET_MS = 1200;
  static constexpr unsigned TEAMMATE_OFFSET_MS = 4400;
  static constexpr unsigned TEAMMATE_INTERVAL_MS = 120;
  static constexpr unsigned FLASH_MS = 30;

  LedRequestK1 lastRequest;

  void update(LedRequestK1& ledRequest) override;

  void updateFromGameState(LedRequestK1 &ledRequest);
  void updateFromGroundState(LedRequestK1 &ledRequest);
  void updateFromTeammates(LedRequestK1 &ledRequest);
  void updateFromKeyStates(LedRequestK1 &ledRequest);
  void updateFromRobotSafetyState(LedRequestK1 &ledRequest);
};

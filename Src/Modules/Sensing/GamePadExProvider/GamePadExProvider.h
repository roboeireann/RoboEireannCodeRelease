/**
 * GamePadExProvider.h
 *
 * @author Rudi Villing
 */

#pragma once

#include "Representations/Infrastructure/SensorData/GamePad.h"
#include "Framework/Module.h"

MODULE(GamePadExProvider,
{,
  REQUIRES(GamePad),
  PROVIDES(GamePadEx),
});

class GamePadExProvider : public GamePadExProviderBase
{
  void update(GamePadEx& gamePadEx) override;
};

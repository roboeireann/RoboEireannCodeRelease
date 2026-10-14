/**
 * @file FrameInfoExProvider.h
 *
 * A module that provides the FrameInfoEx representation.
 *
 * @author Rudi Villing
 */

#pragma once

#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/FrameInfoEx.h"

MODULE(FrameInfoExProvider,
{,
  REQUIRES(FrameInfo),
  PROVIDES(FrameInfoEx),
});

class FrameInfoExProvider : public FrameInfoExProviderBase
{
  /**
   * Update the FrameInfoEx representation for the current frame
   */
  void update(FrameInfoEx& theFrameInfoEx) override;

  unsigned prevFrameTime = 0;
  unsigned frameNumber = 0;
};

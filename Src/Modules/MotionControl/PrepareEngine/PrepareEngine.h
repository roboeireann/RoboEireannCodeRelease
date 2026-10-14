/**
 * @file PrepareEngine.h
 *
 * The robot transitions from playDead/unstiff to a stable pose where it is still
 * safe to handle the robot, but it will be ready to transition to full
 * RL based motion control.
 *
 * Since it mainly relies on the Booster SDK, it is largely based on the 
 * PlayDead approach
 *
 * @author Rudi Villing
 * @author Philip Reichmann (original outline)
 */

#pragma once

#include "Framework/Module.h"
#include "Framework/Settings.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/MotionControl/EnergySaving.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/PrepareGenerator.h"
#include "Representations/Sensing/IMUValueState.h"
#include "Representations/Sensing/InertialData.h"

MODULE(PrepareEngine,
{,
  REQUIRES(EnergySaving),
  REQUIRES(FrameInfo),
  REQUIRES(IMUValueState),
  REQUIRES(InertialData),
  REQUIRES(JointAngles),
  PROVIDES(PrepareGenerator),
  DEFINES_PARAMETERS(
  {,
    (Angle)(15_deg) torsoAngleForLeaving,
    (Angle)(15_deg) gyroAnglesForLeaving,
  }),
});

class PrepareEngine : public PrepareEngineBase
{
public:
  PrepareEngine();
  void update(PrepareGenerator& thePrepareGenerator) override;

  JointAngles target;
};


// ============================================================================

struct PreparePhase : MotionPhase
{
  explicit PreparePhase(PrepareEngine& engine);

private:
  void update() override;
  bool isDone(const MotionRequest& motionRequest) const override;
  void calcJoints(const MotionRequest& motionRequest, JointRequest& jointRequest, Pose2f& odometryOffset, MotionInfo& motionInfo) override;
  std::unique_ptr<MotionPhase> createNextPhase(const MotionPhase& defaultPhase) const override;

  PrepareEngine& engine;

};
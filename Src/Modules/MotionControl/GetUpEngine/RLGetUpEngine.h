/**
 * @file RLGetUpEngine.h
 *
 * @Author Philip Reichenberg
 */

#pragma once

#include "Framework/Module.h"
#include "Framework/Settings.h"
#include "Representations/Configuration/JointLimits.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/MotionControl/GetUpGenerator.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/WalkGenerator.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/InertialData.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Tools/Motion/MotionPhase.h"

#include "Platform/File.h"
#include <CompiledNN2ONNX/CompiledNN.h>

using namespace NeuralNetworkONNX;

STREAMABLE(RLGetUpKeyframeInfo,
{,
  (Rangea) torsoYRange,
  (Rangea) torsoXRange,
  (float) executionTime,
});

STREAMABLE(RecoverMotion,
{,
  (ENUM_INDEXED_ARRAY(Angle, Joints::Joint)) positions,
  (float) duration,
});

MODULE(RLGetUpEngine,
{,
  REQUIRES(FallDownState),
  REQUIRES(FrameInfo),
  REQUIRES(GameState),
  REQUIRES(InertialData),
  REQUIRES(JointAngles),
  REQUIRES(JointLimits),
  USES(JointRequest),
  REQUIRES(TorsoMatrix),
  REQUIRES(WalkGenerator),
  PROVIDES(GetUpGenerator),
  LOADS_PARAMETERS(
  {,
    (int) maxTryCounter,
    (std::string) policyName,
    (unsigned) numInput,
    (unsigned) numOutput,
    (Rangef) clipActions,
    (bool) useWaist,
    (float) recoveryTime,
    (float) speedFactor, // Get up can be between 100 and 200% speed -> value between 1 and 2. NOTE: only trained with 1.5 to 2 :)
    (Angle) breakUpJointSpeed,
    (Angle) breakUpHeadAngle,
    (Angle) maxPositionDifference,
    (int) breakUpTime,
    (int) stopSoundTime,
    (float) earliestDoneTime,
    (float) minStandHeightWhenDone,
    (int) helpMeSoundTimeWindow,
    (std::vector<RLGetUpKeyframeInfo>) frontInfo,
    (std::vector<RLGetUpKeyframeInfo>) backInfo,
    (std::vector<RecoverMotion>) recoverFront,
    (std::vector<RecoverMotion>) recoverBack,
    (std::vector<RecoverMotion>) recoverNormal,
  }),
});

class RLGetUpEngine : public RLGetUpEngineBase
{
public:
  /** Constructor */
  RLGetUpEngine();
  const float motionCycleTime = Global::getSettings().motionCycleTime;
  CompiledNN policy; /**< The compiled neural network. */
  JointAngles offset;
  unsigned int lastFrameInfo = 0;
  JointAngles lastMeasurement;
  JointAngles fallAngles;

  std::vector<Joints::Joint> boosterJoints;
  std::vector<Joints::Joint> boosterWaistJoints;

  JointRequest recoveryPose;
private:

  void update(GetUpGenerator& getUpGenerator) override;

  /**
   * Compile the model.
   * @param output Whether to output information.
   */
  void compile(bool output);

  const std::string modelPath = std::string(File::getBHDir()) + "/Config/NeuralNets/RLGetUpEngine/";
};

struct RLGetUpPhase : MotionPhase
{
  RLGetUpPhase(RLGetUpEngine& engine);

private:
  bool isDone(const MotionRequest& motionRequest) const override;
  void calcJoints(const MotionRequest& motionRequest, JointRequest& jointRequest, Pose2f& odometryOffset, MotionInfo& motionInfo) override;
  std::unique_ptr<MotionPhase> createNextPhase(const MotionPhase& defaultNextPhase) const override;
  void update() override;

  void executePolicy(JointAngles& target);

  void doRecovery(JointRequest& request);

  void doBreakUp(JointRequest& request);

  void setUpRecovery();

  void getRefTorso(Rangea& torsoXRange, Rangea& torsoYRange);

  bool shouldBreakUp();

  std::vector<Joints::Joint> getBoosterLegJointSequence();

  JointAngles startAngles;
  unsigned int startTime = 0;
  float maxStandUpTime = 0.f;
  float executedTime = 0.f;
  unsigned int lastInference = 0;
  JointRequest nextRequest;
  int tryCounter = 0;
  unsigned lastStopSoundTimestamp = 0;

  unsigned lastHelpMeSound = 0;

  std::vector<RecoverMotion>* recoverMotion;
  std::size_t recoverMotionIndex = 0;

  enum State
  {
    recovery,
    breakUp,
    standUp,
    helpMe,
    done,
  };

  State state = State::recovery;

  bool isFront = true;

  RLGetUpEngine& engine; /**< A reference to the running motion engine. */

  friend class WalkingEngine;
};

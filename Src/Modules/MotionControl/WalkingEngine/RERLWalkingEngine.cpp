/**
 * @file RERLWalkingEngine.h
 *
 * Walking engine implementation for an RL model trained in IsaacLab.
 * This file utilizes the custom inputs and outputs from the our locomotion
 * policy.
 * 
 * This file is based on RLWalkingEngine.
 * 
 * @Author James Florin Petri (adding safety features, configured for our walking policy)
 * @Author Philip Reichenberg (initial work on RLWalkingEngine)
 */

#include "RERLWalkingEngine.h"
#include "Debugging/Annotation.h"
#include "Platform/SystemCall.h"
#include "TextLogging/TextLogging.h"


#include <filesystem>


MAKE_MODULE(RERLWalkingEngine);

DECL_TLOGGER(tlogger, "RERLWalkingEngine", TextLogging::DEBUG);
DECL_TLOGGER(walkPhaseLogger, "RERLWalkPhaseDummy", TextLogging::DEBUG);

RERLWalkingEngine::RERLWalkingEngine():
  network(&Global::getAsmjitRuntime())
{
  // https://github.com/BoosterRobotics/booster_gym/blob/main/deploy/configs/T1.yaml#L19
  offset.angles[Joints::lHipPitch] = -0.2f;
  offset.angles[Joints::lKneePitch] = 0.4f;
  offset.angles[Joints::lAnklePitch] = -0.25f;
  offset.angles[Joints::rHipPitch] = -0.2f;
  offset.angles[Joints::rKneePitch] = 0.4f;
  offset.angles[Joints::rAnklePitch] = -0.25f;
  while(!lastMeasurements.full())
    lastMeasurements.push_front(JointAngles());

  const float stepDuration = configuredParameters.baseWalkPeriod / 1000.f;
  translationPolygon.emplace_back(stepDuration * Vector2f(configuredParameters.maxSpeed.translation.x() / 2.f, configuredParameters.maxSpeed.translation.y()));
  translationPolygon.emplace_back(stepDuration * Vector2f(configuredParameters.maxSpeed.translation.x(), configuredParameters.maxSpeed.translation.y() / 2.f));
  translationPolygon.emplace_back(stepDuration * Vector2f(configuredParameters.maxSpeed.translation.x(), -configuredParameters.maxSpeed.translation.y() / 2.f));
  translationPolygon.emplace_back(stepDuration * Vector2f(configuredParameters.maxSpeed.translation.x() / 2.f, -configuredParameters.maxSpeed.translation.y()));
  translationPolygon.emplace_back(stepDuration * Vector2f(-configuredParameters.maxSpeedBackwards / 2.f, -configuredParameters.maxSpeed.translation.y()));
  translationPolygon.emplace_back(stepDuration * Vector2f(-configuredParameters.maxSpeedBackwards, -configuredParameters.maxSpeed.translation.y() / 2.f));
  translationPolygon.emplace_back(stepDuration * Vector2f(-configuredParameters.maxSpeedBackwards, configuredParameters.maxSpeed.translation.y() / 2.f));
  translationPolygon.emplace_back(stepDuration * Vector2f(-configuredParameters.maxSpeedBackwards / 2.f, configuredParameters.maxSpeed.translation.y()));

  boosterJoints = { 
    Joints::lHipPitch,
    Joints::rHipPitch,
    Joints::lHipRoll,
    Joints::rHipRoll,
    Joints::lHipYaw,
    Joints::rHipYaw,
    Joints::lKneePitch,
    Joints::rKneePitch,
    Joints::lAnklePitch,
    Joints::rAnklePitch,
    Joints::lAnkleRoll,
    Joints::rAnkleRoll
  };

  boosterWaistJoints = { Joints::waistYaw };
  boosterWaistJoints.insert(boosterWaistJoints.end(), boosterJoints.begin(), boosterJoints.end());

  naoJoints = {
    Joints::lHipYawPitch,
    Joints::lHipRoll,
    Joints::lHipPitch,
    Joints::lKneePitch,
    Joints::lAnklePitch,
    Joints::lAnkleRoll,
    Joints::rHipYawPitch,
    Joints::rHipRoll,
    Joints::rHipPitch,
    Joints::rKneePitch,
    Joints::rAnklePitch,
    Joints::rAnkleRoll
  };

  compile(false);

  const DummyPhase dummy(MotionPhase::playDead);
  RERLWalkPhaseDummy phase(*this, Pose2f(), dummy);
}

void RERLWalkingEngine::compile(bool output)
{
  if(output)
  {
    if(!std::filesystem::exists(modelPath + reboostWalkNNParameters.modelName))
    {
      OUTPUT_ERROR("File " << modelPath << reboostWalkNNParameters.modelName << " does not exist");
      return;
    }
  }
  else
    ASSERT(std::filesystem::exists(modelPath + reboostWalkNNParameters.modelName));

  // set up the expected number of inputs
  constexpr unsigned numGyroInputs = 3;
  constexpr unsigned numGravityInputs = 3;
  constexpr unsigned numWalkCmdInputs = 3;
  constexpr unsigned numPhaseInputs = 2;
  constexpr unsigned numFreqInputs = 1;
  constexpr unsigned numBallInputs = 3 + 3+ 2 + 1 + 1 + 1;
  const unsigned numJoints = static_cast<unsigned>(boosterJoints.size());

  networkInputSize = numGyroInputs + numGravityInputs + numWalkCmdInputs + numJoints * 3; /* angles, velocities, lastRequests*/
  if (reboostWalkNNParameters.enablePhaseInputs)
    networkInputSize += numPhaseInputs;
  if (frequencyParameters.active)
    networkInputSize += numFreqInputs;
  if (ballNetworkParameters.active)
    networkInputSize += numBallInputs;

  // set the expected number of outputs
  constexpr unsigned numFreqOutputs = 1;

  networkOutputSize = numJoints;
  if (frequencyParameters.active)
    networkOutputSize += numFreqOutputs;

  network.compile(Model(modelPath + reboostWalkNNParameters.modelName));
  ASSERT(network.valid());

  TLOGI(tlogger, "Input model (expected): size={} ({}), rank=1 (1), tensors=1 (1)", network.input(0).dims(0),
        networkInputSize, network.input(0).rank(), network.numOfInputs());
  TLOGI(tlogger, "Output model (expected): size={} ({}), rank=1 (1), tensors=1 (1)", network.output(0).dims(0),
        networkOutputSize, network.output(0).rank(), network.numOfOutputs());

  // Input shape: (47)
  ASSERT(network.numOfInputs() == 1);
  ASSERT(network.input(0).rank() == 1); // (Batch, Time, Features)
  // ASSERT(network.input(0).dims(0) == reboostWalkNNParameters.numInput);
  ASSERT(network.input(0).dims(0) == networkInputSize);

  // Output shape: (1, 11)
  ASSERT(network.numOfOutputs() == 1);
  ASSERT(network.output(0).rank() == 1); // (Batch, Time, Features)
  // ASSERT(network.output(0).dims(0) == reboostWalkNNParameters.numOutput); // TODO
  ASSERT(network.output(0).dims(0) == networkOutputSize); // TODO
}

void RERLWalkingEngine::update(WalkStepData& walkStepData)
{
  lastMeasurements.push_front(theJointAngles);

  walkStepData.updateCounter = [&walkStepData](const bool)
  {
    walkStepData.usedPredictedSwitch = 0; // Unused
  };
  walkStepData.updateWalkValues = [this, &walkStepData](const Pose2f& stepTarget, const float, const bool isLeftPhase)
  {
    walkStepData.isLeftPhase = isLeftPhase;
    walkStepData.stepTarget = stepTarget;
    walkStepData.stepDuration = configuredParameters.baseWalkPeriod;
    walkStepData.lastUpdate = theFrameInfo.time;
  };

  walkStepData.yHipOffset = theRobotDimensions.yHipOffset;
}

void RERLWalkingEngine::update(StandGenerator& standGenerator)
{
  standGenerator.createPhase = [this](const MotionRequest&, const MotionPhase& lastPhase)
  {
    return std::make_unique<RERLWalkPhaseDummy>(*this, Pose2f(), lastPhase);
  };
}

void RERLWalkingEngine::update(WalkingEngineOutput& walkingEngineOutput)
{
  Pose2f useSpeed;
  useSpeed.rotation = configuredParameters.maxSpeed.rotation;
  useSpeed.translation = configuredParameters.maxSpeed.translation;
  const float useSpeedBackwards = configuredParameters.maxSpeedBackwards;

  walkingEngineOutput.maxSpeed = useSpeed;
  walkingEngineOutput.maxSpeedBackwards = useSpeedBackwards;
  walkingEngineOutput.walkStepDuration = configuredParameters.baseWalkPeriod / 1000.f;

  walkingEngineOutput.maxStepSize = Pose2f(useSpeed.rotation * walkingEngineOutput.walkStepDuration,
                                           useSpeed.translation.x() * walkingEngineOutput.walkStepDuration,
                                           useSpeed.translation.y() * walkingEngineOutput.walkStepDuration * 2.f);
  walkingEngineOutput.maxBackwardStepSize = useSpeedBackwards * walkingEngineOutput.walkStepDuration;

  walkingEngineOutput.maxPossibleStepSize = Pose2f(configuredParameters.maxSpeed.rotation * walkingEngineOutput.walkStepDuration,
                                                   configuredParameters.maxSpeed.translation.x() * walkingEngineOutput.walkStepDuration,
                                                   configuredParameters.maxSpeed.translation.y() * walkingEngineOutput.walkStepDuration * 2.f);
  walkingEngineOutput.maxPossibleBackwardStepSize = configuredParameters.maxSpeedBackwards * walkingEngineOutput.walkStepDuration;

  walkingEngineOutput.energyEfficientWalkSpeed = configuredParameters.maxSpeed;
  walkingEngineOutput.energyEfficientWalkSpeed.rotation *= 0.9f; // prevent possible divisions by 0 in behavior
  walkingEngineOutput.energyEfficientWalkSpeed.translation *= 0.9f;
  walkingEngineOutput.noEfficientWalkSpeed = configuredParameters.maxSpeed;

  walkingEngineOutput.brakingModel = brakingModelParameters;
}

void RERLWalkingEngine::update(WalkGenerator& walkGenerator)
{
  walkGenerator.createPhase = [this](const Pose2f& step, const MotionPhase& lastPhase, float)->std::unique_ptr<MotionPhase>
  {
    return std::make_unique<RERLWalkPhaseDummy>(*this, step, lastPhase);
  };

  walkGenerator.createPhaseWithNextPhase = [this](const WalkKickStep& walkKickStep, const MotionPhase& lastPhase, const WalkGenerator::CreateNextPhaseCallback& createNextPhaseCallback, float)->std::unique_ptr<MotionPhase>
  {
    return std::make_unique<RERLWalkPhaseDummy>(*this, walkKickStep.keyframe[walkKickStep.keyframe.size() - 1].stepTarget, lastPhase, false, createNextPhaseCallback, walkKickStep);
  };

  walkGenerator.isNextLeftPhase = [this](const MotionPhase& lastPhase, const Pose2f& stepTarget)
  {
    switch(lastPhase.type)
    {
      case MotionPhase::walk:
      {
        const auto& lastWalkPhase = static_cast<const RERLWalkPhaseDummy&>(lastPhase);
        return lastWalkPhase.tBase < 0.44f || lastWalkPhase.tBase > 0.94f;
      }
      case MotionPhase::kick:
      case MotionPhase::stand:
      {
        return stepTarget.translation.y() != 0.f // first step based on side translation
               ? stepTarget.translation.y() > 0
               : (stepTarget.rotation != 0_deg ? stepTarget.rotation > 0_deg // else first step based rotation
                  : theFootSupport.support < 0.f); // otherwise based on support foot
      }
      default:
        return theFootSupport.support < 0.f;
    }
  };

  walkGenerator.wasLastPhaseLeftPhase = [this](const MotionPhase& lastPhase)
  {
    if(lastPhase.type != MotionPhase::walk)
      return theFootSupport.support > 0.f;

    const auto& lastWalkPhase = static_cast<const RERLWalkPhaseDummy&>(lastPhase);
    return lastWalkPhase.isLeftPhase;
  };

  walkGenerator.getLastWalkPhase = [](WalkKickStep& walkKickStep, Pose2f& step, const MotionPhase& lastPhase)
  {
    if(lastPhase.type != MotionPhase::walk)
    {
      step = Pose2f();
      walkKickStep = WalkKickStep();
      return;
    }
    const auto& lastWalkPhase = static_cast<const RERLWalkPhaseDummy&>(lastPhase);
    step = lastWalkPhase.step;
    walkKickStep = lastWalkPhase.walkKickStep;
  };

  walkGenerator.isWalkDelayPossible = [](const MotionPhase&, const float, const Pose2f&, const bool)
  {
    return false;
  };

  walkGenerator.getRotationRange = [this](const bool isLeftPhase, const Pose2f& walkSpeedRatio)
  {
    const Angle rotation = configuredParameters.maxSpeed.rotation;
    const float innerTurn = 2.f * 0.5f * rotation * std::abs(walkSpeedRatio.rotation) * configuredParameters.baseWalkPeriod / 1000.f;
    const float outerTurn = 2.f * (1.f - 0.5f) * rotation * std::abs(walkSpeedRatio.rotation) * configuredParameters.baseWalkPeriod / 1000.f;
    return Rangea(isLeftPhase ? -innerTurn : -outerTurn, isLeftPhase ? outerTurn : innerTurn);
  };

  walkGenerator.getStepRotationRangeOther = [this](const bool isLeftPhase, const Pose2f& walkSpeedRatio, Vector2f step,
                                                   const bool isFastWalk, const std::vector<Vector2f>& translationPolygon,
                                                   const bool ignoreXTranslation, const bool isMaxPossibleStepSize)
  {
    ASSERT(translationPolygon.size() >= 4);
    if(translationPolygon.size() < 4)
      return Rangea(0_deg, 0_deg);

    const float durationValue = configuredParameters.baseWalkPeriod / 1000.f;
    const float defaultMaxSide = 2.f * configuredParameters.maxSpeed.translation.y() * durationValue;

    if(!isMaxPossibleStepSize)
    {
      if(!Geometry::isPointInsideConvexPolygon(translationPolygon.data(), static_cast<int>(translationPolygon.size()), step))
      {
        Vector2f p1;
        VERIFY(Geometry::getIntersectionOfLineAndConvexPolygon(translationPolygon, Geometry::Line(Vector2f(0.f, 0.f),
                                                               step / step.norm()), p1, false));
        step = p1;
      }
    }

    Vector2f stepRatio(0.f, 0.f);
    Rangef xRange(0.1f, 0.1f);
    Rangef yRange(0.1f, 0.1f);
    for(const Vector2f& p : translationPolygon)
    {
      xRange.min = std::min(xRange.min, p.x());
      xRange.max = std::max(xRange.max, p.x());
      yRange.min = std::min(yRange.min, p.y());
      yRange.max = std::max(yRange.max, p.y());
      if(isMaxPossibleStepSize)
      {
        yRange.min = std::max(yRange.min, -defaultMaxSide);
        yRange.max = std::min(yRange.max, defaultMaxSide);
      }
    }

    if(!ignoreXTranslation)
    {
      if(step.x() < 0)
        stepRatio.x() = step.x() / xRange.min;
      else if(step.x() > 0.f)
        stepRatio.x() = step.x() / xRange.max;
    }
    if(step.y() < 0)
      stepRatio.y() = step.y() / yRange.min;
    else if(step.y() > 0.f)
      stepRatio.y() = step.y() / yRange.max;

    stepRatio.x() = Rangef::ZeroOneRange().limit(stepRatio.x());
    stepRatio.y() = Rangef::ZeroOneRange().limit(stepRatio.y());

    // Ensure stability
    if(isMaxPossibleStepSize)
    {
      stepRatio.y() = std::sqrt(stepRatio.y());
      stepRatio.x() = std::sqrt(stepRatio.x());
    }

    const Rangea maxAllowedRotation = getMaxRotationToStepFactor(isFastWalk, isLeftPhase, 0, stepRatio);

    const Angle rotation = configuredParameters.maxSpeed.rotation;
    const float innerTurn = 2.f * stepSizeParameters.insideTurnRatio * rotation * std::abs(walkSpeedRatio.rotation) * configuredParameters.baseWalkPeriod / 1000.f;
    const float outerTurn = 2.f * (1.f - stepSizeParameters.insideTurnRatio) * rotation * std::abs(walkSpeedRatio.rotation) * configuredParameters.baseWalkPeriod / 1000.f;
    return Rangea(isLeftPhase ? maxAllowedRotation.limit(-innerTurn) : maxAllowedRotation.limit(-outerTurn), isLeftPhase ? maxAllowedRotation.limit(outerTurn) : maxAllowedRotation.limit(innerTurn));
  };

  walkGenerator.getStepRotationRange = [&walkGenerator](const bool isLeftPhase, const Pose2f& walkSpeedRatio, Vector2f step,
                                                        const bool isFastWalk, const MotionPhase& lastPhase, const bool ignoreXTranslation, const bool clipTranslation)
  {
    std::vector<Vector2f> translationPolygon;
    std::vector<Vector2f> translationPolygonNoCenter;
    walkGenerator.getTranslationPolygon(isLeftPhase, 0, lastPhase, walkSpeedRatio, translationPolygon, translationPolygonNoCenter, isFastWalk, false);

    return walkGenerator.getStepRotationRangeOther(isLeftPhase, walkSpeedRatio, step, isFastWalk, translationPolygon, ignoreXTranslation, clipTranslation);
  };

  walkGenerator.getTranslationPolygon = [this](const bool isLeftPhase, float rotation, const MotionPhase& lastPhase, const Pose2f& walkSpeedRatio, std::vector<Vector2f>& translationPolygon, std::vector<Vector2f>& translationPolygonNoCenter, const bool fastWalk, const bool useMaxPossibleStepSize)
  {
    bool useFastWalk = fastWalk;
    // After an InWalkKick, the next steps are balance steps to ensure that the robot will not fall
    Vector2f forwardBalance = Vector2f(0.f, 0.f);
    Pose2f useWalkSpeedRatio = walkSpeedRatio;

    Vector2f frontLeft, backRight;
    const Pose2f lastStep = lastPhase.type == MotionPhase::walk ? static_cast<const RERLWalkPhaseDummy&>(lastPhase).step : Pose2f();
    const Vector2f maxStepSizeChange = commonSpeedParameters.maxAcceleration * configuredParameters.baseWalkPeriod / 1000.f;
    const Vector2f maxStepSizeChangeToZero = commonSpeedParameters.maxDeceleration * configuredParameters.baseWalkPeriod / 1000.f;
    if(lastStep.translation.x() > 0.f)
    {
      frontLeft.x() = lastStep.translation.x() + maxStepSizeChange.x();
      backRight.x() = std::max(lastStep.translation.x() - maxStepSizeChangeToZero.x(), 0.f);
    }
    else if(lastStep.translation.x() < 0.f)
    {
      frontLeft.x() = std::min(lastStep.translation.x() + maxStepSizeChangeToZero.x(), 0.f);
      backRight.x() = lastStep.translation.x() - maxStepSizeChange.x();
    }
    else
    {
      frontLeft.x() = maxStepSizeChange.x();
      backRight.x() = -maxStepSizeChange.x(); // when walking circular around the ball, the feet must be allowed to move far backward
    }

    backRight.y() = -1000.f;
    frontLeft.y() = 1000.f;

    // Scale by JointPlay
    Vector2f useSpeed = configuredParameters.maxSpeed.translation;
    const float useSpeedBackwards = configuredParameters.maxSpeedBackwards;

    // Limit to maximum speed (which is influenced by the rotation).
    // When the arms are on the back, the robot will unintentionally turn more in each step.
    // Allow more rotation + translation, so the robot does not walk that much slower
    const Vector2f stepSizeFactor = getStepSizeFactor(rotation, useFastWalk, isLeftPhase, 10_deg, useWalkSpeedRatio.translation);

    backRight.x() = std::max(backRight.x(), stepSizeFactor.x() * -useSpeedBackwards * configuredParameters.baseWalkPeriod / 1000.f);
    backRight.y() = std::max(backRight.y(), stepSizeFactor.y() * -2.f * useSpeed.y() * configuredParameters.baseWalkPeriod / 1000.f);
    frontLeft.x() = std::min(frontLeft.x(), stepSizeFactor.x() * useSpeed.x() * configuredParameters.baseWalkPeriod / 1000.f);
    frontLeft.y() = std::min(frontLeft.y(), stepSizeFactor.y() * 2.f * useSpeed.y() * configuredParameters.baseWalkPeriod / 1000.f);

    // Make sure some minimum exists
    const float useMinXBackwardTranslation = stepSizeParameters.minXBackwardTranslationFastRange.max;
    frontLeft.x() = std::max(stepSizeParameters.minXForwardTranslationFast * useWalkSpeedRatio.translation.x(), frontLeft.x());
    backRight.x() = std::min(useMinXBackwardTranslation * useWalkSpeedRatio.translation.x(), backRight.x());

    // Step size in x translation has a min size
    const float maxMinStepX = std::min(stepSizeParameters.minXTranslationStep, (useWalkSpeedRatio.translation.x() >= 0.f ? useSpeed.x() : useSpeedBackwards) * configuredParameters.baseWalkPeriod / 1000.f * std::abs(useWalkSpeedRatio.translation.x()) + 0.01f);

    backRight.x() = std::min(backRight.x(), -maxMinStepX);
    frontLeft.x() = std::max(frontLeft.x(), maxMinStepX);

    // (0,0) must be part of the rectangle.
    backRight.x() = std::min(backRight.x(), -.01f);
    frontLeft.x() = std::max(frontLeft.x(), .01f);
    backRight.y() = std::min(backRight.y(), -.01f);
    frontLeft.y() = std::max(frontLeft.y(), .01f);

    Vector2f frontLeftNoCenter = frontLeft;
    Vector2f backRightNoCenter = backRight;

    generateTranslationPolygon(translationPolygon, backRight, frontLeft, useMaxPossibleStepSize);
    generateTranslationPolygon(translationPolygonNoCenter, backRightNoCenter, frontLeftNoCenter, useMaxPossibleStepSize);

    ASSERT(translationPolygon.size() != 0);
    ASSERT(translationPolygonNoCenter.size() != 0);
  };

  walkGenerator.generateTranslationPolygon = [this](const bool isLeftPhase, const Angle rotation, const Pose2f& walkSpeedRatio,
                                                    std::vector<Vector2f>& translationPolygon, Vector2f backRight, Vector2f frontLeft,
                                                    const bool useFastWalk, const bool useMaxPossibleStepSize)
  {
    const Vector2f stepSizeFactor = getStepSizeFactor(rotation, useFastWalk, isLeftPhase, 0, walkSpeedRatio.translation);

    // (0,0) must be part of the rectangle.
    const float maxXRatio = std::min(stepSizeFactor.x(), std::max(std::abs(walkSpeedRatio.translation.x()), 0.01f));
    const float maxYRatio = std::min(stepSizeFactor.y(), std::max(std::abs(walkSpeedRatio.translation.y()), 0.01f));
    backRight.x() = std::min(backRight.x() * maxXRatio, -.01f);
    frontLeft.x() = std::max(frontLeft.x() * maxXRatio, .01f);
    backRight.y() = std::min(backRight.y() * maxYRatio, -.01f);
    frontLeft.y() = std::max(frontLeft.y() * maxYRatio, .01f);
    generateTranslationPolygon(translationPolygon, backRight, frontLeft, useMaxPossibleStepSize);
  };

  walkGenerator.getStartOffsetOfNextWalkPhase = [this](const MotionPhase&)
  {
    Pose2f left;
    Pose2f right;
    Pose2f lastStep;

    const RobotModel lastRobotModel(theJointRequest, theRobotDimensions, theMassCalibration);
    left.translate(lastRobotModel.soleLeft.translation.head<2>()).rotate(lastRobotModel.soleLeft.rotation.getZAngle()); // Wrong, because the 0 position is unknown
    right.translate(lastRobotModel.soleRight.translation.head<2>()).rotate(lastRobotModel.soleRight.rotation.getZAngle());

    return std::make_tuple(left, right, lastStep);
  };

  walkGenerator.getLastStepChange = [](const MotionPhase&)
  {
    return Pose2f();
  };

  walkGenerator.getLastStepChangeWithOffsets = [&](const MotionPhase&)
  {
    return std::make_tuple(Pose2f(), Pose2f(), Pose2f());
  };

  walkGenerator.wasLastPhaseInWalkKick = [](const MotionPhase& lastPhase)
  {
    if(lastPhase.type == MotionPhase::walk)
    {
      const auto& lastWalkPhase = static_cast<const RERLWalkPhaseDummy&>(lastPhase);
      return lastWalkPhase.walkKickStep.currentKick != WalkKicks::none;
    }
    return false;
  };
}

float RERLWalkingEngine::rampValue(const float current, const float target, const float maxAccelDelta, const float maxDecelDelta) const
{
  const float delta = target - current;
  if(delta == 0.f)
    return target;
  const bool accelerating = current == 0.f || delta * current > 0.f;
  const float maxDelta = accelerating ? maxAccelDelta : maxDecelDelta;
  if(std::abs(delta) <= maxDelta)
    return target;
  return current + (delta > 0.f ? maxDelta : -maxDelta);
}

Pose2f RERLWalkingEngine::rampCommand(const Pose2f& current, const Pose2f& target, const float dt) const
{
  const float period = configuredParameters.baseWalkPeriod / 1000.f;
  const Vector2f accelTrans = commandRampParameters.maxAcceleration * dt * 1000.f * period;
  const Vector2f decelTrans = commandRampParameters.maxDeceleration * dt * 1000.f * period;
  const float accelRot = commandRampParameters.maxRotationAcceleration * dt * period;
  const float decelRot = commandRampParameters.maxRotationDeceleration * dt * period;

  Pose2f result;
  result.translation.x() = rampValue(current.translation.x(), target.translation.x(), accelTrans.x(), decelTrans.x());
  result.translation.y() = rampValue(current.translation.y(), target.translation.y(), accelTrans.y(), decelTrans.y());
  result.rotation = rampValue(current.rotation, target.rotation, accelRot, decelRot);
  return result;
}

bool RERLWalkingEngine::updateTiltRecovery(const float cmdMag, const Vector3f& gravity)
{
  const TiltRecoveryParameters& p = tiltRecoveryParameters;
  const bool engaged = tiltRecoveryActive();

  // base tilt -- gravity vector projected into the body xy-plane.
  // Forward x pos, right y neg
  const Vector2f tiltXY(gravity.x(), p.lateralEnabled ? gravity.y() : 0.f);
  const Angle pitch = std::abs(theInertialData.angle.y());
  const Angle roll = std::abs(theInertialData.angle.x());
  const bool pitchBeyond = p.pitchTrigger > 0_deg && pitch > (engaged ? p.pitchExit : p.pitchTrigger);
  const bool rollBeyond = p.rollTrigger > 0_deg && roll > (engaged ? p.rollExit : p.rollTrigger);

  // CoM not over the support foot
  const bool leftSupport = theFootSupport.support >= 0.f;
  const float forward = theRobotStableState.comInFeet[leftSupport ? Legs::left : Legs::right].forward;
  const float comBias = forward - p.comForwardNeutral; // +front / -back
  const bool comBeyond = p.comForwardTrigger > 0.f &&
                         std::abs(comBias) > (engaged ? p.comForwardExit : p.comForwardTrigger);

  const bool stateAllowed = !p.restrictToPlayingStates ||
                            !theGameState.gameControllerActive ||
                            ((theGameState.isPlaying() || theGameState.isReady()) && !theGameState.isPenalized());

  const bool allow = p.active &&
                     stateAllowed &&
                     (!p.requireGroundContact || theGroundContactState.contact) &&
                     cmdMag <= p.maxCmdToActivate &&
                     (pitchBeyond || rollBeyond || comBeyond) &&
                     theFrameInfo.time >= tiltRecoveryBlockedUntil &&
                     (!engaged || theFrameInfo.getTimeSince(tiltRecoveryEngagedSince) <= p.maxDuration);

  if(!allow)
  {
    if(engaged)
      tiltRecoveryBlockedUntil = theFrameInfo.time + p.cooldown;
    tiltRecoveryEngagedSince = 0;
    tiltRecoveryCmd.setZero();
    return false;
  }

  Vector2f recover = p.tiltGain * tiltXY + Vector2f(p.comGain * comBias, 0.f); // body-frame velocity (m/s)
  const float norm = recover.norm();
  if(norm < 1e-5f)
    return false;

  if(!engaged)
  {
    tiltRecoveryEngagedSince = theFrameInfo.time;
    ANNOTATION("RERLWalkingEngine", "recovery engaged, pitch " << theInertialData.angle.y() << " roll " << theInertialData.angle.x() << " comFwd " << forward);
    TLOGI(tlogger, "recovery engaged: pitch={}deg roll={}deg comFwd={} cmdMag={}", theInertialData.angle.y().toDegrees(), theInertialData.angle.x().toDegrees(), forward, cmdMag);
  }

  const float minCmd = std::max(p.minCommand, 1.001f * reboostWalkNNParameters.zeroCmdThreshold);
  const float mag = Rangef(minCmd, std::max(p.maxCommand, minCmd)).limit(norm);
  tiltRecoveryCmd = recover * (mag / norm);
  return true;
}

Rangea RERLWalkingEngine::getMaxRotationToStepFactor(const bool isFastWalk, const bool isLeftPhase, const Angle customReduceOffset, const Vector2f& stepRatio)
{
  // rotation based on y-translation
  const float useReduceTranslationFromRotation = !isFastWalk ? stepSizeParameters.reduceTranslationFromRotation.y() : stepSizeParameters.reduceTranslationYFromRotationFast;
  const float reduceTransitionYNeg = !isFastWalk ? stepSizeParameters.noTranslationFromRotation.y() : (!isLeftPhase ? stepSizeParameters.noTranslationYFromRotationFastOuter : stepSizeParameters.noTranslationYFromRotationFastInner);
  const float reduceTransitionYPos = !isFastWalk ? stepSizeParameters.noTranslationFromRotation.y() : (isLeftPhase ? stepSizeParameters.noTranslationYFromRotationFastOuter : stepSizeParameters.noTranslationYFromRotationFastInner);
  const Angle minRotY = -std::sqrt(std::max(0.f, -(stepRatio.y() - 1.f))) * std::max(reduceTransitionYNeg - useReduceTranslationFromRotation, 0.f) - useReduceTranslationFromRotation;
  const Angle maxRotY = std::sqrt(std::max(0.f, -(stepRatio.y() - 1.f))) * std::max(reduceTransitionYPos - useReduceTranslationFromRotation, 0.f) + useReduceTranslationFromRotation;

  // rotation based on x-translation
  const Angle useReduceFactor = std::max(customReduceOffset, stepSizeParameters.reduceTranslationFromRotation.x());
  const Angle rotX = std::sqrt(std::max(0.f, -(stepRatio.x() - 1.f))) * (stepSizeParameters.noTranslationFromRotation.x() - useReduceFactor) + useReduceFactor;

  return Rangea(std::max(minRotY, -rotX), std::min(maxRotY, rotX));
}

Vector2f RERLWalkingEngine::getStepSizeFactor(const Angle rotation, const bool isFastWalk, const bool isLeftPhase, const Angle customReduceOffset, const Vector2f& walkSpeedRatio)
{
  const Angle useReduceFactor = std::max(customReduceOffset, stepSizeParameters.reduceTranslationFromRotation.x());
  const float tFactorX = std::max(0.f, 1.f - sqr(std::max(0.f, (std::abs(rotation) - useReduceFactor) / (stepSizeParameters.noTranslationFromRotation.x() - useReduceFactor))));
  const float reduceTransitionY = !isFastWalk ? stepSizeParameters.noTranslationFromRotation.y() : ((isLeftPhase && rotation > 0_deg) || (!isLeftPhase && rotation < 0_deg) ? stepSizeParameters.noTranslationYFromRotationFastOuter : stepSizeParameters.noTranslationYFromRotationFastInner);
  const float useReduceTranslationFromRotation = !isFastWalk ? stepSizeParameters.reduceTranslationFromRotation.y() : stepSizeParameters.reduceTranslationYFromRotationFast;
  const float tFactorY = std::max(0.f, 1.f - sqr(std::max(0.f, (std::abs(rotation) - useReduceTranslationFromRotation) / (reduceTransitionY - useReduceTranslationFromRotation))));
  return Vector2f(std::min(tFactorX, std::abs(walkSpeedRatio.x())), std::min(tFactorY, std::abs(walkSpeedRatio.y())));
}

void RERLWalkingEngine::filterTranslationPolygon(std::vector<Vector2f>& polygonOut, std::vector<Vector2f>& polygonIn, const std::vector<Vector2f>& polygonOriginal)
{
  // adjust y forward
  Geometry::Line lineForwardAdjusted(polygonIn[1], (polygonIn[1] - polygonIn[2]).normalized());
  Vector2f leftY;
  if(Geometry::getIntersectionOfLineAndConvexPolygon(polygonOriginal, lineForwardAdjusted, leftY, false))
  {
    polygonIn[1].y() = std::min(leftY.y(), polygonIn[0].y());
    polygonIn[2].y() = std::max(-leftY.y(), polygonIn[3].y());
  }

  // adjust y backward
  Geometry::Line lineBackAdjusted(polygonIn[5], (polygonIn[6] - polygonIn[5]).normalized());
  if(Geometry::getIntersectionOfLineAndConvexPolygon(polygonOriginal, lineBackAdjusted, leftY, false))
  {
    polygonIn[5].y() = std::max(-leftY.y(), polygonIn[4].y());
    polygonIn[6].y() = std::min(leftY.y(), polygonIn[7].y());
  }

  polygonOut.clear();

  for(size_t i = 0; i < polygonIn.size(); ++i)
  {
    const Vector2f& p1 = polygonIn[i];
    const Vector2f& p2 = polygonIn[(i + 1) % polygonIn.size()];
    if(p1 != p2)
      polygonOut.emplace_back(p1);
  }
}

void RERLWalkingEngine::generateTranslationPolygon(std::vector<Vector2f>& polygon, const Vector2f& backRight, const Vector2f& frontLeft, const bool)
{
  ASSERT(!translationPolygon.empty());
  const std::vector<Vector2f>& original = translationPolygon;
  ASSERT(original.size() == 8);
  std::vector<Vector2f> translationPolygonTemp = original;
  for(Vector2f& edge : translationPolygonTemp)
  {
    // x
    if(edge.x() >= 0.f)
      edge.x() = std::min(edge.x(), frontLeft.x());
    else
      edge.x() = std::max(edge.x(), backRight.x());

    // y
    if(edge.y() >= 0.f)
      edge.y() = std::min(edge.y(), frontLeft.y());
    else
      edge.y() = std::max(edge.y(), backRight.y());
  }

  if(frontLeft.x() < 0.f)
  {
    translationPolygonTemp[0].x() = translationPolygonTemp[1].x() = std::max(original[7].x(), translationPolygonTemp[0].x());
    translationPolygonTemp[2].x() = translationPolygonTemp[3].x() = std::max(original[4].x(), translationPolygonTemp[3].x());
  }
  if(backRight.x() > 0.f)
  {
    translationPolygonTemp[7].x() = translationPolygonTemp[6].x() = std::min(original[0].x(), translationPolygonTemp[7].x());
    translationPolygonTemp[4].x() = translationPolygonTemp[5].x() = std::min(original[3].x(), translationPolygonTemp[4].x());
  }

  ASSERT(translationPolygonTemp[3].x() == translationPolygonTemp[0].x());
  ASSERT(translationPolygonTemp[4].x() == translationPolygonTemp[7].x());

  // back right
  translationPolygonTemp[4].x() = translationPolygonTemp[7].x();

  filterTranslationPolygon(polygon, translationPolygonTemp, original);
}

RERLWalkPhaseDummy::RERLWalkPhaseDummy(RERLWalkingEngine& engine, Pose2f useStepTarget, const MotionPhase& lastPhase, const bool,
                         const WalkGenerator::CreateNextPhaseCallback& callBack,
                         const WalkKickStep& walkKickStep):
  MotionPhase(MotionPhase::walk),
  walkKickStep(walkKickStep),
  engine(engine),
  callBack(callBack)
{
  step = useStepTarget;
  bool standRequested = (step == Pose2f());
  bool slowWalkRequested =
      (std::abs(step.rotation) < engine.reboostWalkNNParameters.slowWalkStepSpeed.rotation &&
       std::abs(step.translation.x()) < engine.reboostWalkNNParameters.slowWalkStepSpeed.translation.x() &&
       std::abs(step.translation.y()) < engine.reboostWalkNNParameters.slowWalkStepSpeed.translation.y());

  slowingDownToStandStartTime = 0; // only non-zero if actually slowing down
  tBase = engine.motionCycleTime;

  lastTarget.angles = engine.theJointRequest.angles;
  lastModelRequest = engine.theFrameInfo.time;

  lastTimeWalking = 0;
  interpStartTime = 0; // by default we are not interpolating

  // helper to ensure we always start interpolation the same way
  // (useful if we need it for transitions other than just from prepare)
  auto beginInterpolation = [&](unsigned durationMs)
  {
    interpStartTime = engine.theFrameInfo.time;
    interpDurationMs = durationMs;
    // const std::vector<Joints::Joint> jointList = getBoosterLegJointSequence();
    // for (Joints::Joint j : jointList)
    //   interpStartAngles.angles[j] = engine.theJointAngles.angles[j];

    FOREACH_ENUM(Joints::Joint, joint)
    {
      interpStartAngles.angles[joint] = engine.theJointRequest.stiffnessData.stiffnesses[joint] > 0
                                            ? engine.theJointRequest.angles[joint]
                                            : engine.theJointAngles.angles[joint];
    }

    TLOGI(walkPhaseLogger, "begin interpolation to model outputs");
  };

  // handle transitions from different motions as needed
  if (lastPhase.type == MotionPhase::walk || lastPhase.type == MotionPhase::stand)
  {
    const auto& lastWalkPhase = static_cast<const RERLWalkPhaseDummy&>(lastPhase);
    tBase = lastWalkPhase.tBase;
    lastTarget = lastWalkPhase.lastTarget;
    lastModelRequest = lastWalkPhase.lastModelRequest;
    nextTarget = lastWalkPhase.nextTarget;
    interpStartTime = lastWalkPhase.interpStartTime;
    // Carry the frequency state across phases
    rawFrequency = lastWalkPhase.rawFrequency;
    frequency = lastWalkPhase.frequency;
    armRecoveryStartTime = lastWalkPhase.armRecoveryStartTime;
    armRecoveryElbowYawDuration = lastWalkPhase.armRecoveryElbowYawDuration;
    armRecoveryArmDuration = lastWalkPhase.armRecoveryArmDuration;
    armRecoveryStartAngles = lastWalkPhase.armRecoveryStartAngles;

    if (lastPhase.type == MotionPhase::stand)
      tBase = engine.theWalkGenerator.isNextLeftPhase(lastPhase, useStepTarget) ? 0.f : 0.5f;

    slowingDownToStandStartTime =
        slowWalkRequested ? lastWalkPhase.slowingDownToStandStartTime : engine.theFrameInfo.time;
  }
  else if (lastPhase.type == MotionPhase::prepare)
  {
    beginInterpolation(2000u); // FIXME make configurable
  }
  else if(engine.armRecoveryParameters.active)
  {
    auto getAngle = [&](Joints::Joint j) -> Angle
    {
      return engine.theJointRequest.stiffnessData.stiffnesses[j] > 0
             ? engine.theJointRequest.angles[j]
             : engine.theJointAngles.angles[j];
    };

    const Angle threshold = engine.armRecoveryParameters.deviationThreshold;
    const bool deviated = (
      (std::abs(getAngle(Joints::lShoulderPitch) - engine.armParameters.armShoulderPitch) > threshold)
      || (std::abs(getAngle(Joints::lShoulderRoll) - engine.armParameters.armShoulderRoll) > threshold)
      || (std::abs(getAngle(Joints::lElbowYaw) - engine.armParameters.armElbowYaw) > threshold)
      || (std::abs(getAngle(Joints::lElbowRoll) - (-30_deg)) > threshold)
      || (std::abs(getAngle(Joints::rShoulderPitch) - engine.armParameters.armShoulderPitch) > threshold)
      || (std::abs(getAngle(Joints::rShoulderRoll) - (-engine.armParameters.armShoulderRoll)) > threshold)
      || (std::abs(getAngle(Joints::rElbowYaw) - (-engine.armParameters.armElbowYaw)) > threshold)
      || (std::abs(getAngle(Joints::rElbowRoll) - 30_deg) > threshold)
    );
    if(deviated)
    {
      armRecoveryStartTime = engine.theFrameInfo.time;
      armRecoveryElbowYawDuration = engine.armRecoveryParameters.elbowYawDuration;
      armRecoveryArmDuration = engine.armRecoveryParameters.fullArmDuration;
      FOREACH_ENUM(Joints::Joint, joint)
        armRecoveryStartAngles.angles[joint] = getAngle(joint);
    }
  }

  // Motion-level command ramping
  const bool isWalkKick = walkKickStep.currentKick != WalkKicks::none;
  const bool rampActive = engine.commandRampParameters.active && !isWalkKick;
  if (rampActive)
  {
    // The ramp is not used in fall, getUp, kick, prepare, playDead
    if (lastPhase.type != MotionPhase::walk && lastPhase.type != MotionPhase::stand)
    {
      engine.lastRampedStep = Pose2f();
      if (lastPhase.type == MotionPhase::fall || lastPhase.type == MotionPhase::getUp ||
          lastPhase.type == MotionPhase::freeze || lastPhase.type == MotionPhase::playDead)
        engine.lastInstabilityPhaseEnd = engine.theFrameInfo.time;
    }
    const bool settling = engine.commandRampParameters.settleTimeAfterInstability > 0 && engine.lastInstabilityPhaseEnd &&
                          engine.theFrameInfo.getTimeSince(engine.lastInstabilityPhaseEnd) < engine.commandRampParameters.settleTimeAfterInstability;
    const Pose2f rampTarget = standRequested || settling ? Pose2f() : step; // ramp toward zero when a stand is requested
    float dt = engine.theFrameInfo.getTimeSince(engine.lastRampTime) / 1000.f;
    dt = std::min(std::max(dt, 0.f), engine.commandRampParameters.maxRampTimeStep);
    step = engine.rampCommand(engine.lastRampedStep, rampTarget, dt);
    engine.lastRampedStep = step;
    engine.lastRampTime = engine.theFrameInfo.time;
  }

  if (standRequested)
  {
    // Standing is only allowed after we're definitely slow enough to prevent face planting
    const bool timeGateOpen = !(slowingDownToStandStartTime &&
        (engine.theFrameInfo.getTimeSince(slowingDownToStandStartTime) < engine.reboostWalkNNParameters.walkToStandSlowdownMs));

    const float invPeriod = 1.f / (engine.configuredParameters.baseWalkPeriod / 1000.f);
    const float cmdMag = Vector3f(step.translation.x() * invPeriod / 1000.f,
                                  step.translation.y() * invPeriod / 1000.f,
                                  step.rotation * invPeriod).norm();
    const bool slowedEnough = !rampActive || cmdMag <= engine.reboostWalkNNParameters.zeroCmdThreshold;

    if (timeGateOpen && slowedEnough)
    {
      type = MotionPhase::stand;
      tBase = 0.f;
      engine.lastRampedStep = Pose2f(); // command is now zero
    }
    else if (!rampActive)
    {
      step = Pose2f(0.001f, 0.f); // legacy behaviour: keep walking slowly until the time gate opens
      // MotionPhase::walk was already set as default type on MotionPhase initializer
    }
  }

  // Handle walking commands below zeroCmdThreshold
  if (type == MotionPhase::walk && !standRequested && !isWalkKick)
  {
    const float invPeriod = 1.f / (engine.configuredParameters.baseWalkPeriod / 1000.f);
    const float cmdMag = Vector3f(step.translation.x() * invPeriod / 1000.f,
                                  step.translation.y() * invPeriod / 1000.f,
                                  step.rotation * invPeriod).norm();
    const float minCmd = engine.reboostWalkNNParameters.zeroCmdThreshold;
    if (cmdMag > 0.f && cmdMag < minCmd)
    {
      const float scale = 1.001f * minCmd / cmdMag;
      step.translation *= scale;
      step.rotation *= scale;
    }
  }

  // Set walk values
  engine.theWalkStepData.updateWalkValues(step, 0.2f, isLeftPhase);
  //lastTarget = engine.theJointAngles;
  lastModelRequest = engine.theFrameInfo.time;
  getNextTargetRequest(nextTarget);

  isLeftPhase = type == MotionPhase::stand || tBase < 0.5f;

  tWalk = 0.f;

  if (type == MotionPhase::walk)
    lastTimeWalking = engine.theFrameInfo.time;

  // else
  // if (engine.theFrameInfo.getTimeSince(interpStartTime) >= 2000)
  // {
  //   FOREACH_ENUM(Joints::Joint, joint)
  //   {
  //     requestAtStandStart.angles[joint] = engine.theJointRequest.stiffnessData.stiffnesses[joint] > 0 ?
  //                                         engine.theJointRequest.angles[joint] :
  //                                         engine.theJointAngles.angles[joint];
  //   }
  // }

  if (!engine.frequencyParameters.active)
    frequency = rawFrequency = engine.frequencyParameters.base;
}

void RERLWalkPhaseDummy::getNextTargetRequest(JointAngles& target)
{
  float* input = engine.network.input(0).data();
  [[maybe_unused]] float* startOfInput = input;

  const Vector3f gravity = engine.theInertialData.orientation3D.inverse() * Vector3f(0.f, 0.f, -1.f);

  // angular momentum
  *input++ = engine.theInertialData.gyro.x();
  *input++ = engine.theInertialData.gyro.y();
  *input++ = engine.theInertialData.gyro.z();

  // gravity
  *input++ = gravity.x();
  *input++ = gravity.y();
  *input++ = gravity.z();

  // walk command (m/s, m/s, rad/s) — derived from the step target and base walk period
  const float invPeriod = 1.f / (engine.configuredParameters.baseWalkPeriod / 1000.f);
  float cmdX   = type == MotionPhase::walk ? step.translation.x() * invPeriod / 1000.f : 0.f;
  float cmdY   = type == MotionPhase::walk ? step.translation.y() * invPeriod / 1000.f : 0.f;
  float cmdYaw = type == MotionPhase::walk ? step.rotation        * invPeriod          : 0.f;

  // Tilt recovery:
  if(engine.updateTiltRecovery(Vector3f(cmdX, cmdY, cmdYaw).norm(), gravity))
  {
    cmdX = engine.tiltRecoveryCmd.x();
    cmdY = engine.tiltRecoveryCmd.y();
    cmdYaw = 0.f;
  }

  standing = Vector3f(cmdX, cmdY, cmdYaw).norm() <= engine.reboostWalkNNParameters.zeroCmdThreshold;

  *input++ = cmdX;
  *input++ = cmdY;
  *input++ = cmdYaw;

  // joint sequence
  std::vector<Joints::Joint> jointList = getBoosterLegJointSequence();

  // Measurements
  for(Joints::Joint j : jointList)
    *input++ = engine.theJointAngles.angles[j] - engine.offset.angles[j];

  // Velocities
  for(Joints::Joint j : jointList)
    *input++ = engine.theJointAngles.velocity[j] * engine.reboostWalkNNParameters.velocityFactor;

  // Last Requests
  for(Joints::Joint j : jointList)
    *input++ = engine.theJointRequest.angles[j] - engine.offset.angles[j];

  if(engine.reboostWalkNNParameters.enablePhaseInputs)
  {
    // Tilt recovery also runs in stand phases, where the policy must see a live phase clock to step.
    const bool phaseActive = (type == MotionPhase::walk && !standing) || engine.tiltRecoveryActive();
    *input++ = phaseActive ? std::cos(2.f * Constants::pi * tBase) : 0.f;
    *input++ = phaseActive ? std::sin(2.f * Constants::pi * tBase) : 0.f;
  }

  if(engine.frequencyParameters.active)
    *input++ = rawFrequency;

  if(engine.ballNetworkParameters.active)
  {
    // Ball position (3D)
    *input++ = 0; // x
    *input++ = 0; // y
    *input++ = 0; // z

    // Ball velocity (3D)
    *input++ = 0; // x
    *input++ = 0; // y
    *input++ = 0; // z

    // Ball direction as sin and cos
    *input++ = 0; // sin(direction), 0 for direction 0deg
    *input++ = 1; // cos(direction), 1 for direction 0deg

    // Ball range
    *input++ = 0;

    // Play ball (1) or not play ball (0)
    *input++ = 0;

    // Just walk (1) or not just walk (0)
    *input++ = 1;
  }

  ASSERT((input - startOfInput) == engine.networkInputSize);

  // Run network.
  STOPWATCH("module:RERLWalkingEngine:apply")
    engine.network.apply();

  // Get next request
  const float* output = engine.network.output(0).data();
  [[maybe_unused]] const float* startOfOutput = output;

  for(Joints::Joint j : jointList)
    target.angles[j] = engine.reboostWalkNNParameters.actionClipRange.limit(*output++) + engine.offset.angles[j];

  if(engine.frequencyParameters.active)
  {
    rawFrequency = Rangef::OneRange().limit(*output++);
    frequency = engine.frequencyParameters.clipRange.limit(rawFrequency) + engine.frequencyParameters.base;
  }

  ASSERT((output - startOfOutput) == engine.networkOutputSize);
}

void RERLWalkPhaseDummy::update()
{
  logInterval.updateTriggered(engine.theFrameInfo.time, 2000u);

  if(lastModelRequest > engine.theFrameInfo.time)
    lastModelRequest = engine.theFrameInfo.time;

  if(type == MotionPhase::walk || engine.tiltRecoveryActive()) // recovery steps must keep odometry alive
    lastTimeWalking = engine.theFrameInfo.time;

  tWalk += engine.motionCycleTime;
  // Freeze the gait clock while effectively standing so the phase does not drift; it then
  // restarts cleanly from the current value once a non-zero command returns.
  const bool phaseActive = (type == MotionPhase::walk && !standing) || engine.tiltRecoveryActive();
  tBase = std::fmod(tBase + engine.motionCycleTime * frequency * (phaseActive ? 1.f : 0.f), 1.f);
  if(type == MotionPhase::stand && engine.theFrameInfo.getTimeSince(lastModelRequest) >= 20)
  {
    lastModelRequest = engine.theFrameInfo.time;

    getNextTargetRequest(nextTarget);
  }
}

bool RERLWalkPhaseDummy::isDone(const MotionRequest& motionRequest) const
{
  // Does not work with WalkKickEngine
  return type == MotionPhase::stand ? motionRequest.motion != MotionRequest::stand // switch from stand to walk
         : engine.theFrameInfo.getTimeSince(lastModelRequest) >= 20 && tWalk > engine.motionCycleTime;
}

void RERLWalkPhaseDummy::calcJoints(const MotionRequest&, JointRequest& jointRequest, Pose2f& odometryOffset, MotionInfo& motionInfo)
{
  // if (logInterval.wasTriggered())
  //   TLOGD(walkPhaseLogger, "calcJoints - setting stiffnesses to 100");

  jointRequest.stiffnessData.stiffnesses.fill(100);

  const Angle lSP = engine.armParameters.armShoulderPitch + engine.theRobotModel.soleLeft.translation.x()  * engine.armParameters.armShoulderPitchFactor / 1000.f;
  const Angle lSR = engine.armParameters.armShoulderRoll  + std::abs(engine.theRobotModel.soleLeft.translation.y())  * engine.armParameters.armShoulderRollIncreaseFactor / 1000.f;
  const Angle lEY = engine.armParameters.armElbowYaw;
  const Angle lER = -30_deg;
  const Angle rSP = engine.armParameters.armShoulderPitch + engine.theRobotModel.soleRight.translation.x() * engine.armParameters.armShoulderPitchFactor / 1000.f;
  const Angle rSR = -engine.armParameters.armShoulderRoll - std::abs(engine.theRobotModel.soleRight.translation.y()) * engine.armParameters.armShoulderRollIncreaseFactor / 1000.f;
  const Angle rEY = -engine.armParameters.armElbowYaw;
  const Angle rER = 30_deg;

  jointRequest.angles[Joints::lShoulderPitch] = lSP;
  jointRequest.angles[Joints::lShoulderRoll]  = lSR;
  jointRequest.angles[Joints::lElbowYaw]      = lEY;
  jointRequest.angles[Joints::lElbowRoll]     = lER;
  jointRequest.angles[Joints::lWristYaw]      = -90_deg;
  jointRequest.angles[Joints::lHand]          = 0.f;
  jointRequest.angles[Joints::rShoulderPitch] = rSP;
  jointRequest.angles[Joints::rShoulderRoll]  = rSR;
  jointRequest.angles[Joints::rElbowYaw]      = rEY;
  jointRequest.angles[Joints::rElbowRoll]     = rER;
  jointRequest.angles[Joints::rWristYaw]      = 90_deg;
  jointRequest.angles[Joints::rHand]          = 0.f;

  jointRequest.angles[Joints::waistYaw] = 0_deg;

  // check if we need to interpolate from some previous joint setting to model outputs
  if ((SystemCall::getMode() != SystemCall::simulatedRobot) && interpStartTime)
  {
    // take target arm joints from just above
    for (std::size_t j = Joints::firstArmJoint; j < Joints::firstLegJoint; j++) // override arm joints
      standTarget.angles[j] = jointRequest.angles[j];

    // take target leg joints from the policy's nextTarget (obtained in getNextTargetRequest)
    std::vector<Joints::Joint> jointList = getBoosterLegJointSequence();
    for(Joints::Joint j : jointList)
    {
      lastTarget.angles[j] = nextTarget.angles[j];
      standTarget.angles[j] = lastTarget.angles[j];
    }

    // crossfade from all start angles to all standTarget angles over the interpolation period
    const float tNorm = Rangef::ZeroOneRange().limit(engine.theFrameInfo.getTimeSince(interpStartTime) /
                                                     static_cast<float>(interpDurationMs));
    const float alpha = tNorm * tNorm * (3.f - 2.f * tNorm); // the smoothstep function / ease-in, ease-out

    for (std::size_t j = Joints::firstArmJoint; j < Joints::numOfJoints; j++) // leave out head joints
      jointRequest.angles[j] = interpStartAngles.angles[j] + alpha * (standTarget.angles[j] - interpStartAngles.angles[j]);

    if (engine.theFrameInfo.getTimeSince(interpStartTime) >= static_cast<int>(interpDurationMs))
      interpStartTime = 0; // no longer interpolating
  }
  else // no interpolation - just apply the model output
  {
    std::vector<Joints::Joint> jointList = getBoosterLegJointSequence();
    for(Joints::Joint j : jointList)
    {
      lastTarget.angles[j] = nextTarget.angles[j];
      jointRequest.angles[j] = lastTarget.angles[j];
    }
    standTarget = nextTarget;

    if(armRecoveryStartTime)
    {
      const float t = static_cast<float>(engine.theFrameInfo.getTimeSince(armRecoveryStartTime));
      const float eyRatio  = Rangef::ZeroOneRange().limit(t / armRecoveryElbowYawDuration);
      const float armRatio = Rangef::ZeroOneRange().limit((t - armRecoveryElbowYawDuration) / armRecoveryArmDuration);

      auto lerp = [](Angle a, Angle b, float r) { return a * (1.f - r) + b * r; };

      // Phase 1 (elbow yaw first — prevents knee clipping)
      jointRequest.angles[Joints::lElbowYaw] = lerp(armRecoveryStartAngles.angles[Joints::lElbowYaw], lEY, eyRatio);
      jointRequest.angles[Joints::rElbowYaw] = lerp(armRecoveryStartAngles.angles[Joints::rElbowYaw], rEY, eyRatio);

      // Phase 2 (remaining arm joints)
      jointRequest.angles[Joints::lShoulderPitch] = lerp(armRecoveryStartAngles.angles[Joints::lShoulderPitch], lSP, armRatio);
      jointRequest.angles[Joints::lShoulderRoll] = lerp(armRecoveryStartAngles.angles[Joints::lShoulderRoll], lSR, armRatio);
      jointRequest.angles[Joints::lElbowRoll] = lerp(armRecoveryStartAngles.angles[Joints::lElbowRoll], lER, armRatio);
      jointRequest.angles[Joints::rShoulderPitch] = lerp(armRecoveryStartAngles.angles[Joints::rShoulderPitch], rSP, armRatio);
      jointRequest.angles[Joints::rShoulderRoll] = lerp(armRecoveryStartAngles.angles[Joints::rShoulderRoll], rSR, armRatio);
      jointRequest.angles[Joints::rElbowRoll] = lerp(armRecoveryStartAngles.angles[Joints::rElbowRoll], rER, armRatio);

      if(t >= armRecoveryElbowYawDuration + armRecoveryArmDuration)
        armRecoveryStartTime = 0;
    }
  }

  motionInfo.isMotionStable = true;
  motionInfo.isWalkPhaseInWalkKick = walkKickStep.currentKick != WalkKicks::none;
  const float stepDuration = engine.configuredParameters.baseWalkPeriod / 1000.f;

  if (type == MotionPhase::walk)
    motionInfo.speed =
        Pose2f(step.rotation / stepDuration, step.translation.x() / stepDuration, step.translation.y() / stepDuration);
  else
    motionInfo.speed = Pose2f();

  if (engine.theFrameInfo.getTimeSince(lastTimeWalking) >= 2000)
    odometryOffset = Pose2f(0_deg, 0.f, 0.f);
  else
    odometryOffset = engine.theOdometryDataPreview.odometryChange;
}

std::unique_ptr<MotionPhase> RERLWalkPhaseDummy::createNextPhase(const MotionPhase& defaultNextPhase) const
{
  // There must be another walk phase if the robot should transition into something other than walking and the feet are not next to each other.
  if(defaultNextPhase.type != MotionPhase::walk && type != MotionPhase::stand)
    return std::make_unique<RERLWalkPhaseDummy>(engine, Pose2f(), *this);
  if(std::unique_ptr<MotionPhase> nextPhase; callBack && (nextPhase = callBack(*this, this->walkKickStep)))
    return nextPhase;
  return std::unique_ptr<MotionPhase>();
}

std::vector<Joints::Joint> RERLWalkPhaseDummy::getBoosterLegJointSequence()
{
  if(Global::getSettings().robotType != Settings::nao)
  {
    if(engine.reboostWalkNNParameters.useWaist)
      return engine.boosterWaistJoints;
    return engine.boosterJoints;
  }
  return engine.naoJoints;
}

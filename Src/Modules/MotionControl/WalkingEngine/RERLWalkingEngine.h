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

#pragma once

#include "Framework/Module.h"
#include "Framework/Settings.h"
#include "Representations/Configuration/JointLimits.h"
#include "Representations/Configuration/MassCalibration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Infrastructure/StiffnessData.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/MotionControl/StandGenerator.h"
#include "Representations/MotionControl/WalkGenerator.h"
#include "Representations/MotionControl/WalkStepData.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/FootSupport.h"
#include "Representations/Sensing/InertialData.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/RobotStableState.h"
#include "Representations/Sensing/SolePressureState.h"
#include "Tools/Motion/MotionPhase.h"
#include "Tools/RECommon/IntervalTrigger.h"

#include "Platform/File.h"
#include <CompiledNN2ONNX/CompiledNN.h>

using namespace NeuralNetworkONNX;

STREAMABLE(BoosterArmParameters,
{,
  (Angle) armShoulderRoll, /**< Arm shoulder angle in radians. */
  (float) armShoulderRollIncreaseFactor, /**< Factor between sideways step size (in m) and additional arm roll angles. */
  (float) armShoulderPitchFactor, /**< Factor between forward foot position (in m) and arm pitch angles. */
  (Angle) armShoulderPitch, /**< Arm shoulder pitch angle. */
  (Angle) armElbowYaw, /**< Arm elbow yaw angle. */
});

STREAMABLE(BoosterCommonSpeedParameters,
{,
  (Vector2f) maxAcceleration, /**< Maximum acceleration of forward and sideways speed at each leg change to ratchet up/down in (mm/s/step). */
  (Vector2f) maxDeceleration, /**< (Positive) maximum deceleration of forward and sideways speed at each leg change to ratchet up/down in (mm/s/step). */
});

STREAMABLE(CommandRampParameters,
{,
  (bool)(true) active, /**< Enable motion-level ramping (rate limiting) of the body velocity command. */
  (Vector2f)(Vector2f(3.f, 3.f)) maxAcceleration, /**< Max increase of the linear velocity command magnitude, in (m/s)/s. */
  (Vector2f)(Vector2f(4.f, 4.f)) maxDeceleration, /**< Max (positive) decrease of the linear velocity command toward zero / on reversal, in (m/s)/s. */
  (Angle)(360_deg) maxRotationAcceleration, /**< Max increase of the angular velocity command magnitude, in (rad/s)/s. */
  (Angle)(540_deg) maxRotationDeceleration, /**< Max (positive) decrease of the angular velocity command toward zero / on reversal, in (rad/s)/s. */
  (float)(0.06f) maxRampTimeStep, /**< Upper clamp on the ramp integration time step [s], guards against large gaps (e.g. resuming from stand). */
  (int)(0) settleTimeAfterInstability, /**< Hold the ramp target at zero for this long (ms) after fall/getUp/freeze/playDead phases. 0 = off. */
});

STREAMABLE(TiltRecoveryParameters,
{,
  (bool)(false) active, /**< Enable capture-step commands */

  (Angle)(12_deg) pitchTrigger, /**< |pitch| beyond which recovery engages. */
  (Angle)(6_deg) pitchExit, /**< Hysteresis: once engaged, the pitch trigger releases below this |pitch|. */
  (Angle)(0_deg) rollTrigger, /**< |roll| trigger. */
  (Angle)(0_deg) rollExit, /**< Hysteresis exit for roll. */

  (float)(0.5f) comForwardNeutral, /**< Value of comInFeet.forward when balanced fore-aft. */
  (float)(0.f) comForwardTrigger, /**< |forward - neutral| beyond which recovery engages. */
  (float)(0.15f) comForwardExit, /**< Hysteresis exit for the CoM trigger. */

  (float)(2.f) tiltGain, /**< Command per unit gravity xy-projection. */
  (float)(1.5f) comGain, /**< Command per unit of forward CoM bias. */
  (bool)(false) lateralEnabled, /**< Allow a sideways (y) recovery command component (tilt-derived). */
  (float)(0.f) minCommand, /**< Floor for the recovery command (m/s). */
  (float)(0.5f) maxCommand, /**< Clip for the recovery command magnitude (m/s). */

  (float)(0.25f) maxCmdToActivate, /**< Only engage while the current command magnitude is below this. */
  (bool)(true) requireGroundContact, /**< Require ground contact to engage. */
  (int)(1500) maxDuration, /**< Give up after this long (ms) continuously engaged (e.g. leaning against an obstacle). */
  (int)(500) cooldown, /**< No reengagement for this long (ms) after disengaging. */
  (bool)(true) restrictToPlayingStates, /**< Only engage in playing/ready and unpenalized (or without an active GameController). */
});

STREAMABLE(BoosterStepSizeParameters,
{,
  (float) insideTurnRatio, /**< The ratio of turn that can be done in the inner step of a double-step. */
  (Vector2a) reduceTranslationFromRotation, /**< The rotation from which the translation per step will be reduced. */
  (Vector2a) noTranslationFromRotation, /**< The rotation from which no translation is possible at all. */
  (Angle) noTranslationYFromRotationFastInner, /**< When walking fast with a rotation outwards, the rotation from which no translation is possible at all is higher for the y translation. */
  (Angle) noTranslationYFromRotationFastOuter, /**< When walking fast with a rotation inwards, the rotation from which no translation is possible at all is higher for the y translation. */
  (Angle) reduceTranslationYFromRotationFast, /**< When walking fast, the rotation from which the translation per step will be reduced is higher for the y translation. */
  (float) minXTranslationStep, /**< The forward and backward step size has a minimum. */
  (float) minXForwardTranslationFast, /**< The forward step size has a minimum for the fast translation polygon. */
  (Rangef) minXBackwardTranslationFastRange,/**< The backward step size has a minimum for the fast translation polygon. */
  (Rangef) translationPolygonSafeRange, /**< For the calculation of the translation polygon, the feet are adjusted by this amount in x translation,
                                             to ensure a possible pose even when the arms are on the back or the feet are balanced. */
  (int) noFastTranslationPolygonStepsNumber, /**< Number of steps before big steps near the target are allowed. */
});

STREAMABLE(BoosterConfiguredParameters,
{,
  (Pose2f) maxSpeed, /**< Maximum speeds in mm/s and degrees/s. */
  (float) maxSpeedBackwards, /**< Maximum backwards speed. Positive, in mm/s. */
  (Pose2f) thresholdStopStandTransition, /**< Threshold to be able to switch from walk state stopping to standing. */
  (float) baseWalkPeriod,
});

STREAMABLE(FrequencyParameters,
{,
  (float) base, /**< Base frequency. */
  (Rangef) clipRange, /**< Clip frequency offset from neuronal network. */
  (bool) active, /**< Does the neuronal network output a frequency offset? */
});

STREAMABLE(BallNetworkParameters,
{,
  (bool) active,
});

STREAMABLE(ArmRecoveryParameters,
{,
  (bool) active,               /**< Enable arm recovery interpolation on deviated transitions. */
  (Angle) deviationThreshold,  /**< Per-joint deviation from nominal that triggers recovery. */
  (float) elbowYawDuration,    /**< Duration (ms) of Phase 1: elbow yaw interpolation. */
  (float) fullArmDuration,     /**< Duration (ms) of Phase 2: all other arm joints interpolation. */
});

STREAMABLE(ReboostWalkNNParameters,
{,
  (std::string) modelName, /** Neural Network name. Must be .onnx or .hdf5. */
  (float) velocityFactor, /**< Factor for the joint velocities. */
  (Rangef) actionClipRange, /**< Clip action output. */
  (bool) useWaist, /**< Is the waist also part of the action output. */
  (unsigned) numInput, /**< DEPRECATED (no point having it here) Neural Network input size. */
  (unsigned) numOutput, /**< DEPRECATED (no point having it here) Neural Network output size. */
  (bool) enablePhaseInputs, /**< Allows you to disable the phase inputs to the model */
  (int) walkToStandSlowdownMs, /**< To allow standing, the robot must walk slow for this time. */
  (Pose2f) slowWalkStepSpeed, /**< Speed values to classify slow walking. This is based on the step size. */
  (float) zeroCmdThreshold, /**< Command norm (m/s, rad/s) below which we are standing: phase obs zeroed, gait clock frozen. Mirrors training zero_cmd_threshold. */
});

MODULE(RERLWalkingEngine,
{,
  REQUIRES(FootSupport),
  REQUIRES(FrameInfo),
  REQUIRES(GameState),
  REQUIRES(GroundContactState),
  REQUIRES(InertialData),
  REQUIRES(JointAngles),
  USES(JointRequest),
  REQUIRES(MassCalibration),
  REQUIRES(OdometryDataPreview),
  REQUIRES(RobotDimensions),
  REQUIRES(RobotModel),
  REQUIRES(RobotStableState),
  REQUIRES(SolePressureState),
  PROVIDES(WalkStepData),
  USES(WalkStepData),
  PROVIDES(WalkGenerator),
  REQUIRES(WalkGenerator),
  PROVIDES(StandGenerator),
  PROVIDES(WalkingEngineOutput),
  LOADS_PARAMETERS(
  {,
    (BoosterConfiguredParameters) configuredParameters,   /**< Speed parameters */
    (BoosterStepSizeParameters) stepSizeParameters,       /**< Step size parameters. */
    (BoosterCommonSpeedParameters) commonSpeedParameters, /**< Acceleration parameters. */
    (BoosterArmParameters) armParameters,                 /**< Arm parameters. */
    (ReboostWalkNNParameters) reboostWalkNNParameters,    /**< Parameters of the walk neuroal network. */
    (FrequencyParameters) frequencyParameters,            /**< Frequency parameters. */
    (BallNetworkParameters) ballNetworkParameters,        /**< Ball parameters for the network. */
    (BrakingModelParameters) brakingModelParameters,      /**< Empirical braking-distance model (forwarded to WalkingEngineOutput). */
    (CommandRampParameters) commandRampParameters,        /**< Motion-level velocity-command ramping (rate limiting). */
    (ArmRecoveryParameters) armRecoveryParameters,        /**< Arm recovery interpolation triggered when arms deviate substantially. */
    (TiltRecoveryParameters) tiltRecoveryParameters,      /**< Tilt-triggered recovery commands while (nearly) standing. */
  }),
});

class RERLWalkingEngine : public RERLWalkingEngineBase
{
public:
  /** Constructor */
  RERLWalkingEngine();
  const float motionCycleTime = Global::getSettings().motionCycleTime;
  RingBuffer<JointAngles, 10> lastMeasurements;
  CompiledNN network; /**< The compiled neural network. */
  unsigned networkInputSize = 0; // will be populated in ctor based on real code usage
  unsigned networkOutputSize = 0; // will be populated in ctor based on real code usage
  JointAngles offset;

  std::vector<Joints::Joint> boosterJoints;
  std::vector<Joints::Joint> boosterWaistJoints;
  std::vector<Joints::Joint> naoJoints;

  Pose2f lastRampedStep;     /**< Last ramped step target. */
  unsigned lastRampTime = 0; /**< Frame time [ms] the ramp was last advanced. */
  unsigned lastInstabilityPhaseEnd = 0; /**< Frame time [ms] a fall/getUp/freeze/playDead phase last handed control back. */

  Vector2f tiltRecoveryCmd = Vector2f::Zero(); /**< Current tilt-recovery command (m/s), valid while engaged. */

  Pose2f rampCommand(const Pose2f& current, const Pose2f& target, const float dt) const;

  /**
   * Advance the balance-recovery state machine. Engages only when active, on the ground, in an
   * allowed game state and the current command is (nearly) zero; then commands a capture step
   * "into" the imbalance so the policy places its feet back under the CoM instead of freezing at
   * zero command. Two independent triggers: base tilt (gravity projection) and CoM-over-feet
   * offset (from forward kinematics) — the latter fires even when the base is still straight.
   * @param cmdMag Norm of the current velocity command (m/s, rad/s).
   * @param gravity Gravity unit vector in the body frame (as fed to the policy).
   * @return Whether the recovery command in tiltRecoveryCmd should override the walk command.
   */
  bool updateTiltRecovery(const float cmdMag, const Vector3f& gravity);
  bool tiltRecoveryActive() const { return tiltRecoveryEngagedSince != 0; }
private:
  unsigned tiltRecoveryEngagedSince = 0; /**< Frame time [ms] the recovery engaged, 0 = inactive. */
  unsigned tiltRecoveryBlockedUntil = 0; /**< No re-engagement before this frame time [ms]. */

  float rampValue(const float current, const float target, const float maxAccelDelta, const float maxDecelDelta) const;

  void update(WalkStepData& walkStepData) override;
  void update(StandGenerator& standGenerator) override;
  void update(WalkGenerator& walkGenerator) override;
  void update(WalkingEngineOutput& walkingEngineOutput) override;

  Rangea getMaxRotationToStepFactor(const bool isFastWalk, const bool isLeftPhase, const Angle customReduceOffset, const Vector2f& stepRatio);
  Vector2f getStepSizeFactor(const Angle rotation, const bool isFastWalk, const bool isLeftPhase, const Angle customReduceOffset, const Vector2f& walkSpeedRatio);
  void filterTranslationPolygon(std::vector<Vector2f>& polygonOut, std::vector<Vector2f>& polygonIn, const std::vector<Vector2f>& polygonOriginal);
  void generateTranslationPolygon(std::vector<Vector2f>& polygon, const Vector2f& backRight, const Vector2f& frontLeft, const bool useMaxPossibleStepSize);

  /**
   * Compile the model.
   * @param output Whether to output information.
   */
  void compile(bool output);

  std::vector<Vector2f> translationPolygon; /**< The polygon that defines the max allowed translation for the step size. */

  const std::string modelPath = std::string(File::getBHDir()) + "/Config/NeuralNets/BoosterWalk/";
  IntervalTriggerMs logInterval;
};

// ==========================================================================
struct DummyPhase : MotionPhase
{
  using MotionPhase::MotionPhase;

  bool isDone(const MotionRequest&) const override { return true; }
  void calcJoints(const MotionRequest&, JointRequest&, Pose2f&, MotionInfo&) override {}
};

// ==========================================================================

struct RERLWalkPhaseDummy : MotionPhase
{
  RERLWalkPhaseDummy(RERLWalkingEngine& engine, Pose2f stepTarget, const MotionPhase& lastPhase, const bool isDelay = false,
              const WalkGenerator::CreateNextPhaseCallback& createNextPhaseCallback = WalkGenerator::CreateNextPhaseCallback(),
              const WalkKickStep& walkKickStep = WalkKickStep());

public:
  bool isLeftPhase = false;
  Pose2f step;
  WalkKickStep walkKickStep;

  float tBase = 0.f;
  float tWalk = 0.f;
  float frequency = 0.f;
  float rawFrequency = 0.f;
  bool standing = false; ///< command below zeroCmdThreshold -> stand (phase obs zeroed, gait clock frozen)
private:
  bool isDone(const MotionRequest& motionRequest) const override;
  void calcJoints(const MotionRequest& motionRequest, JointRequest& jointRequest, Pose2f& odometryOffset, MotionInfo& motionInfo) override;
  std::unique_ptr<MotionPhase> createNextPhase(const MotionPhase& defaultNextPhase) const override;
  void update() override;

  void getNextTargetRequest(JointAngles& target);
  std::vector<Joints::Joint> getBoosterLegJointSequence();

  friend class WalkingEngine;

  RERLWalkingEngine& engine; /**< A reference to the running motion engine. */
  unsigned lastModelRequest = 0;
  JointAngles nextTarget;
  JointAngles lastTarget;
  // JointAngles requestAtStandStart;
  JointAngles standTarget;
  WalkGenerator::CreateNextPhaseCallback callBack;
  IntervalTriggerMs logInterval;

  unsigned lastTimeWalking = 0; ///< the last time we were actually walking
  unsigned slowingDownToStandStartTime = 0;

  // Interpolation to our stand from something else (e.g. Booster PREP mode)
  unsigned interpStartTime = 0; ///< zero means no interpolation active, otherwise time when interpolation started
  unsigned interpDurationMs = 0; ///< how long to interpolate for
  JointAngles interpStartAngles; ///< the starting joint values that we'll blend from until eventually 100% model output

  // Arm recovery: two-phase interpolation triggered when arms are far from walking position
  unsigned armRecoveryStartTime = 0; ///< zero = inactive; frame time start for arm interpolation
  float armRecoveryElbowYawDuration = 0.f; ///< ms for Phase 1 (elbow yaw only)
  float armRecoveryArmDuration = 0.f; ///< ms for Phase 2 (all other arm joints)
  JointAngles armRecoveryStartAngles; ///< arm joint values captured at recovery start

protected:
  unsigned freeLimbs() const override
  {
    return bit(MotionPhase::head);
  }
};

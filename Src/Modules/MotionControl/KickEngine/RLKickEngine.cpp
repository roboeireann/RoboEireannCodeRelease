/**
 * @file RLKickEngine.cpp
 *
 * RL-policy implementation of KickGenerator for the K1 robot.
 *
 * Observation vector (51 dims, must match training in REIsaacLab):
 *   [0:3]   projected_gravity        body-frame gravity unit vector
 *   [3:6]   base_ang_vel             gyro x/y/z (rad/s)
 *   [6:18]  dof_pos - default        12 leg joints (rad)
 *   [18:30] dof_vel * 0.1            12 leg joints (scaled rad/s)
 *   [30:42] last_action              previous raw network output (zero on first inference)
 *   [42:44] ball_position_xy         relative to robot (m), live estimate
 *   [44:46] ball_velocity_xy         relative to robot (m/s), live estimate
 *   [46:48] kick_direction           (cos, sin) of motionRequest.targetDirection
 *   [48]    kick_length_normalized   kickLength / maxKickLength, clipped to [0,1]
 *   [49]    phase_progress           executedTime / maxKickTime, clipped to [0,1]
 *   [50]    kick_foot                +1 left, -1 right
 *
 * NOTE: Don't include arms in kick policy! 
 *
 * @author James Petri
 */

#include "RLKickEngine.h"
#include "Debugging/Annotation.h"
#include "Debugging/Stopwatch.h"
#include <filesystem>

MAKE_MODULE(RLKickEngine);

RLKickEngine::RLKickEngine()
{
  compile();

  offset.angles[Joints::lHipPitch] = -0.2f;
  offset.angles[Joints::lKneePitch] = 0.4f;
  offset.angles[Joints::lAnklePitch] = -0.25f;
  offset.angles[Joints::rHipPitch] = -0.2f;
  offset.angles[Joints::rKneePitch] = 0.4f;
  offset.angles[Joints::rAnklePitch] = -0.25f;

  // 12-joint ordering matches K1_locomotion.urdf used in IsaacLab training.

  boosterLegJoints = { 
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

}

void RLKickEngine::compile()
{
  policies.clear();

  for(const RLKickPolicyConfig& kickPolicy : kickPolicies)
  {
    if(!std::filesystem::exists(modelPath + kickPolicy.policyName))
    {
      OUTPUT_ERROR("RLKickEngine: model file " << modelPath << kickPolicy.policyName << " does not exist; "
                   << TypeRegistry::getEnumName(kickPolicy.kickMotionType) << " kicks disabled.");
      continue;
    }

    auto compiled = std::make_unique<CompiledNN>(&Global::getAsmjitRuntime());
    compiled->compile(Model(modelPath + kickPolicy.policyName));
    if(!compiled->valid())
    {
      OUTPUT_ERROR("RLKickEngine: failed to compile " << kickPolicy.policyName << "; "
                   << TypeRegistry::getEnumName(kickPolicy.kickMotionType) << " kicks disabled.");
      continue;
    }

    ASSERT(compiled->numOfInputs() == 1);
    ASSERT(compiled->input(0).rank() == 1);
    ASSERT(compiled->input(0).dims(0) == numInput);

    ASSERT(compiled->numOfOutputs() == 1);
    ASSERT(compiled->output(0).rank() == 1);
    ASSERT(compiled->output(0).dims(0) == numOutput);

    policies[kickPolicy.kickMotionType] = std::move(compiled);
  }
}

void RLKickEngine::update(KickGenerator& kickGenerator)
{
  // Recompute joint velocities at module rate; fall back to hardware-reported velocity on large gaps.
  bool calcVelocity = true;
  if(lastFrameInfo == 0 || lastFrameInfo > theFrameInfo.time)
  {
    lastFrameInfo = theFrameInfo.time;
    calcVelocity = false;
  }
  const float numberFrames = std::max(1.f, std::floor((theFrameInfo.time - lastFrameInfo) / 2.f));
  if(numberFrames > 2.5f)
    calcVelocity = false;
  FOREACH_ENUM(Joints::Joint, joint)
  {
    lastMeasurement.velocity[joint] = !calcVelocity
      ? static_cast<float>(theJointAngles.velocity[joint])
      : (theJointAngles.angles[joint] - lastMeasurement.angles[joint]) * 500.f / numberFrames;
  }
  lastFrameInfo = theFrameInfo.time;
  lastMeasurement.angles = theJointAngles.angles;

  kickGenerator.createPhase = [this](const KickRequest& kickRequest, const MotionPhase&) -> std::unique_ptr<MotionPhase>
  {
    const auto it = policies.find(kickRequest.kickMotionType);
    if(it == policies.end() || !it->second->valid())
    {
      ANNOTATION("RLKickEngine", "No RL policy loaded for kick motion type " << TypeRegistry::getEnumName(kickRequest.kickMotionType) << ".");
      return std::unique_ptr<MotionPhase>();
    }

    // Snapshot the kick command; the ball estimate is read live from theMotionRequest each inference.
    const Angle targetDirection = theMotionRequest.targetDirection;
    const float kickLength = theMotionRequest.kickLength;
    const bool isLeftKick = theKickInfo[theMotionRequest.kickType].kickLeg == Legs::left;

    return std::make_unique<RLKickPhase>(*this, *it->second, kickRequest, targetDirection, kickLength, isLeftKick);
  };

  kickGenerator.wasLeftPhase = [](const MotionPhase& lastPhase) -> bool
  {
    if(lastPhase.type == MotionPhase::kick)
      return static_cast<const RLKickPhase&>(lastPhase).isLeftKick;
    return false;
  };
}

// ---------------------------------------------------------------------------
// RLKickPhase
// ---------------------------------------------------------------------------

RLKickPhase::RLKickPhase(RLKickEngine& engine,
                         CompiledNN& policy,
                         const KickRequest& /*kickRequest*/,
                         const Angle targetDirection,
                         const float kickLength,
                         const bool isLeftKick) :
  MotionPhase(MotionPhase::kick),
  isLeftKick(isLeftKick),
  engine(engine),
  policy(policy),
  startTime(engine.theFrameInfo.time),
  targetDirection(targetDirection),
  kickLength(kickLength),
  requestOdometry(engine.theMotionRequest.odometryData),
  lastSupportSole(supportSolePose())
{
  // Training zeroes the last_action observation at episode reset.
  lastAction.fill(0.f);

  // Seed with current measurements so the first leg target is a smooth handoff from the previous phase.
  nextRequest.angles = engine.theJointAngles.angles;
  nextRequest.stiffnessData.stiffnesses.fill(100);
}

Pose2f RLKickPhase::supportSolePose() const
{
  const Pose3f sole3D = engine.theTorsoMatrix * (isLeftKick ? engine.theRobotModel.soleRight : engine.theRobotModel.soleLeft);
  return Pose2f(sole3D.rotation.getZAngle(), sole3D.translation.head<2>());
}

void RLKickPhase::executePolicy(JointRequest& target)
{
  // Policy trained at 50 Hz - throttle inference to ~20 ms (always run the first frame).
  if(lastInference != 0 && engine.theFrameInfo.getTimeSince(lastInference) < 20)
    return;

  JointAngles clippedJointAngles = engine.theJointAngles;
  FOREACH_ENUM(Joints::Joint, joint)
    clippedJointAngles.angles[joint] = engine.theJointLimits.limits[joint].limit(clippedJointAngles.angles[joint]);

  float* input = policy.input(0).data();
  const Vector3f gravity = engine.theInertialData.orientation3D.inverse() * Vector3f(0.f, 0.f, -1.f);

  // [0:3] projected gravity
  *input++ = gravity.x();
  *input++ = gravity.y();
  *input++ = gravity.z();

  // [3:6] base angular velocity
  *input++ = engine.theInertialData.gyro.x();
  *input++ = engine.theInertialData.gyro.y();
  *input++ = engine.theInertialData.gyro.z();

  // [6:18] dof position delta
  for(Joints::Joint j : engine.boosterLegJoints)
    *input++ = clippedJointAngles.angles[j] - engine.offset.angles[j];

  // [18:30] dof velocity (scaled by 0.1 to match training normalization)
  for(Joints::Joint j : engine.boosterLegJoints)
    *input++ = engine.lastMeasurement.velocity[j] * 0.1f;

  // [30:42] last action
  for(const float a : lastAction)
    *input++ = a;

  // [42:46] live ball estimate
  const Pose2f ballToCurrent = engine.theOdometryDataPreview.inverse() * engine.theMotionRequest.odometryData;
  const Vector2f ballInRobot = ballToCurrent * engine.theMotionRequest.ballEstimate.position;
  const Vector2f ballVelInRobot = Pose2f(ballToCurrent.rotation) * engine.theMotionRequest.ballEstimate.velocity;

  // [42:44] ball position in robot frame (m)
  *input++ = ballInRobot.x() * 0.001f;
  *input++ = ballInRobot.y() * 0.001f;

  // [44:46] ball velocity in robot frame (m/s)
  *input++ = ballVelInRobot.x() * 0.001f;
  *input++ = ballVelInRobot.y() * 0.001f;

  // [46:48] kick direction
  const Angle currentTargetDirection = Angle::normalize(
    targetDirection + requestOdometry.rotation - engine.theOdometryDataPreview.rotation);
  *input++ = std::cos(static_cast<float>(currentTargetDirection));
  *input++ = std::sin(static_cast<float>(currentTargetDirection));

  // [48] kick length normalized
  *input++ = Rangef::ZeroOneRange().limit(kickLength / engine.maxKickLength);

  // [49] phase progress
  *input++ = Rangef::ZeroOneRange().limit(executedTime / engine.maxKickTime);

  // [50] kick foot indicator
  *input++ = isLeftKick ? 1.f : -1.f;

  STOPWATCH("module:RLKickEngine:apply")
    policy.apply();

  // Isaac Lab applies the offset first and clips the result (processed = raw + default, then clip).
  const float* output = policy.output(0).data();
  for(std::size_t i = 0; i < engine.boosterLegJoints.size(); ++i)
  {
    lastAction[i] = *output++;
    target.angles[engine.boosterLegJoints[i]] = engine.clipActions.limit(lastAction[i] + engine.offset.angles[engine.boosterLegJoints[i]]);
  }

  lastInference = engine.theFrameInfo.time;
}

void RLKickPhase::update()
{
  executedTime = engine.theFrameInfo.getTimeSince(startTime) * engine.speedFactor;

  if(executedTime >= engine.maxKickTime + engine.postKickSettleTime)
    done = true;

  // once the ball is clearly moving it has been kicked
  if(executedTime >= engine.minKickTime &&
     engine.theMotionRequest.ballEstimate.velocity.norm() >= engine.finishBallSpeed)
    done = true;

  // if the torso has tipped past the configured "cone"
  if(std::abs(engine.theInertialData.angle.x()) > engine.abortTorsoTilt ||
     std::abs(engine.theInertialData.angle.y()) > engine.abortTorsoTilt)
    done = true;
}

bool RLKickPhase::isDone(const MotionRequest& request) const
{
  return done || request.motion == MotionRequest::playDead;
}

void RLKickPhase::calcJoints(const MotionRequest&, JointRequest& jointRequest, Pose2f& odometryOffset, MotionInfo& motionInfo)
{
  executePolicy(nextRequest);

  jointRequest = engine.theJointRequest;
  for(Joints::Joint j : engine.boosterLegJoints)
  {
    jointRequest.angles[j] = nextRequest.angles[j];
    jointRequest.stiffnessData.stiffnesses[j] = nextRequest.stiffnessData.stiffnesses[j];
  }

  motionInfo.isMotionStable = false;
  motionInfo.isWalkPhaseInWalkKick = false;
  motionInfo.speed = Pose2f();

  const Pose2f supportSole = supportSolePose();
  odometryOffset = lastSupportSole * supportSole.inverse();
  lastSupportSole = supportSole;
}

unsigned RLKickPhase::freeLimbs() const
{
  return bit(MotionPhase::head) | bit(MotionPhase::leftArm) | bit(MotionPhase::rightArm);
}

std::unique_ptr<MotionPhase> RLKickPhase::createNextPhase(const MotionPhase& nextPhase) const
{
  if(nextPhase.type == MotionPhase::playDead)
    return std::unique_ptr<MotionPhase>();
  // Hand off to a zero-target walk step so the walking engine stabilizes the robot after the kick.
  return engine.theWalkGenerator.createPhase(Pose2f(0.f, 0.01f, 0.f), *this, 0.f);
}

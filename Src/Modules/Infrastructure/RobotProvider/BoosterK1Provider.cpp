/**
 * @file BoosterK1Provider.cpp
 *
 * This file implements a module that interfaces to the Booster robot via
 * various SDK functions.
 *
 * On startup we start in booster damping mode (unstiff) and as if we had just
 * performed an emergency stop (e-stop). The human operator needs to go through
 * the state machine below to make the robot operational.
 *
 * Mode, PlayerState (GameState)      | button     | new mode, playerState (GameState)
 * ------------------------------------------------------------------------------------
 * emergency stop (e-stop) handling
 * ------------------------------------------------------------------------------------
 * custom, active (any)               | f1         | damping, unstiff
 * damping, unstiff                   | f1 long    | custom, unstiff
 * damping, unstiff                   | stand long | custom, active (initial)
 * ------------------------------------------------------------------------------------
 * Normal operations
 * ------------------------------------------------------------------------------------
 * custom, active (initial)           | walk       | custom, active (penalized)
 * custom, active (penalized)         | walk       | custom, active (playing)
 * custom, active (playing)           | walk       | custom, active (penalized)
 * custom, active (playing/penalized) | stand      | custom, active (initial)
 *
 * If the F1 key is pressed in any mode with motors operational, it acts as an emergency
 * stop and puts the robot into damping mode (motors disabled).
 *
 * In playDead/unstiff, the damping mode is commanded. In any other case,
 * the custom mode is generally used. 
 *
 * While waiting for a connection to the
 * robot, the custom mode is requested as well, because the robot
 * seems to accept the damping mode when it is not completely ready.
 *
 * NOTE: All of the handleXxxMsg functions are Booster SDK subscriber callbacks.
 *       These do not run in the motion thread, and synchronisation between
 *       threads is required for any data shared between the callbacks and the
 *       motion thread.
 *
 * @author Thomas Röfer
 * @author Rudi Villing (extensive modifications)
 */

#include "BoosterK1Provider.h"
#include "Framework/Settings.h"
#include "Platform/File.h"
#include "Platform/SystemCall.h"
#include "Platform/Thread.h"
#include "Platform/Time.h"
#include "Debugging/Annotation.h"
#include "TextLogging/TextLogging.h"
#ifdef TARGET_BOOSTER
#include <fcntl.h>
#include <linux/input.h>
#include <unistd.h>
#endif

MAKE_MODULE(BoosterK1Provider);

DECL_TLOGGER(tlogger, "BoosterK1Provider", TextLogging::DEBUG);

// convenience shorthand for annotation in a callback (Booster SDK thread)
#define ANNOTATION_K1_CB(...)                                                                                          \
  do                                                                                                                   \
  {                                                                                                                    \
    SYNC;                                                                                                              \
    pendingAnnotations.push_back(fmt::format(__VA_ARGS__));                                                            \
  } while (0)

// convenience shorthand for annotation in the motion thread
#define ANNOTATION_K1(...) ANNOTATION_FMT("BoosterK1Provider", __VA_ARGS__)


namespace
{
// inline bool isOK(int32_t boosterResultCode) { return boosterResultCode == 0; }

#ifdef TARGET_BOOSTER

constexpr unsigned LOG_INTERVAL_LONG = 30000u;
constexpr unsigned LOG_INTERVAL_MEDIUM = 15000u;


const char *robotModeString(booster::robot::RobotMode mode)
{
  switch (mode)
  {
  case booster::robot::RobotMode::kUnknown:
  default:
    return "Unknown";
  case booster::robot::RobotMode::kDamping:
    return "DAMPing";
  case booster::robot::RobotMode::kPrepare:
    return "PREP";
  case booster::robot::RobotMode::kWalking:
    return "WALK";
  case booster::robot::RobotMode::kCustom:
    return "CUSTOM";
  case booster::robot::RobotMode::kSoccer:
    return "SOCCER";
  }
}
#endif

} // namespace



#ifndef TARGET_BOOSTER
thread_local
#endif
    BoosterK1Provider *BoosterK1Provider::theInstance = nullptr;

#ifdef TARGET_BOOSTER
BoosterK1Provider::BoosterK1Provider()
    : lowCmdPublisher(booster::robot::b1::kTopicJointCtrl),
      lowStateSubscriber(booster::robot::b1::kTopicLowState,
                         makeHandler<booster_interface::msg::LowState, &BoosterK1Provider::handleLowStateMsg>()),
      robotStatusSubscriber(
          "rt/device_gateway",
          makeHandler<booster_interface::msg::RobotStatusDdsMsg, &BoosterK1Provider::handleRobotStatusMsg>()),
      buttonEventSubscriber(
          // "rt/button_re",
          "rt/button_re",
          makeHandler<booster_interface::msg::ButtonEventMsg, &BoosterK1Provider::handleButtonEventMsg>()),
      remoteControllerSubscriber(
          // "rt/remote_controller_re",
          "rt/remote_controller_re",
          makeHandler<booster_interface::msg::RemoteControllerState, &BoosterK1Provider::handleRemoteControllerMsg>()),
      robotStatesSubscriber(
          "rt/robot_states",
          makeHandler<booster_interface::msg::RobotStatesMsg, &BoosterK1Provider::handleRobotStatesMsg>())
{
  theInstance = this;
  client.Init();

  lightControlClient.Init();

  lowCmdPublisher.InitChannel();

  lowCmd.cmd_type(booster_interface::msg::SERIAL);
  lowCmd.motor_cmd(std::vector<booster_interface::msg::MotorCmd>(jointMapping.size()));

  jointDisconnectionTime.resize(jointMapping.size(), 0);

  connectedJoints = 0; // Initially joint data is not available - was previously lowCmd.motor_cmd().size();

  Thread::sleep(10);


  // TODO - decide if we want to use this as a gate for changing mode
  booster::robot::b1::GetModeResponse getModeResponse;
  if (client.GetMode(getModeResponse) == 0)
    robotMode = getModeResponse.mode_; // intentionally unsync'd - no callbacks to contend for reads/writes yet
  else
  {
    TLOGW(tlogger, "Ctor: unable to get robot mode during initialisation");
    // robotMode stays at kUnknown
  }

  // only register the subscribers at this point so that we don't need any SYNC
  // in the ctor itself.
  lowStateSubscriber.InitChannel();
  robotStatusSubscriber.InitChannel();
  buttonEventSubscriber.InitChannel();
  remoteControllerSubscriber.InitChannel();
  robotStatesSubscriber.InitChannel();
}
#else
BoosterK1Provider::BoosterK1Provider()
{
  theInstance = this;
}
#endif

BoosterK1Provider::~BoosterK1Provider()
{
#ifdef TARGET_BOOSTER
  buttonEventSubscriber.CloseChannel();
  remoteControllerSubscriber.CloseChannel();
  robotStatesSubscriber.CloseChannel();

  robotStatusSubscriber.CloseChannel();
  lowStateSubscriber.CloseChannel();
  lowCmdPublisher.CloseChannel();
#endif
  theInstance = nullptr;
}

void BoosterK1Provider::update([[maybe_unused]] KeyStates& theKeyStates)
{
#ifdef TARGET_BOOSTER
  theKeyStates.pressed[KeyStates::k1_f1] = buttonPressed[button_f1];
  theKeyStates.pressed[KeyStates::k1_stand] = buttonPressed[button_stand];
  theKeyStates.pressed[KeyStates::k1_walk] = buttonPressed[button_walk];
#endif
}

void BoosterK1Provider::update(GamePad &theGamePad)
{
  // bool changed = (gamePad.hat != theGamePad.hat || gamePad.buttons != theGamePad.buttons);

  theGamePad = gamePad;

  // if (changed) 
  //   TLOGI(tlogger, "GamePad changed: hat={} buttons={:#014b}", TypeRegistry::getEnumName(theGamePad.hat), theGamePad.buttons);
}

void BoosterK1Provider::update(RobotSafetyState &theRobotSafetyState)
{
  RobotSafetyState::State stateLastCycle = theRobotSafetyState.state;

#ifdef TARGET_BOOSTER
  theRobotSafetyState.state = robotSafetyState.load(); // update the state as needed
  {
    SYNC;
    theRobotSafetyState.hardwareConnected = hardwareDataAvailable;
  }
#else
  theRobotSafetyState.state = RobotSafetyState::operational;
  theRobotSafetyState.hardwareConnected = true;
#endif

  if (theRobotSafetyState.state != stateLastCycle)
    theRobotSafetyState.timeWhenStateStarted = theFrameInfo.time;
}



#ifdef TARGET_BOOSTER
void BoosterK1Provider::handleLowStateMsg(const booster_interface::msg::LowState& lowState)
{
  // track any timing issues with this callback.
  lowStateIntervalStats.stop(); // new cycle
  if (lowStateIntervalStats.sum() >= 15000u)
  {
    TLOGI(tlogger, "handleLowStateMsg: interval stats: {}", lowStateIntervalStats);
    lowStateIntervalStats.reset();
  }
  lowStateIntervalStats.start();

  ASSERT(lowState.motor_state_serial().size() == jointMapping.size()); // TODO check whether this works for K1

  {
    SYNC;

    // these joints are not implemented on the K1, so set to zeros
    for (size_t joint : {Joints::lWristYaw, Joints::lHand, Joints::rWristYaw, Joints::rHand})
    {
      jointSensorData.angles[joint] = 0_deg;
      jointSensorData.currents[joint] = 0;
      jointSensorData.temperatures[joint] = 0;
      jointSensorData.status[joint] = JointSensorData::regular;
    }

    // now fill in the K1 joint values...
    for (size_t joint = 0; joint < lowState.motor_state_serial().size(); ++joint)
    {
      const size_t index = jointMapping[joint];
      ASSERT(index != Joints::numJoint);
      jointSensorData.angles[index] = lowState.motor_state_serial()[joint].q();
      jointSensorData.velocity[index] = lowState.motor_state_serial()[joint].dq();
      jointSensorData.currents[index] = static_cast<short>(lowState.motor_state_parallel()[joint].tau_est() *
                                                           100.f); // This is torque (1/100 Nm), not current
      const unsigned char temperature = lowState.motor_state_parallel()[joint].temperature();
      jointSensorData.temperatures[index] = temperature;
      jointSensorData.status[index] = temperature < minHot             ? JointSensorData::regular
                                      : temperature < minVeryHot       ? JointSensorData::hot
                                      : temperature < minCriticallyHot ? JointSensorData::veryHot
                                                                       : JointSensorData::criticallyHot;
    }

    rawInertialSensorData.gyro = {lowState.imu_state().gyro()[0], lowState.imu_state().gyro()[1],
                                  lowState.imu_state().gyro()[2]};
    rawInertialSensorData.acc = {lowState.imu_state().acc()[0], lowState.imu_state().acc()[1],
                                 lowState.imu_state().acc()[2]};
    rawInertialSensorData.angle = {lowState.imu_state().rpy()[0], lowState.imu_state().rpy()[1],
                                   lowState.imu_state().rpy()[2]};

    jointSensorData.timestamp = Time::getRealSystemTime();

    // let motion thread know that we have new data
    frameDataSignal.post();
  }

  if (logIntervalLowStateRx.updateTriggered(jointSensorData.timestamp, LOG_INTERVAL_LONG))
    TLOGI(tlogger, "handleLowStateMsg - received low state ({}/{} joints and inertial data)",
          lowState.motor_state_serial().size(), jointMapping.size());

  // armPositionsOK, armPositionWarningTime are only used in this function
  // we are only reading the jointSensorData which is only written in this function
  // hence no SYNC needed for this bit
  if(!armPositionsOK)
  {
    if (!shoulderPitchRangeAtStart.isInside(jointSensorData.angles[Joints::lShoulderPitch]) ||
        !shoulderPitchRangeAtStart.isInside(jointSensorData.angles[Joints::rShoulderPitch]))
    {
      if (theFrameInfo.getTimeSince(armPositionWarningTime) > armsWarningTime)
      {
        armPositionWarningTime = theFrameInfo.time;
        SystemCall::playSound("siren", true);
        // SystemCall::say("Wrong Arm Position.");
        if(!shoulderPitchRangeAtStart.isInside(jointSensorData.angles[Joints::lShoulderPitch]))
          SystemCall::say("Wrong Arm Position, Left Arm");
        if(!shoulderPitchRangeAtStart.isInside(jointSensorData.angles[Joints::rShoulderPitch]))
          SystemCall::say("Wrong Arm Position, Right Arm");
      }
    }
    else
    {
      armPositionsOK = true;
      if (armPositionWarningTime != 0)
        SystemCall::say("Arms are correct");
    }
  }
}

void BoosterK1Provider::handleRobotStatusMsg(const booster_interface::msg::RobotStatusDdsMsg& robotStatus)
{
  // track any timing issues with this callback.
  robotStatusIntervalStats.stop(); // new cycle
  if (robotStatusIntervalStats.sum() >= 15000u)
  {
    TLOGI(tlogger, "handleRobotStatusMsg: interval stats: {}", robotStatusIntervalStats);
    robotStatusIntervalStats.reset();
  }
  robotStatusIntervalStats.start();

  const std::vector<booster_interface::msg::RobotDdsBatteryStatus> &batteryStatuses = robotStatus.battery_vec();
  float stateOfCharge = -1.f;
  bool reportStateOfCharge = false;
  size_t numConnected = 0;
  bool connectedJointsChange = false;

  if(!batteryStatuses.empty())
    stateOfCharge = batteryStatuses.front().soc() * 0.01f;

  {
    SYNC;
    if (stateOfCharge >= 0.f)
    {

      reportStateOfCharge = (std::abs(lastReportedStateOfCharge - stateOfCharge) > 0.01);
      if (reportStateOfCharge)
        lastReportedStateOfCharge = stateOfCharge;
      
      systemSensorData.batteryLevel = stateOfCharge;
      systemSensorData.batteryCharging = false;
    }

    // count the number of connected joints
    lastConnectedJoints = connectedJoints;
    connectedJoints = 0;
    for (const booster_interface::msg::RobotDdsJointStatus &joint : robotStatus.joint_vec())
      if (joint.is_connected())
        ++connectedJoints;

    numConnected = connectedJoints;
    connectedJointsChange = (connectedJoints != lastConnectedJoints);
  }


  // only read/written within this function, so outside SYNC
  unsigned timeNow = Time::getCurrentSystemTime(); // don't use frameTime because we're in a booster thread here

  if ((connectedJointsChange) && tlogger.isEnabled(TextLogging::WARNING))
  {
    ANNOTATION_K1_CB("changed num of connected joints, now {}", numConnected);

    size_t numDisconnected = jointMapping.size() - numConnected;;
    if (numDisconnected < 5)
    {
      int i = 0;
      for (const booster_interface::msg::RobotDdsJointStatus &joint : robotStatus.joint_vec())
      {
        if (joint.is_connected())
        {
          if (jointDisconnectionTime[i])
          {
            // Was disconnected, now reconnected
            // fmt::format_to(std::back_inserter(reconnectedJointsBuf), "{}{}({})", reconnectedOut.size() ? ", " : "",
            //                TypeRegistry::getEnumName(jointMapping[i]), timeNow - jointDisconnectionTime[i]);
            TLOGI(tlogger, "joint reconnected: {} (after {} ms)", TypeRegistry::getEnumName(jointMapping[i]),
                  timeNow - jointDisconnectionTime[i]);
            jointDisconnectionTime[i] = 0;
          }
        }
        else if (!jointDisconnectionTime[i])
        {
          // Newly disconnected
          jointDisconnectionTime[i] = timeNow;
          TLOGW(tlogger, "joint disconnected: {}", TypeRegistry::getEnumName(jointMapping[i]));
        }
        ++i;
      }
    }
    else
    {
      TLOGI(tlogger, "changed num of connected joints, {} disconnected now", numDisconnected);
    }
  }

  if (reportStateOfCharge)
    TLOGI(tlogger, "Battery stateOfCharge={:.2f}", stateOfCharge);

  // if (logIntervalRobotStatus.updateTriggered(timeNow, 20000u))
  //   TLOGI(tlogger, "handleRobotStatusMsg: connectedJoints={}", tmpConnectedJoints);
}




void BoosterK1Provider::triggerSafetyTrip(const char *reason, bool fromCallback)
{
  // we set tripped regardless of whether we successfully change mode or not.
  // This will later force us to suppress motor actuations if needed.
  // Fully lock-free. Safe from any thread even if motion thread is blocked or holds SYNC.
  robotSafetyState.store(RobotSafetyState::safetyTripped);

  int32_t res = client.ChangeMode(booster::robot::RobotMode::kDamping);

  // robotMode intentionally NOT updated here — no SYNC needed on this path.
  // If SDK call succeeded: robot is physically in kDamping; robotMode corrected next time
  //   the motion thread runs the safetyTripped check in waitForFrameData2.
  // If SDK call failed: robotMode stays at previous value, causing safetyTripped check
  //   to reapply the change.
  
  if (res == 0)
    TLOGW(tlogger, "******* {} ==> safetyTripped, entered DAMPing mode", reason);
  else
    TLOGE(tlogger, "******* {} ==> safetyTripped but FAILED ({}) to enter DAMPing mode", reason, res);

  if (fromCallback)
    ANNOTATION_K1_CB("safetyTripped: {}", reason);
  else
    ANNOTATION_K1("safetyTripped: {}", reason);
}



void BoosterK1Provider::handleButtonEventMsg(const booster_interface::msg::ButtonEventMsg &buttonEvent)
{
  const static std::array buttonNames = {"F1", "Stand", "Walk"};
  const static std::array eventNames = {"PRESS_DOWN",   "PRESS_UP",         "SINGLE_CLICK",    "DOUBLE_CLICK",
                                        "TRIPLE_CLICK", "LONG_PRESS_START", "LONG_PRESS_HOLD", "LONG_PRESS_END"};

  ASSERT(static_cast<size_t>(buttonEvent.button()) < buttonNames.size());
  ASSERT(static_cast<size_t>(buttonEvent.event()) < eventNames.size());

  TextLogging::Level level =
      (buttonEvent.event() == booster_interface::msg::LONG_PRESS_HOLD) ? TextLogging::VERBOSE : TextLogging::DEBUG;
  TLOG_IMPL(tlogger, level, "button {} ({}), event {} ({})", buttonEvent.button(), buttonNames[buttonEvent.button()],
            static_cast<uint32_t>(buttonEvent.event()), eventNames[buttonEvent.event()]);

  unsigned timeNow = Time::getCurrentSystemTime();

  // -------------------------------------------------------------------------
  // top level safety handling
  // -------------------------------------------------------------------------

  // The following is written defensively.
  // 
  // The transition to safetyTripped is lock free. We make a best effort to change
  // the robot mode immediately and then SYNC to mark it as such (all in triggerSafetyTrip).
  //
  // We don't allow transitions out of safetyTripped unless the hardware data is
  // available (lowState updates) and robot mode is already DAMPing.
  //
  // If already in resetPending, we confirm again that the hardware interface is
  // OK AND that the MotionRequest is safe, before transitioning to operational.
  // Otherwise we failsafe back to safetyTripped. 

  // Note: our guide says that the button should be pressed for 3 secs here, but since the handleButtonMessage
  // method is not called that frequently (approx every 500ms during a long press) we just
  // check that the button is held for something longer than 2 secs
  constexpr int resetPendingPressMs = 2000;

  switch (robotSafetyState.load())
  {
  case RobotSafetyState::operational:
    if (buttonEvent.button() == button_f1 && buttonEvent.event() == booster_interface::msg::PRESS_DOWN)
      triggerSafetyTrip("E-stop (F1) pressed", true);
    break;

  case RobotSafetyState::safetyTripped:
    if (((buttonEvent.button() == button_f1) && (timeWhenButtonPressed[button_f1] > 0) &&
         (Time::getTimeBetween(timeWhenButtonPressed[button_f1], timeNow) > resetPendingPressMs)) ||
        ((buttonEvent.button() == button_stand) && (timeWhenButtonPressed[button_stand] > 0) &&
         (Time::getTimeBetween(timeWhenButtonPressed[button_stand], timeNow) > resetPendingPressMs)))
    {
      robotSafetyState.store(RobotSafetyState::resetPending); // checks will happen in waitForFrameData2 + delegates
      TLOGI(tlogger, "**** from safetyTripped: {} long press ({} ms) detected ==> resetPending",
            buttonNames[buttonEvent.button()],
            Time::getTimeBetween(timeWhenButtonPressed[buttonEvent.button()], timeNow));
      ANNOTATION_K1_CB("safetyState: safetyTripped ==> resetPending");
    }
    break;

  case RobotSafetyState::resetPending:
    if ((buttonEvent.button() == button_f1 || buttonEvent.button() == button_stand) &&
        (buttonEvent.event() == booster_interface::msg::PRESS_UP))
    {
      RobotSafetyState::State expected = RobotSafetyState::resetPending;
      if (robotSafetyState.compare_exchange_strong(expected, RobotSafetyState::operational))
      {
        TLOGI(tlogger, "**** from resetPending: {} press released ==> operational", buttonNames[buttonEvent.button()]);
        ANNOTATION_K1_CB("safetyState: resetPending ==> operational");
      }

      // the CAS will only fail in case of a concurrent switch to safetyTripped which will already
      // be logged. Other checks also take place in waitForFrameData2 + delegates.
    }

    break;
  }

  // update button press times (not shared with the main motion thread so not SYNC'd)
  if (buttonEvent.event() == booster_interface::msg::PRESS_DOWN)
    timeWhenButtonPressed[buttonEvent.button()] = timeNow;
  else if ((buttonEvent.event() == booster_interface::msg::PRESS_UP) ||
           (buttonEvent.event() == booster_interface::msg::LONG_PRESS_END))
    timeWhenButtonPressed[buttonEvent.button()] = 0;

  // -------------------------------------------------------------------------
  // store basic button info for the KeyStates representation
  // -------------------------------------------------------------------------

  SYNC; // locking for buttonPressed changes

  if (buttonEvent.event() == booster_interface::msg::PRESS_DOWN)
    buttonPressed[buttonEvent.button()] = true;
  else if (buttonEvent.event() == booster_interface::msg::PRESS_UP)
    buttonPressed[buttonEvent.button()] = false;
}


void BoosterK1Provider::handleRemoteControllerMsg(const booster_interface::msg::RemoteControllerState &rc)
{
  // we use 3 GamePad structs: tmpGamePad (only here) allows us to convert from rc to GamePad outside of SYNC region.
  // lastGamePad is a copy taken at the end of the function (before the SYNC region) that represents the GamePad from
  // the last event. It is only used here so again is safe to use outside of SYNC so we keep text logging etc
  // outside of SYNC also. Finally gamePad is the actual struct shared with the main thread. We copy to that inside 
  // a SYNC region as the last step here. The copies are light since they are basically 4 floats, a bool, a 2 int size elements
  GamePad tmpGamePad;
  
  // translate from rc event to GamePad

  // we detected something so we must be connected, right?
  if (rc.event() == booster::robot::b1::REMOVE)
    tmpGamePad.connected = false;
  else
    tmpGamePad.connected = true;

  tmpGamePad.buttons = (GamePad::maskA * rc.a()) | (GamePad::maskB * rc.b()) | (GamePad::maskX * rc.x()) |
                    (GamePad::maskY * rc.y()) | (GamePad::maskLB * rc.lb()) | (GamePad::maskRB * rc.rb()) |
                    (GamePad::maskLT * rc.lt()) | (GamePad::maskRT * rc.rt()) | (GamePad::maskLS * rc.ls()) |
                    (GamePad::maskRS * rc.rs()) | (GamePad::maskBack * rc.back()) | (GamePad::maskStart * rc.start());

  if (rc.hat_c())
    tmpGamePad.hat = GamePad::centered;
  else if(rc.hat_u())
    tmpGamePad.hat = GamePad::up;
  else if (rc.hat_ru())
    tmpGamePad.hat = GamePad::upRight;
  else if (rc.hat_r())
    tmpGamePad.hat = GamePad::right;
  else if (rc.hat_rd())
    tmpGamePad.hat = GamePad::downRight;
  else if (rc.hat_d())
    tmpGamePad.hat = GamePad::down;
  else if (rc.hat_ld())
    tmpGamePad.hat = GamePad::downLeft;
  else if (rc.hat_l())
    tmpGamePad.hat = GamePad::left;
  else // if (rc.hat_lu())
    tmpGamePad.hat = GamePad::upLeft;

  tmpGamePad.lStickX = rc.lx();
  tmpGamePad.lStickY = -rc.ly(); // we want forward to be +1
  tmpGamePad.rStickX = rc.rx();
  tmpGamePad.rStickY = -rc.ry(); // we want forward to be +1

  // log it

  constexpr float deadzone = 0.1f;

  auto axisTransitioned = [&](float current, float previous)
  {
    const bool isAbove = std::abs(current) > deadzone;
    const bool wasAbove = std::abs(previous) > deadzone;
    return isAbove != wasAbove;
  };


  if (rc.event() != 0)
  {
    const bool axisChanged = (axisTransitioned(tmpGamePad.lStickX, lastGamePad.lStickX) ||
                              axisTransitioned(tmpGamePad.lStickY, lastGamePad.lStickY) ||
                              axisTransitioned(tmpGamePad.rStickX, lastGamePad.rStickX) ||
                              axisTransitioned(tmpGamePad.rStickY, lastGamePad.rStickY));

    // remote controller input events come fast when the analogue sticks are
    // manipulated, so we need to reduce the logging output in that situation.
    // In general we try to limit log output to changes worth noting
    logIntervalRemoteController.updateTriggered(Time::getCurrentSystemTime(), LOG_INTERVAL_MEDIUM);

    bool anyButtonChange = (tmpGamePad.hat != lastGamePad.hat) || (tmpGamePad.buttons != lastGamePad.buttons);

    if (anyButtonChange || axisChanged || ((rc.event() == booster::robot::b1::AXIS) && logIntervalRemoteController.wasTriggered()))
    {
      TLOGD(tlogger,
            "remote ({:#05x}): lstk=({:.2f},{:.2f}), rstk=({:.2f},{:.2f}), hat={:d}'{:d}{:d}{:d}{:d}'{:d}{:d}{:d}{:d}, "
            "abxy={:d}{:d}{:d}{:d} lbrb={:d}{:d}, ltrt={:d}{:d} lsrs={:d}{:d} (-)/(+)={:d}{:d}",
            rc.event(), rc.lx(), rc.ly(), rc.rx(), rc.ry(), rc.hat_c(), rc.hat_u(), rc.hat_d(), rc.hat_l(), rc.hat_r(),
            rc.hat_lu(), rc.hat_ld(), rc.hat_ru(), rc.hat_rd(), rc.a(), rc.b(), rc.x(), rc.y(), rc.lb(), rc.rb(),
            rc.lt(), rc.rt(), rc.ls(), rc.rs(), rc.back(), rc.start());
    }
  }
  else
    TLOGV(tlogger, "remoteController event 0 received");

  lastGamePad = tmpGamePad;

  SYNC;

  gamePad = tmpGamePad;
}

// It seems that this message arrives with a delay of anything between 100
// and 500 ms after the robot mode was actually changed, so not very
// useful.
void BoosterK1Provider::handleRobotStatesMsg(const booster_interface::msg::RobotStatesMsg &rs)
{
  auto bodyControlString = [](int32_t bodyControl)
  {
    using enum booster::robot::BodyControl;

    switch (static_cast<BodyControl>(bodyControl))
    {
    case kUnknown: return "Unknown";
    case kDamping: return "Damping";
    case kPrepare: return "Prepare";
    case kHumanlikeGait: return "HumanlikeGait";
    case kSoccerGait: return "SoccerGait";
    case kShoot: return "Shoot";
    case kInsideFoot: return "InsideFoot";
    case kGoalie: return "Goalie";
    case kWBCGait: return "WholeBodyControlGait";
    case kCustom: return "Custom";
    case kGetUp: return "GetUp";
    default: return "Other BodyControl";
    }
  };
  
  // local tmp snapshots to take outside sync region
  int32_t lastReportedModeLocal;
  int32_t lastReportedBodyControlLocal;
  size_t lastReportedActionsSizeLocal;
  {
    SYNC;

    lastReportedModeLocal = lastReportedMode;
    lastReportedBodyControlLocal = lastReportedBodyControl;
    lastReportedActionsSizeLocal = lastReportedActionsSize;
    lastReportedMode = rs.current_mode();
    lastReportedBodyControl = rs.current_body_control();
    lastReportedActionsSize = rs.current_actions().size();
  }

  if (rs.current_mode() != lastReportedModeLocal)
    TLOGI(tlogger, "robotStates: mode has changed to {}",
          robotModeString(static_cast<booster::robot::RobotMode>(rs.current_mode())));
  if (rs.current_body_control() != lastReportedBodyControlLocal)
    TLOGI(tlogger, "robotStates: body control has changed to {}", bodyControlString(rs.current_body_control()));
  if (rs.current_actions().size() != lastReportedActionsSizeLocal)
    TLOGD(tlogger, "robotStates: current number of actions is now {}", rs.current_actions().size());
}


// threadsafe - does nothing if robot mode is already set to targetMode
bool BoosterK1Provider::changeRobotMode(booster::robot::RobotMode targetMode)
{
  // Read current mode and early-exit under SYNC, then release before SDK call
  {
    SYNC;
    if (targetMode == robotMode)
      return true;
  }

  int32_t changeResult = client.ChangeMode(targetMode);

  booster::robot::RobotMode prevRobotMode;
  bool changeModeFailure = false;
  {
    SYNC;
    prevRobotMode = robotMode;

    changeModeFailure = ((robotMode != lastRobotMode) || (targetMode != lastTargetRobotMode));

    lastRobotMode = robotMode;
    lastTargetRobotMode = targetMode;

    if (changeResult == 0)
      robotMode = targetMode;
  }

  if (changeResult == 0) // no error
  {
    TLOGI(tlogger, "changeRobotMode: from {} to {}", robotModeString(prevRobotMode), robotModeString(targetMode));
    return true;
  }
  else if (changeModeFailure)
  {
    TLOGW(tlogger, "changeRobotMode: FAILED ({}) to change mode to {}, robotMode={}", changeResult,
          robotModeString(targetMode), robotModeString(prevRobotMode));
  }

  return false;
}



void BoosterK1Provider::logRobotInfoOnce()
{
  if (!robotInfoLogged)
  {
    // booster::robot::b1::GetRobotInfoResponse robotInfo;
    // if (isOK(client.GetRobotInfo(robotInfo)))
    // {
    //   TLOGI(tlogger, "Booster robot info:");
    //   TLOGI(tlogger, "  name='{}', nickname='{}'", robotInfo.name_, robotInfo.nickname_);
    //   TLOGI(tlogger, "  version={}, model={}, serial={}, edition={}, region={}", robotInfo.version_, robotInfo.model_,
    //         robotInfo.serial_number_, robotInfo.edition_, robotInfo.region_);
    // }
    // else
      TLOGW(tlogger, "Unable to get booster robot info");

    robotInfoLogged = true;
  }
}

bool BoosterK1Provider::tryWaitForJointData()
{
  if (frameDataSignal.wait(maxDelayForFrameData))
  {
    while (frameDataSignal.tryWait())
      ; // drain any queued signals
    return true;
  }

  return false;
}


// true means connected, false is disconnected
// called from motion thread only
bool BoosterK1Provider::updateHardwareDataAvailable(bool jointDataReceived, size_t numConnectedJoints)
{
  // bool isDataAvailable = ((numConnectedJoints==jointMapping.size()) && jointDataReceived);
  bool isDataAvailable = (numConnectedJoints && jointDataReceived);
  bool wasDataAvailable;
  {
    SYNC;

    wasDataAvailable = hardwareDataAvailable;
    hardwareDataAvailable = isDataAvailable; // shared data, but only written here - can safely read anywhere in this thread
  }

  // does lack of sensor inputs indicate that the hardware disconnected?
  if (!isDataAvailable)
  {
    if (waitingForJointDataAnnouncement.once())
    {
      TLOGI(tlogger, "updateHardwareConnectedState: awaiting initial joint data");
      SystemCall::say("Waiting for initial body connection");
    }
    // if we think the robot is connected, then this is a new disconnection
    else if (wasDataAvailable)
    {
      triggerSafetyTrip("Joint data unavailable");
      SystemCall::playSound("siren", true);
      SystemCall::say("Body disconnected", true);
    }

    if (logIntervalUpdateHardwareDataAvailable.updateTriggered(theFrameInfo.time, LOG_INTERVAL_MEDIUM))
      TLOGI(tlogger, "updateHardwareDataAvailable: hardwareData is NOT available: jointDataReceived={}, numJoints={}",
            jointDataReceived, numConnectedJoints);

    if (robotSafetyState.load() == RobotSafetyState::operational)
    {
      TLOGE(tlogger, "****** Pseudo-assert failure: hardware isDataAvailable={}, wasDataAvailable={}, safetyState=operational",
            isDataAvailable, wasDataAvailable);
      triggerSafetyTrip("Joint data unavailable (SHOULD HAVE BEEN HANDLED ALREADY)");
      SystemCall::playSound("siren", true);
      SystemCall::say("Body disconnected", true);
    }
  }
  else if (!wasDataAvailable)
  {
    TLOGI(tlogger, "updateHardwareDataAvailable: hardwareData now available");
    ANNOTATION_K1("body/jointData connected");
    SystemCall::say("Body connected", true);
  }

  return isDataAvailable;
}


bool BoosterK1Provider::checkJointRequestIsSafe(booster::robot::RobotMode targetMode)
{
  {
    SYNC;

    if (targetMode == booster::robot::RobotMode::kCustom && robotMode != booster::robot::RobotMode::kCustom)
    {
      bool ret = true;

      // verify all joint request angles are "close" to current joint angles
      for (size_t joint = 0; joint < lowCmd.motor_cmd().size(); ++joint)
      {
        const size_t index = jointMapping[joint];
        ASSERT(index != Joints::numJoint);

        // FIXME: 5_deg is a guessed tolerance value to avoid jerks on transition - make it configurable?
        // FIXME: we assume that while not in kCustom, and in particular just before transitioning into
        //        it, that stiffness should be zero (if the joint is not off)
        //
        // this code allows a jointRequest.angle to be off (e.g. to disable a joint for some test)
        // but be aware that if a joint is off due to a programming error, weird and maybe
        // bad things will happen
        if (theJointRequest.angles[index] != SensorData::off)
        {
          if (std::abs(theJointRequest.angles[index] - jointSensorData.angles[index]) > 5_deg)
          {
            TLOGW(tlogger, "Joint unsafe: {} angle={:.1f}, request={:.1f}",
                  TypeRegistry::getEnumName(static_cast<Joints::Joint>(index)),
                  jointSensorData.angles[index].toDegrees(), theJointRequest.angles[index].toDegrees());
            ret = false; // not safe
          }
        }
        else if (theJointRequest.stiffnessData.stiffnesses[index] > 0)
        {
          TLOGW(tlogger, "Joint unsafe: {} request off but stiffness={}",
                TypeRegistry::getEnumName(static_cast<Joints::Joint>(index)),
                theJointRequest.stiffnessData.stiffnesses[index]);
          ret = false;
        }
      }

      return ret;
    }
  }

  return true; // safe
}

// Switches between kCustom and kDamping as the motion request demands.
// called from motion thread only
void BoosterK1Provider::updateRobotModeFromMotion()
{
  // has the motion changed?
  if (motionInfoPhase.update(theMotionInfo.executedPhase))
    TLOGI(tlogger, "updateRobotModeFromMotion: requested motion = {}",
          TypeRegistry::getEnumName(theMotionInfo.executedPhase));

  // decide the appropriate booster mode for the requested motion
  booster::robot::RobotMode targetMode = booster::robot::RobotMode::kCustom;

  // if (theMotionRequest.motion == MotionRequest::playDead)
  if ((theMotionInfo.executedPhase == MotionPhase::playDead) || (theDamageConfigurationBody.disableAllMotion))
    targetMode = booster::robot::RobotMode::kDamping;
  // else if (theMotionRequest.motion == MotionRequest::prepare)
  else if (theMotionInfo.executedPhase == MotionPhase::prepare)
    targetMode = booster::robot::RobotMode::kPrepare;

  // check if it is safe to change mode
  if (!checkJointRequestIsSafe(targetMode))
  {
    triggerSafetyTrip("Motion request unsafe for intended mode");
    SystemCall::playSound("siren");
    SystemCall::say("Motion request is unsafe.");
  }

  if (changeRobotMode(targetMode))
  {
    modeChangeFailureAnnouncement.reset();
  }
  else
  {
    triggerSafetyTrip("changeRobotMode failure");

    if (modeChangeFailureAnnouncement.once())
      SystemCall::say("Changing robot mode failed", true);
  }
}


void BoosterK1Provider::commandLightControl()
{
  static constexpr unsigned MACRO_PERIOD_MS = 1600u; // bitmask macro cycle duration
  static constexpr unsigned BIT_DURATION_MS = 50u;   // duration of each bitmask bit
  static constexpr unsigned FADE_PERIOD_MS = 2 * MACRO_PERIOD_MS;
  static constexpr unsigned HALF_FADE_MS = FADE_PERIOD_MS / 2;
  static constexpr unsigned QUARTER_FADE_MS = FADE_PERIOD_MS / 4;

  const auto &req = theLedRequestK1;
  const unsigned t = theFrameInfo.time;

  // only send a colour if it changes
  auto setColor = [this](uint8_t r, uint8_t g, uint8_t b)
  {
    const uint32_t packed = (uint32_t(r) << 16) | (uint32_t(g) << 8) | b;
    if (packed == mSentLedColor)
      return;
    lightControlClient.SetLEDLightColor(r, g, b);
    mSentLedColor = packed;
  };

  // fade using a cosine shaped curve
  auto fadeScale = [t](LedRequestK1::Mode mode, unsigned offset=0) -> float
  {
    enum Shape { RISE, ON, FALL, OFF, UNUSED };

    static constexpr Shape modeTable[][4] = {
        // P0  P1    P2      P3    (=partIndexes)
        {RISE, FALL, UNUSED, UNUSED}, // fade - half period rise/fall
        {RISE, ON,   ON,   FALL}, // fade2
        {RISE, ON,   FALL, OFF}, // fade3
        {RISE, FALL, OFF,  OFF}, // fade4
    };

    // fade uses half-period part, fade2/3/4 use quarter-period parts
    const unsigned partMs = (mode == LedRequestK1::Mode::fade) ? HALF_FADE_MS : QUARTER_FADE_MS;
    const unsigned phase = (t + (offset * partMs)) % FADE_PERIOD_MS;
    const unsigned partIndex = phase / partMs; // which part of the phase
    const float partPos = float(phase % partMs) / float(partMs); // what fraction of the part

    // WARNING: this mapping to lookup table index requires fade, fade2, fade3, fade4 to be contiguous enum values
    // If the enum values become non-contiguous it will break
    const int modeIndex = static_cast<int>(mode) - static_cast<int>(LedRequestK1::fade);
    switch (modeTable[modeIndex][partIndex])
    {
    case RISE: return 0.5f - (0.5f * std::cos(partPos * pi));
    case ON: return 1.0f;
    case FALL: return 0.5f + (0.5f * std::cos(partPos * pi));
    case OFF: return 0.0f;
    default: ASSERT(false); return 0.0f; // should not occur
    }
  };

  auto applyLayer = [&](const LedColorK1 &color, LedRequestK1::Mode mode, uint32_t mask) -> bool
  {
    if (mode == LedRequestK1::binaryPattern)
    {
      if (!((mask >> ((t % MACRO_PERIOD_MS) / BIT_DURATION_MS)) & 1u)) // is the LED off in this timeslot?
        return false;
      else
      {
        setColor(color.r, color.g, color.b);
        return true;
      }
    }
    else // fade
    {
      const float s = fadeScale(mode, mask);
      setColor(uint8_t(float(color.r) * s), uint8_t(float(color.g) * s), uint8_t(float(color.b) * s));
      return true;
    }
  };

  // foreground overrides background, so only apply background if no foreground
  // LED lighting. If no background LED, then apply 0,0,0 (LED off)
  if (!applyLayer(req.fgColor, req.fgMode, req.fgMask))
  {
    if (!applyLayer(req.bgColor, req.bgMode, req.bgMask))
      setColor(0, 0, 0);
  }
}



#endif // TARGET_BOOSTER


void BoosterK1Provider::waitForFrameData()
{
  if(theInstance)
    theInstance->waitForFrameData2();
}



void BoosterK1Provider::waitForFrameData2()
{
#ifdef TARGET_BOOSTER
  bool jointDataReceived = tryWaitForJointData();
  size_t numConnectedJoints;
  {
    SYNC;
    numConnectedJoints = connectedJoints; // thread safe read
  }

  if (logIntervalWaitForFrameData.updateTriggered(theFrameInfo.time, LOG_INTERVAL_LONG))
    TLOGI(tlogger, "waitForFrameData2: jointDataReceived={}, connectedJoints={}", jointDataReceived, numConnectedJoints);

  updateHardwareDataAvailable(jointDataReceived, numConnectedJoints);

  booster::robot::RobotMode robotModeLocal;
  {
    SYNC;
    robotModeLocal = robotMode;
  }

  // check robotSafetyState transitions here. This is where we verify that
  // the transition is really allowed and safetyTrip if it is not.
  // NOTE: if the hardware (joint) data is unavailable, updateHardwareAvailable
  // will have already safety tripped, so we don't need to check for that here
  RobotSafetyState::State currentRobotSafetyState = robotSafetyState.load();

  // safetyTripped --> resetPending transition?
  if (lastRobotSafetyState == RobotSafetyState::safetyTripped && currentRobotSafetyState == RobotSafetyState::resetPending)
  {
    if (robotModeLocal != booster::robot::RobotMode::kDamping)
    {
      TLOGW(tlogger, "safetyTripped --> resetPending FAILURE because robotMode={}", robotModeString(robotModeLocal));
      triggerSafetyTrip("robotMode not kDamping entering resetPending");
      SystemCall::playSound("bad_button.wav");
      SystemCall::say("Robot mode error.");
    }
  }
  // resetPending --> operational transition?
  else if (lastRobotSafetyState == RobotSafetyState::resetPending &&
           currentRobotSafetyState == RobotSafetyState::operational)
  {
    if (robotModeLocal != booster::robot::RobotMode::kDamping)
    {
      // Programming error: resetPending should only be reachable after confirmed kDamping.
      // However, fail safe rather than assert.
      TLOGE(tlogger, "resetPending --> operational FAILURE because robotMode={}", robotModeString(robotModeLocal));
      triggerSafetyTrip("robotMode not kDamping entering operational");
      SystemCall::playSound("siren.wav");
      SystemCall::say("Robot mode programming error.");
    }
  }

  currentRobotSafetyState = robotSafetyState.load(); // it might have changed in the transition checks above
  lastRobotSafetyState = currentRobotSafetyState; // remember the true safety state for this cycle

  // robot safety state steady state handling
  if (currentRobotSafetyState == RobotSafetyState::operational)
    updateRobotModeFromMotion(); // also checks motion request is safe
  else if (currentRobotSafetyState == RobotSafetyState::safetyTripped)
    changeRobotMode(booster::robot::RobotMode::kDamping); // internally checks if change is really required

#else
  Thread::yield();
#endif
}

void BoosterK1Provider::finishFrame()
{
  if(theInstance)
    theInstance->finishFrame2();
}

void BoosterK1Provider::finishFrame2()
{
  DEBUG_RESPONSE("module:BoosterK1Provider:segfault")* static_cast<volatile char*>(nullptr) = 0;

#ifdef TARGET_BOOSTER
  commandLightControl(); // update the LEDs

  // TMP - deliberately commented out for now  
  ASSERT(lowCmd.motor_cmd().size() == jointMapping.size());
  ASSERT(lowCmd.motor_cmd().size() == controlParameters.size());

  for(size_t joint = 0; joint < lowCmd.motor_cmd().size(); ++joint)
  {
    booster_interface::msg::MotorCmd& motorCmd = lowCmd.motor_cmd()[joint];
    const size_t index = jointMapping[joint];
    ASSERT(index != Joints::numJoint);
    if(robotMode == booster::robot::RobotMode::kUnknown
       || theJointRequest.angles[index] == SensorData::off
       || theJointRequest.stiffnessData.stiffnesses[index] <= 0)
    {
      motorCmd.q(jointSensorData.angles[index]);
      motorCmd.kp(0.f);
      motorCmd.kd(0.f);
      motorCmd.tau(0.f);
    }
    else //if (index != Joints::headPitch && index != Joints::headYaw /* && (index >= Joints::firstLegJoint) */) // FIXME: TESTING with limited joints only
    {
      motorCmd.q(theJointRequest.angles[index] + theJointCalibration.offsets[index]);
      const float stiffness = Rangef::ZeroOneRange().limit(theJointRequest.stiffnessData.stiffnesses[index] * 0.01f);
      motorCmd.kp(controlParameters[joint].p * stiffness);
      motorCmd.kd(controlParameters[joint].d * stiffness);
      motorCmd.tau(0);
    }
    // else
    // {
    //   motorCmd.q(jointSensorData.angles[index]);
    //   motorCmd.kp(0.f);
    //   motorCmd.kd(0.f);
    //   motorCmd.tau(0.f);
    // }
    motorCmd.weight(1.f);
  }

  VERIFY(lowCmdPublisher.Write(&lowCmd));


  // output any annotations created in callbacks.
  // has to be done here so that we're in the motion thread
  std::vector<std::string> pendingAnnotationsSnapshot;

  {
    SYNC;
    if (!pendingAnnotations.empty())
    {
      pendingAnnotationsSnapshot = pendingAnnotations;
      pendingAnnotations.clear();
    }
  }

  for (const auto& annotation : pendingAnnotationsSnapshot)
    ANNOTATION("BoosterK1Provider", annotation);
#endif
}

/**
 * @file BoosterProvider.h
 *
 * This file declares a module that connects to the Booster robot.
 * In playDead, the damping mode is commanded. In any other case,
 * the custom mode is used. When waiting for a connection to the
 * robot, the custom mode is requested as well, because the robot
 * seems to accept the damping mode when it is not completely ready.
 *
 * @author Thomas Röfer
 * @author Rudi Villing (modifications)
 */

#pragma once

#include "Framework/Module.h"
#include "Platform/Semaphore.h"
#include "Platform/Thread.h"
#include "Representations/Configuration/JointCalibration.h"
#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Infrastructure/ModifiedJointRequest.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/Infrastructure/SensorData/GamePad.h"
#include "Representations/Infrastructure/SensorData/RawInertialSensorData.h"
#include "Representations/Infrastructure/SensorData/SystemSensorData.h"
#include "Representations/Infrastructure/RobotSafetyState.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Infrastructure/LedRequestK1.h"
#include "Tools/RECommon/OnceFlag.h"
#include "Tools/RECommon/EdgeTrigger.h"
#include "Tools/RECommon/IntervalTrigger.h"
#include "Tools/RECommon/TimingStats.h"

#include <atomic>
#include <fmt/core.h>

#if defined TARGET_ROBOT && (defined __arm64__ || defined __aarch64__)
#define TARGET_BOOSTER
#include <booster/idl/b1/LowCmd.h>
#include <booster/idl/b1/LowState.h>
#include <booster/idl/b1/ButtonEvent.h>
#include <booster/idl/b1/RemoteControllerState.h>
#include <booster/idl/b1/RobotStates.h>
#include <booster/idl/b1/RobotStatusDdsMsg.h>
#include <booster/robot/common/robot_shared.hpp>
#include <booster/robot/b1/b1_loco_client.hpp>
#include <booster/robot/device/light/light_control_client.hpp>
#include <booster/robot/b1/b1_api_const.hpp>
#include <booster/robot/channel/channel_publisher.hpp>
#include <booster/robot/channel/channel_subscriber.hpp>
#endif

MODULE(BoosterK1Provider,
{,
  REQUIRES(FrameInfo),
  REQUIRES(JointCalibration),
  REQUIRES(DamageConfigurationBody),
  USES(JointRequest),
  USES(JointSensorData),
  USES(ModifiedJointRequest),
  REQUIRES(MotionRequest),
  USES(MotionInfo),
  REQUIRES(LedRequestK1),
  PROVIDES(FrameInfo),
  PROVIDES(JointSensorData),
  PROVIDES(KeyStates),
  PROVIDES(GamePad),
  PROVIDES(RawInertialSensorData),
  PROVIDES(SystemSensorData),
  PROVIDES(RobotSafetyState),
  LOADS_PARAMETERS(
  {
    STREAMABLE(ControlParameter,
    {,
      (float) p, /**< p factor for a joint controller. */
      (float) d, /**< d factor for a joint controller. */
      (float) maxTorque,
    }),

    (unsigned char) minHot, /**< Minimum temperature for status hot (in °C). */
    (unsigned char) minVeryHot, /**< Minimum temperature for status very hot (in °C). */
    (unsigned char) minCriticallyHot, /**< Minimum temperature for status critically hot (in °C). */
    (int) maxDelayForFrameData, /**< The maximum time waited for new frame data. */
    (std::vector<Joints::Joint>) jointMapping, /**< The mapping from Booster joints indices to B-Human joint indices. */
    (std::vector<ControlParameter>) controlParameters, /**< The controller factors for the different joints. */
    (bool) setTorque,
    (Rangea) shoulderPitchRangeAtStart, /**< At start the shoulder pitches should be insides this range. */
    (int) armsWarningTime, /**< Warn about the arm positions after this much time passed. */
  }),
});

class BoosterK1Provider : public BoosterK1ProviderBase
{
  DECLARE_SYNC; /**< Data is received in a different thread, so synchronize access. */

#ifndef TARGET_BOOSTER
  thread_local
#endif
      static BoosterK1Provider *theInstance; ///< The only instance of this module.

  JointSensorData jointSensorData; ///< The joint sensor data received.
  RawInertialSensorData rawInertialSensorData; ///< The inertial sensor data received.
  SystemSensorData systemSensorData; ///< The battery status received.
  Semaphore frameDataSignal;         /**< A signal used for synchronizing lowStateHandler() and waitForFrameData2(). */
  GamePad gamePad;                   ///< a snapshot of the remote controller state received
  GamePad lastGamePad; ///< a thread-unsafe copy of gamePad that is used ONLY in handleRemoteControllerMsg

  /// the robot safety state. (It is atomic because it participates in critical e-stop handling which we keep lock free)
  std::atomic<RobotSafetyState::State> robotSafetyState = RobotSafetyState::safetyTripped; // startup condition
  static_assert(std::atomic<RobotSafetyState::State>::is_always_lock_free,
                "robotSafetyState must be lock-free for e-stop safety guarantee");

#ifdef TARGET_BOOSTER
  // remember safetyState from last cycle - an EdgeTrigger instance doesn't work here 
  // because there can sometimes be more than one transition per cycle
  RobotSafetyState::State lastRobotSafetyState = RobotSafetyState::safetyTripped;
  
  /// the hardware API client - mainly used to change the robotMode
  booster::robot::b1::B1LocoClient client;

  /// The current client mode of the robot. kUnknown means we don't know.
  booster::robot::RobotMode robotMode = booster::robot::RobotMode::kUnknown;
  // the next two members are used to ensure the text log doesn't get spammed with repeat failures on each cycle
  booster::robot::RobotMode lastRobotMode = booster::robot::RobotMode::kUnknown;
  booster::robot::RobotMode lastTargetRobotMode = booster::robot::RobotMode::kUnknown;

  booster_interface::msg::LowCmd lowCmd;   ///< The current low level joint command

  booster::robot::ChannelPublisher<booster_interface::msg::LowCmd> lowCmdPublisher; /**< Publisher for low level joint commands. */
  booster::robot::ChannelSubscriber<booster_interface::msg::LowState> lowStateSubscriber; /**< Subscriber for low level joint measurements. */
  booster::robot::ChannelSubscriber<booster_interface::msg::RobotStatusDdsMsg> robotStatusSubscriber; /**< Subscriber for the robot status. */
  booster::robot::ChannelSubscriber<booster_interface::msg::ButtonEventMsg> buttonEventSubscriber; /**< Subscriber for button events. */
  booster::robot::ChannelSubscriber<booster_interface::msg::RemoteControllerState> remoteControllerSubscriber; /**< Subscriber for the remote controller (Gamepad). */
  booster::robot::ChannelSubscriber<booster_interface::msg::RobotStatesMsg> robotStatesSubscriber;
  booster::robot::light::LightControlClient lightControlClient; ///< client for back panel LED control

  /// essentially is the lowState subscriber getting all the data it needs
  bool hardwareDataAvailable = false; // default is disconnected initially
  size_t connectedJoints;             /**< How many joints are connected? */
  EdgeTrigger<MotionRequest::Motion> motionRequestMotion {MotionRequest::playDead};
  EdgeTrigger<MotionPhase::Type> motionInfoPhase {MotionPhase::playDead};

  IntervalTriggerMs logIntervalLowStateRx;
  IntervalTriggerMs logIntervalRobotStatus;
  IntervalTriggerMs logIntervalUpdateHardwareDataAvailable;
  IntervalTriggerMs logIntervalWaitForFrameData;
  IntervalTriggerMs logIntervalRemoteController;
  float lastReportedStateOfCharge = 1.f;

  IntervalTimingStats robotStatusIntervalStats;
  IntervalTimingStats lowStateIntervalStats;
  std::vector<unsigned> jointDisconnectionTime;

  // fmt::memory_buffer jointDisconnectedBuf;
  // fmt::memory_buffer jointReconnectedBuf;
  size_t lastConnectedJoints = 0; // how many joints were previously connected

  // what booster reported in RobotStatesMsg subscription (updated when it changes)
  int32_t lastReportedMode = static_cast<int32_t>(booster::robot::RobotMode::kUnknown);
  int32_t lastReportedBodyControl = static_cast<int32_t>(booster::robot::BodyControl::kUnknown);
  size_t lastReportedActionsSize = 0;
  
  OnceFlag waitingForJointDataAnnouncement; ///< at startup or if joint (lowState) data lost
  OnceFlag modeChangeFailureAnnouncement; ///< trigger when a mode change first fails
  
  bool armPositionsOK = false; /**< Are the arms correctly rotated after starting? */
  unsigned armPositionWarningTime = 0; /**< Arms are not correctly rotated and was said at this timestamp. */
  bool robotInfoLogged = false;

  enum Button : int32_t { button_f1, button_stand, button_walk };
  std::array<bool, 3> buttonPressed = {}; // all false initially

  // timeWhenButtonPressed is only used in handleButtonMsg (Callback thread) so no SYNC needed to use it there
  std::array<unsigned, 3> timeWhenButtonPressed = {}; // all zero initially


  // mSentColor: packed RGB of last SetLEDLightColor call; UINT32_MAX = uninitialised sentinel.
  uint32_t mSentLedColor = UINT32_MAX;

  // annotation helper - needed since we cannot write annotations directly from
  // Booster SDK callbacks
  std::vector<std::string> pendingAnnotations;
#endif

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theFrameInfo The representation updated.
   */
  void update(FrameInfo& theFrameInfo) override {SYNC; theFrameInfo.time = jointSensorData.timestamp;}

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theJointSensorData The representation updated.
   */
#ifdef TARGET_BOOSTER
  void update(JointSensorData& theJointSensorData) override {SYNC; theJointSensorData = jointSensorData;}
#else
  void update(JointSensorData& theJointSensorData) override {SYNC; static_cast<JointAngles&>(theJointSensorData) = theJointRequest;}
#endif

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theKeyStates The representation updated.
   */
  void update(KeyStates& theKeyStates) override;

  /**
   * This method is called to update the GamePad representation
   */
  void update(GamePad& theGamePad) override;

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theRawInertialSensorData The representation updated.
   */
  void update(RawInertialSensorData& theRawInertialSensorData) override {SYNC; theRawInertialSensorData = rawInertialSensorData;}

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theSystemSensorData The representation updated.
   */
  void update(SystemSensorData& theSystemSensorData) override {SYNC; theSystemSensorData = systemSensorData;}

  /**
   * Update the RobotSafetyState
   */
  void update(RobotSafetyState &theRobotSafetyState) override;


#ifdef TARGET_BOOSTER
  template <typename MsgType, void (BoosterK1Provider::*MemberFunc)(const MsgType &)>
  auto makeHandler()
  {
    return [](const void *msg)
    {
    // #ifdef TARGET_BOOSTER
      if (theInstance)
        (theInstance->*MemberFunc)(*reinterpret_cast<const MsgType *>(msg));
    // #endif
    };
  }

  void triggerSafetyTrip(const char* reason, bool fromCallback=false);

  /**
   * Process a low state message from the robot - called from booster pub/sub thread
   */
  void handleLowStateMsg(const booster_interface::msg::LowState& lowState);

  /**
   * Process a robot status message from the robot - called from booster pub/sub thread
   */
  void handleRobotStatusMsg(const booster_interface::msg::RobotStatusDdsMsg &robotStatus);

  /// handle buttonEvent messages from the robot - called from booster pub/sub thread
  void handleButtonEventMsg(const booster_interface::msg::ButtonEventMsg &buttonEvent);

  /// handle remote control (gamepad) messages from the robot - called from booster pub/sub thread
  void handleRemoteControllerMsg(const booster_interface::msg::RemoteControllerState &remoteControllerState);

  void handleRobotStatesMsg(const booster_interface::msg::RobotStatesMsg &robotStates);

  /// change mode and update the robotMode member if successful
  /// Not threadsafe - sync externally
  bool changeRobotMode(booster::robot::RobotMode targetMode);

  // bool connectHardwareInterfaceIfNeeded();
  bool tryWaitForJointData();
  bool updateHardwareDataAvailable(bool lowStateReceived, size_t joints);
  bool checkJointRequestIsSafe(booster::robot::RobotMode targetMode);
  void updateRobotModeFromMotion();

  void logRobotInfoOnce();

  /** update the back panel LEDs as appropriate */
  void commandLightControl();
#endif

  /** Waits for the next data to arrive from the Booster robot (called by static method). */
  void waitForFrameData2();

  /** Send requests to Booster robot (called by static method). */
  void finishFrame2();

public:
  BoosterK1Provider();
  ~BoosterK1Provider();

  /** Wait for the next data to arrive from the Booster robot. */
  static void waitForFrameData();

  /** Send requests to Booster robot. */
  static void finishFrame();
};

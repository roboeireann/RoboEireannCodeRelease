/**
 * @file MIPIProvider.h
 *
 * This file declares a common base class for MIPI camera and depth providers.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Platform/Semaphore.h"
#include "Platform/Thread.h"
#include "Representations/Configuration/CameraIntrinsics.h"
#include "Representations/Configuration/CameraResolutionRequest.h"
#include "Representations/Infrastructure/CameraInfo.h"

#if defined TARGET_ROBOT && (defined __arm64__ || defined __aarch64__)
#define TARGET_BOOSTER
#include <booster/idl/sensor_msgs/Image.h>
#include <booster/idl/sensor_msgs/CameraInfo.h>
#include <booster/robot/channel/channel_subscriber.hpp>
#endif

class MIPIProvider
{
private:
  thread_local static MIPIProvider* theInstance;
  Semaphore imageSignal; /**< A signal used for synchronizing imageHandler2() and waitForFrameData2(). */
  Semaphore cameraInfoSignal; /**< A signal used for synchronizing imageHandler2() and waitForFrameData2(). */

#ifdef TARGET_BOOSTER
  booster::robot::ChannelSubscriber<sensor_msgs::msg::Image> imageSubscriber; /**< Subscriber for camera images. */
  booster::robot::ChannelSubscriber<sensor_msgs::msg::CameraInfo> cameraInfoSubscriber; /**< Subscriber for camera info. */
#endif

protected:
  DECLARE_SYNC; /**< Data is received in a different thread, so synchronize access. */
  const bool useSystemTime = false; /**< use image timestamp(false) or system timestamp(true) when image arrived? */
  CameraInfo cameraInfo; /**< The intrinsic calibration of the color camera. It is \c upper. */
  CameraIntrinsics cameraIntrinsics; /**< The intrinsic calibration of the cameras. The color camera is \c upper. */
  CameraResolutionRequest cameraResolutionRequest; /**< The resolution request for the cameras. The color camera is \c upper. */
  unsigned lastImageTimestamp = 0; /**< The timestamp of the last image received. */
  std::vector<unsigned char> imageBuffer; /**< The buffer for the image data received. */

private:
  /** Read the camera intrinsics from a configuration file. */
  bool readCameraIntrinsics();

  /** Read the initial camera resolutions from a configuration file. */
  bool readCameraResolution();

  /** Setup the camera information. */
  void updateCameraInfo();

#ifdef TARGET_BOOSTER
  /**
   * Process an image message from the camera.
   * @param image The image message.
   */
  void imageHandler(const sensor_msgs::msg::Image& image);

  /**
   * Process an camera info message from the camera.
   * @param image The camera info message.
   */
  void cameraInfoHandler(const sensor_msgs::msg::CameraInfo& cameraInfo);
#endif

protected:
  /**
   * Wait for the next frame.
   * @param timeout The timeout in ms.
   * @return \c true: Data arrived in time. \c false: waiting timed out.
   */
  bool waitForFrameData(int timeout);

  /** The method reports whether a new image is available (called by static method). */
  virtual bool isFrameDataComplete2() const = 0;

  /** The method waits for a new image (called by static method). */
  virtual void waitForFrameData2() = 0;

public:
  /**
   * Initializes the message handlers.
   * @param camera The camera that is used.
   * @param imageTopic The full DDS topic name of the image.
   * @param cameraInfoTopic The full DDS topic name of the camera info for that image.
   * @param useSystemTime Use image timestamp(false) or system timestamp(true) when image arrived
   */
  MIPIProvider(CameraInfo::Camera camera, const std::string& imageTopic, const std::string& cameraInfoTopic, const bool useSystemTime);
  ~MIPIProvider();

  /**
   * The method returns whether a new image is available.
   * @return Is an new image available?
   */
  static bool isFrameDataComplete();

  /** The method waits for a new image. */
  static void waitForFrameData();
};

/**
 * @file MIPIDepthProvider.h
 *
 * This file declares a module that grabs depth images from the MIPI camera sensor.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "MIPIProvider.h"
#include "Framework/Module.h"
#include "Representations/Infrastructure/DepthImage.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JPEGDepthImage.h"

MODULE(MIPIDepthProvider,
{,
  REQUIRES(DepthImage),
  PROVIDES(CameraInfo),
  PROVIDES(CameraIntrinsics),
  PROVIDES_WITHOUT_MODIFY(DepthImage),
  PROVIDES_WITHOUT_MODIFY(JPEGDepthImage),
  PROVIDES(FrameInfo),
  LOADS_PARAMETERS(
  {,
    (std::string) imageTopic, /**< The full DDS topic name of the image. */
    (std::string) cameraInfoTopic, /**< The full DDS topic name of the camera info for that image. */
    (int) maxWaitForImage, /**< Timeout in ms for waiting for new images after the camera was just set up. */
    (int) minWaitForImage, /**< Timeout in ms for waiting for new images when the camera was previously working. */
    (int) jpegQuality, /**< The quality of the JPEG compressing (0 = bad ... 100 = very good). */
    (bool) useSystemTimeFlag, /**< Use the system time for camera time stamp. */
    (Rangei) depthRange, /**< Valid depth range considered when compressing image. Measurements are clamped to this range. */
  }),
});

class MIPIDepthProvider : public MIPIDepthProviderBase, public MIPIProvider
{
  /**
   * This method is called when the representation provided needs to be updated.
   * @param theCameraInfo The representation updated.
   */
  void update(CameraInfo& theCameraInfo) override {SYNC; theCameraInfo = cameraInfo;}

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theCameraIntrinsics The representation updated.
   */
  void update(CameraIntrinsics& theCameraIntrinsics) override {SYNC; theCameraIntrinsics = cameraIntrinsics;}

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theCameraImage The representation updated.
   */
  void update(DepthImage& theDepthImage) override;

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theFrameInfo The representation updated.
   */
  void update(FrameInfo& theFrameInfo) override {theFrameInfo.time = theDepthImage.timestamp;}

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theJPEGDepthImage The representation updated.
   */
  void update(JPEGDepthImage& theJPEGDepthImage) override;

  /** The method reports whether a new image is available. */
  bool isFrameDataComplete2() const override;

  /** The method waits for a new image. */
  void waitForFrameData2() override;

public:
  MIPIDepthProvider();
};

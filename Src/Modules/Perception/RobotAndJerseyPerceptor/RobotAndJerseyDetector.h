/**
 * @file RobotAndJerseyDetector.h
 *
 * This file declares a module that detects Robots and their Jerseys in images with a neural network.
 * Some elements of this code are based off the older PlayersDeeptector module.
 * 
 * @author Eoin Keatley
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Tools/Math/Eigen.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/CameraImage.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Modeling/LabelImage.h"
#include "Representations/Modeling/Odometer.h"
#include "Representations/Perception/ImagePreprocessing/ECImage.h"
#include "Representations/Perception/ImagePreprocessing/BodyContour.h"
#include "Representations/Perception/ObstaclesPercepts/ObstaclesFieldPercept.h"
#include "Representations/Perception/ObstaclesPercepts/ObstaclesImagePercept.h"
#include "Representations/Perception/ObstaclesPercepts/ObstaclesPerceptorData.h"

#include "Modules/Perception/RefereeGesture/TFLiteHelperCopy.h"

MODULE(RobotAndJerseyDetector,
{,
  REQUIRES(CameraImage),
  REQUIRES(CameraInfo),
  REQUIRES(ECImage),
  REQUIRES(CameraMatrix),
  REQUIRES(GameInfo),
  REQUIRES(ImageCoordinateSystem),
  REQUIRES(Odometer),
  REQUIRES(BodyContour),
  REQUIRES(OtherObstaclesPerceptorData),
  PROVIDES(ObstaclesFieldPercept),
  REQUIRES(ObstaclesFieldPercept),
  PROVIDES(ObstaclesImagePercept),
  USES(ObstaclesPerceptorData),
  PROVIDES(ObstaclesPerceptorData),
  DEFINES_PARAMETERS(
  {,
    (float)(0.75f) confidenceThreshold, /**< The minimum confidence for a detection to be considered valid. */
    (float)(0.3f) nmsThreshold, /**< The threshold for non-maximum suppression. */
    (int)(128) modelHeight, /**< The height of the model input image. */
    (int)(160) modelWidth, /**< The width of the model input image. */
    (float)(200.0f) minWidthOnFieldNoMatch,
  }),
});

class RobotAndJerseyDetector : public RobotAndJerseyDetectorBase
{
private:
  void update(ObstaclesImagePercept& theObstaclesImagePercept) override;
  void update(ObstaclesFieldPercept& theObstaclesFieldPercept) override;
  void update(ObstaclesPerceptorData& theObstaclesPerceptorData) override;

public:
  RobotAndJerseyDetector();

private:
  TFLiteHelperCopy tfliteModel;
  float* tfliteInputPtr = nullptr;
  std::vector<ObstaclesImagePercept::Obstacle> obstaclesUpper;
  std::vector<ObstaclesImagePercept::Obstacle> obstaclesLower;
  
  void prepareInputTensor();
  Vector4f applyLetterboxingCorrection(float boxX, float boxY, float width, float height);
  int isInAllowedColours(int colourIndex) const;
  void processFeatureMap(const std::vector<float>& output, int gridW, int gridH,
                        const std::vector<float>& anchors, LabelImage& labelImage);
  
  inline float sigmoid(float x) const;
};


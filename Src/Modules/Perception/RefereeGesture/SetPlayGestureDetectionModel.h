/**
 * @file SetPlayGestureDetectionModel.h
 *
 * Executes the inference model for the SetPlayGestureDetector.
 *
 * @author Rudi Villing
 * @author Conal Hughes
 */

#include "TFLiteHelperCopy.h"
#include "Representations/Perception/SetPlayRefereeGesture.h"
#include "Representations/Perception/ImagePreprocessing/ECImage.h"
#include "Representations/Infrastructure/CameraImage.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Platform/Thread.h"
#include "Platform/Semaphore.h"
#include "Tools/ImageProcessing/Resize.h"

#include <vector>


class SetPlayGestureDetectionModel {
private:
  // REQUIRES(RobotPose);
  /* Image-Related Constants */
  static constexpr int inputBatchSize = 1;
  // static constexpr int inputHeight = 224;
  // static constexpr int inputWidth = 224; 
  static constexpr int inputHeight = 240;
  static constexpr int inputWidth = 320; ///< input image size for the model
  static constexpr int inputChannels = 3;
  static constexpr float inputColorValueScale = (1.f/255);

  const float gestureConfidenceThreshold = 0.6f; ///< threshold for gesture confidence to be considered valid

  bool enabled = false;
  TFLiteHelperCopy tflite;
  float *modelInput = nullptr;    ///< Input tensors. Populate with pixels in RGB
  std::vector<float> modelOutput;   ///< Output tensors. Provides the output of the NN
  // std::string pathToModel;            ///< Path to the model we are using
  std::vector<std::string> labels;    ///< Labels used for the output

  DECLARE_SYNC;
  Thread inferenceThread; ///< thread that will do the asynchronous inference
  Semaphore inferenceSem; ///< used to signal that a new inference should begin (initial token count is zero)

  enum { IDLE, PROCESSING, RESULT } inferenceState = IDLE;

public:
  SetPlayGestureDetectionModel(const std::string& modelName, const std::string& labelsName, bool isEnabled);
  ~SetPlayGestureDetectionModel();

  /**
   * try to set the input image for inference (if not already busy with another inference).
   * @param cameraImage  the camera image to from which to generate the possibly cropped possibly resized input
   * @param robotPose  the robot pose, used to determine the crop area
   * @param fieldDimensions  the field dimensions, used to determine the crop area
   * @param offsetX  x-offset of the crop within the possibly resized image
   * @param offsetY  y-offset of the crop within the possibly resized image
   * @param theECImage  the ECImage to use for the inference (if not already set)
   */
  bool trySetInputImage(const CameraImage& cameraImage, const RobotPose& robotPose, const FieldDimensions& fieldDimensions, const int offsetX, const int offsetY, const ECImage theECImage);
  bool getOutputs(SetPlayRefereeGesture::Gesture& detectedGesture, float& detectionConfidence);
  void saveRGBToFile(const std::string& filename);
  void prepareInputTensor(const CameraImage& cameraImage, const ECImage theECImage);
protected:
  void inferenceThreadMain();
  float getConfidence();
};
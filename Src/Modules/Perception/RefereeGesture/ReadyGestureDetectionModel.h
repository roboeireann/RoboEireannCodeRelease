/**
 * @file ReadyGestureDetectionModel.h
 *
 * Executes the inference model for the StandbyReadyGesture Detector.
 *
 * @author James Petri
 * @author Shauna Recto
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <fmt/core.h>
#include <chrono>
#include "TFLiteHelperCopy.h"
#include "tensorflow/lite/c/c_api.h"
#include "Representations/Infrastructure/CameraImage.h"
#include "Platform/Thread.h"

class ReadyGestureDetectionModel {
private:
  /* Image-Related Constants */
  static constexpr int inputBatchSize = 1;
  static constexpr int mobileNetHeight = 224;
  static constexpr int mobileNetWidth = 224; 
  static constexpr int inputChannels = 3;
  static constexpr float imageInputRescale = (1.f/255);

  bool enabled = false;
  TFLiteHelperCopy tflite;
  float *nNetInput = nullptr;    ///< Input tensors. Populate with pixels in RGB
  std::vector<float> nNetOutput;   ///< Output tensors. Provides the output of the NN
  std::string pathToModel;            ///< Path to the model we are using
  std::vector<std::string> labels;    ///< Labels used for the output

  DECLARE_SYNC;
  Thread inferenceThread; ///< thread that will do the asynchronous inference
  enum {IDLE, PROCESSING, RESULT } state = IDLE;

public:
  ReadyGestureDetectionModel(const std::string& modelName, const std::string& labelsName, bool isEnabled);
  ~ReadyGestureDetectionModel();
  bool setImage(const CameraImage& cameraImage, const int& cropPosX, const int& cropPosY); 
  // The thread classifying the gesture based on upper camera image
  void inferenceThreadMain();
  float getConfidence();
};
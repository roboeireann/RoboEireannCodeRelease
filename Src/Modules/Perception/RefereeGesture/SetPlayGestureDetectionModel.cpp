/**
 * @file SetPlayGestureDetectionModel.cpp
 *
 * Executes the inference model for the StandbyReadyGesture Detector.
 *
 * @author Rudi Villing
 * @author Conal Hughes
 */

#include "SetPlayGestureDetectionModel.h"
#include "Platform/File.h"
#include "Platform/BHAssert.h"
#include "Tools/TextLogging.h"
#include "Tools/Debugging/DebugDrawings.h"

#include <tflite_c_api.h>
#include <tflite_c_api_xnnpack_delegate.h>

#include <fstream>



DECL_TLOGGER(tlogger, "SetPlayGestureDetectionModel", TextLogging::INFO);

SetPlayGestureDetectionModel::SetPlayGestureDetectionModel(const std::string& modelName, const std::string& labelsName, bool isEnabled)
{ 
  enabled = isEnabled;
  if (enabled)
  {
    std::string modelPath = std::string(File::getBHDir()) + "/Config/NeuralNets/VisualReferee/SetPlayGesture/" + modelName;
    std::string labelPath = std::string(File::getBHDir()) + "/Config/NeuralNets/VisualReferee/SetPlayGesture/" + labelsName; 

    // TODO: read the class labels - does this need change for the new model?
    std::ifstream file(labelPath);
    std::string line;
    while (getline(file, line))
      labels.push_back(line);

    TLOGD(tlogger, "loaded class labels from '{}'", labelPath);

    // TODO: load the model with appropriate config
    tflite.loadModelFromFile(modelPath, {inputBatchSize, inputHeight, inputWidth, inputChannels}, 1);

    TLOGD(tlogger, "loaded tflite model from '{}'", modelPath);

    inferenceThread.setPriority(0);
    inferenceThread.start(this, &SetPlayGestureDetectionModel::inferenceThreadMain);
  }
}

SetPlayGestureDetectionModel::~SetPlayGestureDetectionModel()
{
  if (inferenceThread.isRunning())
    inferenceThread.stop();
}

bool SetPlayGestureDetectionModel::trySetInputImage(const CameraImage& cameraImage, const RobotPose& robotPose, const FieldDimensions& fieldDimensions, const int offsetX, const int offsetY, const ECImage theECImage)
{ 
  SYNC;
  if (inferenceState != IDLE)
    return false; // couldn't set new image for inference

  // OK 

  modelInput = tflite.getFloatInputTensor();
  // TLOGI(tlogger, "Camera width: {} and height: {}\n", cameraImage.width, cameraImage.height);

  // TODO: there's something odd about the way cameraImage.width and height works such that
  // we temporarily use these hardcoded values instead
  const int originalWidth = 640;
  const int originalHeight = 480;

  int centreX = static_cast<int>(originalWidth/2);
  int centreY = static_cast<int>(originalHeight/2);

  Vector2f refereePosition = Vector2f(0.f, fieldDimensions.yPosLeftSideline + 300.f); // TODO: use the actual referee position from the robot pose
  float distanceToReferee = (robotPose.translation - refereePosition).norm();

  int topLCropX;
  int topLCropY;
  bool crop;
  
  if (distanceToReferee < 5000.f)
  {
    // Robot is too close to the referee to see them
    // TLOGW(tlogger, "Robot is too close to the referee, just resize the image without cropping");
    crop = false; // don't crop, just resize the image
  }
  else
  {
    crop = true; // crop the image
    topLCropX = centreX - static_cast<int>(inputWidth/2) + offsetX;
    topLCropY = centreY - static_cast<int>(inputHeight/2) + offsetY;
  }
  // int topLCropX = centreX - static_cast<int>(inputWidth/2) + offsetX;
  // int topLCropY = centreY - static_cast<int>(inputHeight/2) + offsetY;

  RECTANGLE("module:SetPlayGestureDetector:cropBox", topLCropX, topLCropY, topLCropX + inputWidth,
          topLCropY + inputHeight, 3, Drawings::solidPen, ColorRGBA::blue);


  if (crop)
  {
    // crop and resize the image
    for (int y = 0; y < inputHeight; ++y)
    {
      for (int x = 0; x < inputWidth; ++x)
      {
        /* Map (x, y) in the 224x224 image to (srcX, srcY) in the cropped area */
        const int srcX = x + topLCropX;
        const int srcY = y + topLCropY;

        /* Obtaining the YUV and RGB values of the specific image */
        const PixelTypes::YUVPixel& yuv = cameraImage.getYUV(srcX, srcY);
        uint8_t r, g, b;
        ColorModelConversions::fromYUVToRGB(yuv.y, yuv.u, yuv.v, r, g, b);

        /* Converts the RGB values from 0-255 to 0-1.f */
        int idx = (y * inputWidth + x) * inputChannels;
        modelInput[idx] = inputColorValueScale * r;
        modelInput[idx + 1] = inputColorValueScale * g;
        modelInput[idx + 2] = inputColorValueScale * b;
      }
    }
  }
  else
  {
    // for (int y = 0; y < inputHeight; ++y)
    // {
    //   int srcY = static_cast<int>(y * (static_cast<float>(originalHeight) / inputHeight));
    //   for (int x = 0; x < inputWidth; ++x)
    //   {
    //     int srcX = static_cast<int>(x * (static_cast<float>(originalWidth) / inputWidth));
    //     const PixelTypes::YUVPixel& yuv = cameraImage.getYUV(srcX, srcY);
    //     uint8_t r, g, b;
    //     ColorModelConversions::fromYUVToRGB(yuv.y, yuv.u, yuv.v, r, g, b);

    //     int idx = (y * inputWidth + x) * inputChannels;
    //     modelInput[idx] = inputColorValueScale * r;
    //     modelInput[idx + 1] = inputColorValueScale * g;
    //     modelInput[idx + 2] = inputColorValueScale * b;
    //   }
    // }
      prepareInputTensor(cameraImage, theECImage);
    }

  inferenceState = PROCESSING;
  inferenceSem.post(); // signal the inference thread to run a new inference
  return true; // image submitted for processing
}


void SetPlayGestureDetectionModel::inferenceThreadMain() 
{
  Thread::nameCurrentThread("SetPlayGestureInference");

  while (inferenceThread.isRunning())
  { 
    if (inferenceSem.wait(100)) // true if signalled, false if timed out
    {
      bool inferenceRunning;
      
      {
        SYNC;
        inferenceRunning = (inferenceState == PROCESSING);
      }

      if (inferenceRunning)
      {
        unsigned long long startTime = Time::getCurrentThreadTime();

        tflite.execute();

        TLOGV(tlogger, "tflite model executed in {} microsecs", Time::getCurrentThreadTime() - startTime);

        // at this point inference is complete
        {
          SYNC;
          inferenceState = RESULT;
        }
      }
    }
  }
}

bool SetPlayGestureDetectionModel::getOutputs(SetPlayRefereeGesture::Gesture& detectedGesture, float& detectionConfidence)
{ 
  SYNC;
  if (inferenceState == RESULT) // new result available?
  {
    modelOutput = tflite.getFloatOutputTensor();

    // Print out the classifier output tensor values
    std::string outputStr;
    for (int i = 0; i < 3; ++i)
    {
      outputStr += std::to_string(modelOutput[i]);
      if (i < 2) outputStr += ", ";
    }
    TLOGV(tlogger, "tflite model outputs received: [{}]", outputStr);
    (void)detectedGesture; // TODO temporarily force use of function parameter
    (void)detectionConfidence;

    // Find the index of the maximum output value (argmax)
    int maxIdx = 0;
    float maxVal = modelOutput[0];
    for (int i = 1; i < 3; i++)
    {
      if (modelOutput[i] > maxVal)
      {
      maxVal = modelOutput[i];
      maxIdx = i;
      }
    }

    // TODO: make enum for this
    // Map index to gesture
    if (maxVal < gestureConfidenceThreshold)
    {
      // If the maximum value is below the threshold, we consider it as no gesture detected
      maxIdx = 0; // index for NONE
    }
    switch (maxIdx)
    {
      default: detectedGesture = SetPlayRefereeGesture::NONE; break;
      case 1: detectedGesture = SetPlayRefereeGesture::TOWARDS_LEFT; break;
      case 2: detectedGesture = SetPlayRefereeGesture::TOWARDS_RIGHT; break;
    }

    detectionConfidence = maxVal; 

    if (maxVal > gestureConfidenceThreshold)
    {
      TLOGV(tlogger, "detected gesture: {}, confidence: {}", static_cast<int>(detectedGesture), detectionConfidence);
    }
    

    inferenceState = IDLE;
    return true; // new outputs set
  }
  else
  {
    return false; // no new outputs set as model was still processing/running
  }
}

void SetPlayGestureDetectionModel::prepareInputTensor(const CameraImage& theCameraImage, const ECImage theECImage)
{

  TLOGI(tlogger, "preparing Input");
    // Prepare resized images
    Image<PixelTypes::GrayscaledPixel> shrunkY;
    Image<unsigned short> shrunkUV;
    
    const unsigned int scale = static_cast<unsigned int>(std::log2(theECImage.grayscaled.width / inputWidth) + 0.5);
    Resize::shrinkY(scale, theECImage.grayscaled, shrunkY);
    Resize::shrinkUV(scale, theCameraImage, shrunkUV);

    const int paddingOffset = 0;
    float *tensorData = modelInput;

    for (int y = 0; y < inputHeight; ++y)
    {
      for (int x = 0; x < inputWidth; ++x)
      {
        // Extract YUV values with bounds checking
        const int shrunkY_y = y - paddingOffset;
        const uint8_t Y =
            (shrunkY_y >= 0 && shrunkY_y < static_cast<int>(shrunkY.height) && x < static_cast<int>(shrunkY.width))
                ? shrunkY[shrunkY_y][x]
                : 0;

        const int uvY = (y - paddingOffset) / 2;
        const int uvX = x / 2;
        uint8_t U = 128, V = 128;

        if (uvY >= 0 && uvY < static_cast<int>(shrunkUV.height) && uvX >= 0 && uvX < static_cast<int>(shrunkUV.width))
        {
          const unsigned short uv = shrunkUV[uvY][uvX];
          U = static_cast<uint8_t>(uv & 0xFF);
          V = static_cast<uint8_t>((uv >> 8) & 0xFF);
        }

        // Convert YUV to RGB and normalize
        uint8_t r, g, b;
        const int tensorIndex = (y * inputWidth + x) * 3;
        ColorModelConversions::fromYUVToRGB(Y, U, V, r, g, b);

        tensorData[tensorIndex] = r / 255.0f;
        tensorData[tensorIndex + 1] = g / 255.0f;
        tensorData[tensorIndex + 2] = b / 255.0f;
      }
    }
}
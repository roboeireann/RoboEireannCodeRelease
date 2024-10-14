/**
 * @file ReadyGestureDetectionModel.cpp
 *
 * Executes the inference model for the StandbyReadyGesture Detector.
 *
 * @author James Petri
 * @author Shauna Recto
 */

#include "ReadyGestureDetectionModel.h"
#include "Tools/TextLogging.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Platform/File.h"
#include "Platform/BHAssert.h"

DECL_TLOGGER(tlogger, "ReadyGestureDetectionModel", TextLogging::INFO);

ReadyGestureDetectionModel::ReadyGestureDetectionModel(const std::string& modelName, const std::string& labelsName, bool isEnabled)
{ 
  enabled = isEnabled;
  if (enabled)
  {
    std::string modelPath = std::string(File::getBHDir()) + "/Config/NeuralNets/VisualReferee/BinaryClassifier/" + modelName;
    std::string labelPath = std::string(File::getBHDir()) + "/Config/NeuralNets/VisualReferee/BinaryClassifier/" + labelsName;

    /* Setting the file and label Paths */
    std::ifstream file(labelPath);
    std::string line;
    while (getline(file, line)) {
        labels.push_back(line);
    }
    pathToModel = modelPath;

    /* Load the model */
    tflite.loadModelFromFile(pathToModel, {inputBatchSize, mobileNetHeight, mobileNetWidth, inputChannels}, 1);

    inferenceThread.setPriority(0);
    inferenceThread.start(this, &ReadyGestureDetectionModel::inferenceThreadMain);
  }
}

ReadyGestureDetectionModel::~ReadyGestureDetectionModel()
{
  if (enabled)
    inferenceThread.stop();
}

bool ReadyGestureDetectionModel::setImage(const CameraImage& cameraImage, const int& cropPosX, const int& cropPosY)
{ 
  SYNC;
  if (state != IDLE)
    return false;

  nNetInput = tflite.getFloatInputTensor();
  // TLOGI(tlogger, "Camera width: {} and height: {}\n", cameraImage.width, cameraImage.height);

  /* Used for now. Will replace */
  const int originalWidth = 640;
  const int originalHeight = 480;

  int centreX = static_cast<int>(originalWidth/2);
  int centreY = static_cast<int>(originalHeight/2);

  int topLCropX = centreX - static_cast<int>(mobileNetWidth/2) + cropPosX;
  int topLCropY = centreY - static_cast<int>(mobileNetHeight/2) + cropPosY;

  RECTANGLE("module:ReadyGestureDetector:cropBox", topLCropX, topLCropY, topLCropX + mobileNetWidth,
          topLCropY + mobileNetHeight, 3, Drawings::solidPen, ColorRGBA::blue);


  /* Loop to crop and resize the image */
  for (int y = 0; y < mobileNetHeight; ++y)
  {
    for (int x = 0; x < mobileNetWidth; ++x)
    {
      /* Map (x, y) in the 224x224 image to (srcX, srcY) in the cropped area */
      int srcX = x + topLCropX;
      int srcY = y + topLCropY;

      /* Obtaining the YUV and RGB values of the specific image */
      const PixelTypes::YUVPixel& yuv = cameraImage.getYUV(srcX, srcY);
      uint8_t r, g, b;
      ColorModelConversions::fromYUVToRGB(yuv.y, yuv.u, yuv.v, r, g, b);

      /* Converts the RGB values from 0-255 to 0-1.f */
      int idx = (y * mobileNetWidth + x) * inputChannels;
      nNetInput[idx] = imageInputRescale * r;
      nNetInput[idx + 1] = imageInputRescale * g;
      nNetInput[idx + 2] = imageInputRescale * b;
    }
  }

  state = PROCESSING;
  return true; // image submitted for processing
}

void ReadyGestureDetectionModel::inferenceThreadMain() 
{
  while(inferenceThread.isRunning())
  { 
    bool processRunning = false;
    
    {
      SYNC;
      if (state == PROCESSING)
      {
        processRunning = true;
      }
    }

    if (processRunning)
    {
      tflite.execute();
      SYNC;
      state = RESULT;
    }
    else
    {
      Thread::sleep(200);
    }
  }
}

float ReadyGestureDetectionModel::getConfidence()
{ 
  SYNC;
  if (state == RESULT)
  {
    nNetOutput = tflite.getFloatOutputTensor();
    state = IDLE;
    /* Clamping if values aren't within 0 to 1 */
    if ((nNetOutput[0] <= 1.f) && (nNetOutput[0] >= 0.f))
      return nNetOutput[0];
    else if (nNetOutput[0] >= 1.f)
      return 1.1f;
    else if (nNetOutput[0] <= 0.f)
      return -0.1f;
    else /* If we get garbage data */
      return 0.f;
  } else {
    return 1.f;
  }
}
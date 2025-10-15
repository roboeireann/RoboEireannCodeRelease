/**
 * @file RobotAndJerseyDetector.cpp
 *
 * This file implements a module that detects Robots and their Jerseys in images with a neural network.
 * Some elements of this code are based off the older PlayersDeeptector module.
 * 
 * @author Eoin Keatley
 */

#include "RobotAndJerseyDetector.h"
#include "Platform/File.h"
#include "Tools/ImageProcessing/Resize.h"
#include "Tools/ImageProcessing/ColorModelConversions.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Transformation.h"

#include <vector>  


MAKE_MODULE(RobotAndJerseyDetector, perception);

DECL_TLOGGER(tlogger, "RobotAndJerseyDetector", TextLogging::INFO);

RobotAndJerseyDetector::RobotAndJerseyDetector() 
{
    std::string tfliteModelPath = std::string(File::getBHDir()) + "/Config/NeuralNets/RobotAndJerseyDetector/RobotAndJerseyModel.tflite";
    std::vector<int> inputDims = {1, modelHeight, modelWidth, 3};
    
    tfliteModel.loadModelFromFile(tfliteModelPath, inputDims);
    tfliteInputPtr = tfliteModel.getFloatInputTensor();
}

void RobotAndJerseyDetector::update(ObstaclesImagePercept& theObstaclesImagePercept)
{
    theObstaclesImagePercept.obstacles = theCameraInfo.camera == CameraInfo::upper ? obstaclesUpper : obstaclesLower;
}

void RobotAndJerseyDetector::update(ObstaclesPerceptorData& theObstaclesPerceptorData)
{
    theObstaclesPerceptorData.cameraInfo = theCameraInfo;
    theObstaclesPerceptorData.cameraMatrix = theCameraMatrix;
    theObstaclesPerceptorData.imageCoordinateSystem = theImageCoordinateSystem;
}

void RobotAndJerseyDetector::update(ObstaclesFieldPercept& theObstaclesFieldPercept)
{

    std::vector<ObstaclesImagePercept::Obstacle>& obstacles = theCameraInfo.camera == CameraInfo::upper ? obstaclesUpper : obstaclesLower;
    obstacles.clear();
    theObstaclesFieldPercept.obstacles.clear();
    const_cast<ObstaclesPerceptorData&>(theObstaclesPerceptorData).incompleteObstacles.clear();

    LabelImage labelImage;
    
    STOPWATCH("RobotAndJerseyDetector:prepare_input_tensor")
    { 
        prepareInputTensor(); 
    }

    STOPWATCH("RobotAndJerseyDetector:run_tflite_model")
    {
        tfliteModel.execute();
    }

    STOPWATCH("RobotAndJerseyDetector:process_output")
    {
        std::vector<float> anchors_scale1 = {10, 14, 23, 27, 37, 58};  /**< 3 anchors for 5x4 grid */
        std::vector<float> anchors_scale2 = {81, 82, 135, 169, 344, 319}; /**< 3 anchors for 10x8 grid */

        std::vector<float> output5x4 = tfliteModel.getFloatOutputTensor(0);
        std::vector<float> output10x8 = tfliteModel.getFloatOutputTensor(1);

        processFeatureMap(output5x4, 5, 4, anchors_scale2, labelImage);
        processFeatureMap(output10x8, 10, 8, anchors_scale1, labelImage);

        labelImage.nonMaximumSuppression(nmsThreshold);
        labelImage.bigBoxSuppression();
    }

    for(const LabelImage::Annotation& box : labelImage.annotations)
    {
        DRAW_TEXT("module:RobotAndJerseyDetector:image", box.upperLeft.x(), box.upperLeft.y() - 2, 10, ColorRGBA::black, "prob: " << box.probability);
        
        obstacles.emplace_back(ObstaclesImagePercept::Obstacle());
        obstacles.back().top = static_cast<int>(box.upperLeft.y());
        obstacles.back().bottom = static_cast<int>(box.lowerRight.y());
        obstacles.back().left = static_cast<int>(box.upperLeft.x());
        obstacles.back().right = static_cast<int>(box.lowerRight.x());
        obstacles.back().fallen = box.fallen;
        obstacles.back().jerseyColour = box.jerseyColour;
        obstacles.back().probability = box.probability;
        obstacles.back().bottomFound = true;
    }

    auto it = obstacles.begin();
    while(it != obstacles.end())
    {
        bool validObstacle = false;
        ObstaclesImagePercept::Obstacle& obstacleInImage = *it;
        ObstaclesFieldPercept::Obstacle obstacleOnField;
        
        if((validObstacle = (Transformation::imageToRobot(theImageCoordinateSystem.toCorrected(Vector2i(obstacleInImage.left, obstacleInImage.bottom)), theCameraMatrix, theCameraInfo, obstacleOnField.left) &&
                             Transformation::imageToRobot(theImageCoordinateSystem.toCorrected(Vector2i(obstacleInImage.right, obstacleInImage.bottom)), theCameraMatrix, theCameraInfo, obstacleOnField.right))))
        {
        obstacleOnField.fallen = obstacleInImage.fallen;
        obstacleOnField.center = (obstacleOnField.left + obstacleOnField.right) * 0.5f;

        if(obstacleInImage.jerseyColour == theGameInfo.ourTeam().fieldPlayerColor || obstacleInImage.jerseyColour == theGameInfo.ourTeam().goalkeeperColor)
        {
            obstacleOnField.type = ObstaclesFieldPercept::ownPlayer;
        }
        else if (obstacleInImage.jerseyColour == theGameInfo.opponentTeam().fieldPlayerColor || obstacleInImage.jerseyColour == theGameInfo.opponentTeam().goalkeeperColor)
        {
            obstacleOnField.type = ObstaclesFieldPercept::opponentPlayer;
        }
        else 
        {
            obstacleOnField.type = ObstaclesFieldPercept::unknown;
        }

        const Rangea range(obstacleOnField.right.angle(), obstacleOnField.left.angle());
        

        if(theCameraInfo.camera == CameraInfo::upper)
        {
          if(obstacleInImage.bottom > theECImage.grayscaled.height * 0.9f)
              for(const ObstaclesFieldPercept::Obstacle& incompleteObstacle : theOtherObstaclesPerceptorData.incompleteObstacles)
                    {
                        const Rangea rangeLower((theOdometer.odometryOffset.inverse() * incompleteObstacle.right).angle(), (theOdometer.odometryOffset.inverse() * incompleteObstacle.left).angle());
                        if(range.min <= rangeLower.max && rangeLower.min <= range.max)
                            obstacleInImage.bottomFound = false;
                    }
          if(!obstacleInImage.bottomFound)
                {
                    const_cast<ObstaclesPerceptorData&>(theObstaclesPerceptorData).incompleteObstacles.emplace_back(obstacleOnField);
                }
                else if((validObstacle = obstacleInImage.right - obstacleInImage.left < static_cast<int>(theECImage.grayscaled.width / 2)))
                {
                    obstacleOnField.center = (obstacleOnField.left + obstacleOnField.right) * 0.5f;
                    theObstaclesFieldPercept.obstacles.emplace_back(obstacleOnField);
                }
        }
        else
        {
          if((validObstacle = obstacleInImage.top < static_cast<int>(theECImage.grayscaled.height / 16) || static_cast<float>(obstacleInImage.bottom - obstacleInImage.top) / theECImage.grayscaled.height > 0.5f))
                {
                    float minWidth = minWidthOnFieldNoMatch;
                    bool hasMatch = false;
                    for(const ObstaclesFieldPercept::Obstacle& incompleteObstacle : theOtherObstaclesPerceptorData.incompleteObstacles)
                    {
                        const Rangea rangeUpper((theOdometer.odometryOffset.inverse() * incompleteObstacle.right).angle(), (theOdometer.odometryOffset.inverse() * incompleteObstacle.left).angle());
                        if((hasMatch = (range.min <= rangeUpper.max && rangeUpper.min <= range.max)))
                        {
                            obstacleOnField.type = incompleteObstacle.type;
                            minWidth = minWidthOnFieldNoMatch / 2;
                            break;
                        }
                    }
                    obstacleOnField.fallen = !hasMatch && (obstacleInImage.right - obstacleInImage.left >= static_cast<int>(theECImage.grayscaled.width / 2) ||
                                                          obstacleInImage.bottom - obstacleInImage.top >= static_cast<int>(theECImage.grayscaled.height / 2));

                    if((validObstacle = (obstacleOnField.right - obstacleOnField.left).squaredNorm() >= sqr(minWidth)))
                    {
                        const_cast<ObstaclesPerceptorData&>(theObstaclesPerceptorData).incompleteObstacles.emplace_back(obstacleOnField);
                        theObstaclesFieldPercept.obstacles.emplace_back(obstacleOnField);
                    }
                    else if(!validObstacle && (obstacleOnField.right - obstacleOnField.left).squaredNorm() >= sqr(minWidthOnFieldNoMatch / 2))
                        const_cast<ObstaclesPerceptorData&>(theObstaclesPerceptorData).incompleteObstacles.emplace_back(obstacleOnField);
                }
        }
        }

        it = validObstacle ? it + 1 : obstacles.erase(it);
    }
}


void RobotAndJerseyDetector::prepareInputTensor()
{
    // Prepare resized images
    Image<PixelTypes::GrayscaledPixel> shrunkY;
    Image<unsigned short> shrunkUV;
    
    STOPWATCH("RobotAndJerseyDetector:image_resizing")
    {
        const unsigned int scale = static_cast<unsigned int>(std::log2(theECImage.grayscaled.width / modelWidth) + 0.5);
        Resize::shrinkY(scale, theECImage.grayscaled, shrunkY);
        Resize::shrinkUV(scale, theCameraImage, shrunkUV);
    }

    STOPWATCH("RobotAndJerseyDetector:tensor_preparation_YUVtoRGB")
    {
        const int paddingOffset = 4;
        float* tensorData = tfliteInputPtr;
        
        for(int y = 0; y < modelHeight; ++y)
        {
            for(int x = 0; x < modelWidth; ++x)
            {
                // Extract YUV values with bounds checking
                const int shrunkY_y = y - paddingOffset;
                const uint8_t Y = (shrunkY_y >= 0 && shrunkY_y < static_cast<int>(shrunkY.height) && 
                                  x < static_cast<int>(shrunkY.width)) ? shrunkY[shrunkY_y][x] : 0;
                
                const int uvY = (y - paddingOffset) / 2;
                const int uvX = x / 2;
                uint8_t U = 128, V = 128;
                
                if(uvY >= 0 && uvY < static_cast<int>(shrunkUV.height) && 
                   uvX >= 0 && uvX < static_cast<int>(shrunkUV.width))
                {
                    const unsigned short uv = shrunkUV[uvY][uvX];
                    U = static_cast<uint8_t>(uv & 0xFF);
                    V = static_cast<uint8_t>((uv >> 8) & 0xFF);
                }

                // Map model coordinates back to original image coordinates for body contour check
                const int originalX = static_cast<int>((x * theECImage.grayscaled.width) / modelWidth);
                const int originalY = static_cast<int>((y * theECImage.grayscaled.height) / modelHeight);
                const bool inBodyContour = theBodyContour.getBottom(originalX, theCameraInfo.height) <= originalY;
 
 
                const int tensorIndex = (y * modelWidth + x) * 3;
 
                if(theCameraInfo.camera == CameraInfo::lower && inBodyContour)
                {
                    // Zero out body contour pixels (black masking)
                    tensorData[tensorIndex] = 0.0f;     // R
                    tensorData[tensorIndex + 1] = 0.0f; // G
                    tensorData[tensorIndex + 2] = 0.0f; // B                  
                    continue;
                }
                
                // Convert YUV to RGB and normalize
                uint8_t r, g, b;
                ColorModelConversions::fromYUVToRGB(Y, U, V, r, g, b);
                
                tensorData[tensorIndex]     = r / 255.0f;
                tensorData[tensorIndex + 1] = g / 255.0f;
                tensorData[tensorIndex + 2] = b / 255.0f;
            }
        }
    }
}


void RobotAndJerseyDetector::processFeatureMap(const std::vector<float>& output, int gridW, int gridH,
                                               const std::vector<float>& anchors, LabelImage& labelImage)
{
  constexpr int numAnchors = 3;
  constexpr int paramsPerAnchor = 18;

  for (int gy = 0; gy < gridH; ++gy)
  {
    for (int gx = 0; gx < gridW; ++gx)
    {
      for (int anchor = 0; anchor < numAnchors; ++anchor)
      {
        const int baseIdx = (gy * gridW + gx) * numAnchors * paramsPerAnchor + anchor * paramsPerAnchor;
        const float confidence = sigmoid(output[baseIdx + 4]);

        if (confidence <= confidenceThreshold) 
          continue;

        // Extract bounding box parameters
        const float tx = output[baseIdx + 0];
        const float ty = output[baseIdx + 1];
        const float tw = output[baseIdx + 2];
        const float th = output[baseIdx + 3];

        // Calculate normalized bounding box coordinates
        const float boxX = (sigmoid(tx) + gx) / gridW;
        const float boxY = (sigmoid(ty) + gy) / gridH;
        const float width = std::exp(tw) * anchors[anchor * 2] / modelWidth;
        const float height = std::exp(th) * anchors[anchor * 2 + 1] / modelHeight;

        const Vector4f finalBox = applyLetterboxingCorrection(boxX, boxY, width, height);

        // Determine robot class (standing vs fallen)
        const float robotProb = sigmoid(output[baseIdx + 5]);
        const float fallenProb = sigmoid(output[baseIdx + 6]);
        const int predictedClass = (robotProb > fallenProb) ? 0 : 1;

        // Find best jersey color
        int selectedColour = -1;
        float maxColourProb = 0.0f;
        
        for (int colourIdx = 0; colourIdx < 11; ++colourIdx)
        {
          const float colourProb = sigmoid(output[baseIdx + 7 + colourIdx]);
          
          if (colourProb > maxColourProb)
          {
            maxColourProb = colourProb;
            if (isInAllowedColours(colourIdx) != 0)
            {
              selectedColour = isInAllowedColours(colourIdx);
            }
            
          }
        }

        // Create annotation
        LabelImage::Annotation annotation;
        annotation.labelType = LabelImage::Annotation::Robot;
        annotation.fallen = predictedClass;
        annotation.upperLeft = Vector2f(finalBox.x(), finalBox.y());
        annotation.lowerRight = Vector2f(finalBox.x() + finalBox.z(), finalBox.y() + finalBox.w());
        annotation.center = Vector2f(finalBox.x() + finalBox.z() / 2.0f, finalBox.y() + finalBox.w() / 2.0f);
        annotation.probability = confidence;
        annotation.jerseyColour = selectedColour;
        
        labelImage.annotations.push_back(annotation);
      }
    }
  }
}

Vector4f RobotAndJerseyDetector::applyLetterboxingCorrection(float boxX, float boxY, float width, float height)
{
  const float originalImageHeight = static_cast<float>(theECImage.grayscaled.height);
  const float originalImageWidth = static_cast<float>(theECImage.grayscaled.width);

  const float scaleFactor = std::min(static_cast<float>(modelWidth) / originalImageWidth,
                                     static_cast<float>(modelHeight) / originalImageHeight);

  const float scaledWidth = originalImageWidth * scaleFactor;
  const float scaledHeight = originalImageHeight * scaleFactor;

  const float padX = (modelWidth - scaledWidth) / 2.0f / modelWidth;
  const float padY = (modelHeight - scaledHeight) / 2.0f / modelHeight;

  float normX = (boxX - padX) * modelWidth / scaledWidth;
  float normY = (boxY - padY) * modelHeight / scaledHeight;
  float normWidth = width * modelWidth / scaledWidth;
  float normHeight = height * modelHeight / scaledHeight;

  normX -= normWidth / 2.0f;
  normY -= normHeight / 2.0f;

  const float finalX = normX * originalImageWidth;
  const float finalY = normY * originalImageHeight;
  const float finalWidth = normWidth * originalImageWidth;
  const float finalHeight = normHeight * originalImageHeight;

  return Vector4f(finalX, finalY, finalWidth, finalHeight);
}

int RobotAndJerseyDetector::isInAllowedColours(int colourIndex) const
{
  static const int trainingColourToGameController[10] = {
    3, // black
    0, // blue
    9, // gray
    5, // green
    6, // orange
    7, // purple
    1, // red
    4, // white
    2, // yellow
  };

  if (colourIndex == 9) 
    return -1;
  
  int mappedColour = trainingColourToGameController[colourIndex];

   if(mappedColour == theGameInfo.ourTeam().fieldPlayerColor ||
         mappedColour == theGameInfo.ourTeam().goalkeeperColor ||
         mappedColour == theGameInfo.opponentTeam().fieldPlayerColor ||
         mappedColour == theGameInfo.opponentTeam().goalkeeperColor)
         {
          return mappedColour;
         }
  return 0;
}


inline float RobotAndJerseyDetector::sigmoid(float x) const
{
    return 1.0f / (1.0f + std::exp(-x));
}
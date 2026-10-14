/**
 * @file TRTMultiPerceptor.cpp
 * @see TRTMultiPerceptor.h
 */

#include "TRTMultiPerceptor.h"
#include "Platform/BHAssert.h"
#include "Platform/File.h"
#include "Debugging/Debugging.h"
#include "Debugging/Stopwatch.h"
#include "Framework/Settings.h"
#include "ImageProcessing/ColorModelConversions.h"
#include "TextLogging/TextLogging.h"

DECL_TLOGGER(tlogger, "TRTMultiPerceptor", TextLogging::DEBUG);

#ifndef BUILD_BOOSTER
#include <memory>
#endif

#include <algorithm>
#include <cmath>

MAKE_MODULE(TRTMultiPerceptor);

TRTMultiPerceptor::TRTMultiPerceptor()
{
#ifdef BUILD_BOOSTER
  if(!trt.load(enginePath))
  {
    TLOGE(tlogger, "TRTMultiPerceptor: failed to load engine from Config/{}", enginePath);
    return;
  }

  if(!trt.hasTensor("images"))
    TLOGE(tlogger, "TRTMultiPerceptor: tensor 'images' not found in engine");
  if(!trt.hasTensor("output0"))
    TLOGE(tlogger, "TRTMultiPerceptor: tensor 'output0' not found in engine");
#else
  auto model = std::make_unique<NeuralNetworkONNX::Model>(
      std::string(File::getBHDir()) + "/Config/" + onnxModelPath);
  NeuralNetworkONNX::CompilationSettings settings;
  network.compile(*model, settings);
  if(!network.valid())
    TLOGE(tlogger, "TRTMultiPerceptor: failed to load ONNX model from {}", onnxModelPath);
  else
    TLOGI(tlogger, "TRTMultiPerceptor: loaded ONNX model '{}'", onnxModelPath);
#endif
}

/* Converts YUYV camera image to RGB CHW float32 normalised to [0,1].
   Layout: [R_plane | G_plane | B_plane], each H×W values. */
void TRTMultiPerceptor::preprocess()
{
  const int W = inputWidth;
  const int H = inputHeight;

#ifdef BUILD_BOOSTER
  std::vector<float>& buf = trt.hostBuffer("images");
  float* planeR = buf.data();
  float* planeG = buf.data() + H * W;
  float* planeB = buf.data() + 2 * H * W;
#else
  // NeuralNetworkONNX::CompiledNN::Tensor is a span whose data() points directly
  // into the pre-allocated ORT input tensor — safe to write before apply().
  auto inTensor = network.input(0);
  float* planeR = inTensor.data();
  float* planeG = inTensor.data() + H * W;
  float* planeB = inTensor.data() + 2 * H * W;
#endif

  for(int y = 0; y < H; ++y)
  {
    for(int x = 0; x < W; ++x)
    {
      // Each YUYVPixel covers two horizontal pixels (x and x+1).
      const PixelTypes::YUYVPixel& yuyv = theCameraImage[y][x / 2];
      const unsigned char Y = (x & 1) ? yuyv.y1 : yuyv.y0;
      const unsigned char U = yuyv.u;
      const unsigned char V = yuyv.v;

      unsigned char R, G, B;
      ColorModelConversions::fromYUVToRGB(Y, U, V, R, G, B);

      const int idx = y * W + x;
      planeB[idx] = static_cast<float>(B) / 255.0f;
      planeG[idx] = static_cast<float>(G) / 255.0f;
      planeR[idx] = static_cast<float>(R) / 255.0f;
    }
  }
}

bool TRTMultiPerceptor::apply()
{
#ifdef BUILD_BOOSTER
  if(!trt.isLoaded())
    return false;
#else
  if(!network.valid())
    return false;
#endif

  if(theCameraInfo.width != inputWidth || theCameraInfo.height != inputHeight)
  {
    TLOGE(tlogger, "TRTMultiPerceptor: camera {}x{} does not match model input {}x{}",
          theCameraInfo.width, theCameraInfo.height, inputWidth, inputHeight);
    return false;
  }

#ifdef BUILD_BOOSTER
  if(trt.hostBuffer("images").size() != static_cast<std::size_t>(3 * inputHeight * inputWidth))
  {
    TLOGE(tlogger, "TRTMultiPerceptor: engine input size {} does not match 3x{}x{}",
          trt.hostBuffer("images").size(), inputHeight, inputWidth);
    return false;
  }
#endif

  if(lastPrediction == theCameraImage.timestamp)
    return true;

  STOPWATCH("module:TRTMultiPerceptor:preprocess")
    preprocess();

  STOPWATCH("module:TRTMultiPerceptor:infer")
  {
#ifdef BUILD_BOOSTER
    if(!trt.infer())
    {
      TLOGE(tlogger, "TRTMultiPerceptor: inference failed");
      return false;
    }
#else
    network.apply();
#endif
  }

  lastPrediction = theCameraImage.timestamp;
  return true;
}

void TRTMultiPerceptor::update(BallPercept& ballPercept)
{
  ballPercept.status = BallPercept::notSeen;

  if(!apply())
    return;

  static constexpr int stride = 6;
#ifdef BUILD_BOOSTER
  const std::vector<float>& outBuf = trt.hostBuffer("output0");
  const float* out = outBuf.data();
  const int maxDet = static_cast<int>(outBuf.size()) / stride;
#else
  const auto outTensor = network.output(0);
  const float* out = outTensor.data();
  const int maxDet = static_cast<int>(outTensor.size()) / stride;
#endif

  DECLARE_DEBUG_DRAWING("module:TRTMultiPerceptor:ball:image", "drawingOnImage");

  float bestConf = ballConfidenceThreshold;

  for(int i = 0; i < maxDet; ++i)
  {
    const float conf = out[i * stride + 4];
    const int   cls  = static_cast<int>(std::round(out[i * stride + 5]));

    if(cls != ballClassId || conf < bestConf)
      continue;

    const float x1 = out[i * stride + 0];
    const float y1 = out[i * stride + 1];
    const float x2 = out[i * stride + 2];
    const float y2 = out[i * stride + 3];
    const float width = x2 - x1;
    const float height = y2 - y1;

    const float radiusInImage = std::max(width, height) * 0.5f;

    constexpr float edgeTolerance = 2.f;
    const bool clippedLeft = x1 <= edgeTolerance;
    const bool clippedRight = x2 >= static_cast<float>(theCameraInfo.width) - edgeTolerance;
    const bool clippedTop = y1 <= edgeTolerance;
    const bool clippedBottom = y2 >= static_cast<float>(theCameraInfo.height) - edgeTolerance;

    float centreX;
    if (clippedLeft)
      centreX = x2 - radiusInImage;
    else if (clippedRight)
      centreX = x1 + radiusInImage;
    else
      centreX = (x1 + x2) * 0.5f;

    float centreY;
    if (clippedTop)
      centreY = y2 - radiusInImage;
    else if (clippedBottom)
      centreY = y1 + radiusInImage;
    else
      centreY = (y1 + y2) * 0.5f;

    const Vector2f centre(centreX, centreY);

    Vector2f posOnField;
    Matrix2f cov;
    if(!theMeasurementCovariance.transformWithCovLegacy(centre, theBallSpecification.radius, Vector2f(0.04f, 0.06f), posOnField, cov))
      continue;

    bestConf = conf;
    ballPercept.positionInImage = centre;
    ballPercept.radiusInImage = radiusInImage;
    ballPercept.positionOnField = posOnField;
    ballPercept.radiusOnField = theBallSpecification.radius;
    ballPercept.covarianceOnField = cov;
    ballPercept.status = BallPercept::seen;

    RECTANGLE("module:TRTMultiPerceptor:ball:image",
              static_cast<int>(x1), static_cast<int>(y1),
              static_cast<int>(x2), static_cast<int>(y2),
              2, Drawings::solidPen, ColorRGBA::cyan);
  }
}

/* Bounding boxes in image coordinates for JerseyClassifierProvider and RobotDetector downstream. */
void TRTMultiPerceptor::update(ObstaclesImagePercept& obstaclesImagePercept)
{
  obstaclesImagePercept.obstacles.clear();

  if(!apply())
    return;

  // Output shape: [1, max_det, 6] — each row: [x1, y1, x2, y2, conf, cls_id]
  static constexpr int stride = 6;
#ifdef BUILD_BOOSTER
  const std::vector<float>& outBuf = trt.hostBuffer("output0");
  const float* out = outBuf.data();
  const int maxDet = static_cast<int>(outBuf.size()) / stride;
#else
  const auto outTensor = network.output(0);
  const float* out = outTensor.data();
  const int maxDet = static_cast<int>(outTensor.size()) / stride;
#endif

  for(int i = 0; i < maxDet; ++i)
  {
    const float conf = out[i * stride + 4];
    const int   cls  = static_cast<int>(std::round(out[i * stride + 5]));

    if((cls != robotClassId && (humanClassId < 0 || cls != humanClassId)) || conf < robotConfidenceThreshold)
      continue;

    const float x1 = out[i * stride + 0];
    const float y1 = out[i * stride + 1];
    const float x2 = out[i * stride + 2];
    const float y2 = out[i * stride + 3];
    
    ObstaclesImagePercept::Obstacle obs;
    obs.left        = static_cast<int>(x1);
    obs.top         = static_cast<int>(y1);
    obs.right       = static_cast<int>(x2);
    obs.bottom      = static_cast<int>(y2);
    obs.bottomFound = true;
    obs.fallen      = (humanClassId >= 0 && cls == humanClassId);
    obs.confidence  = conf;

    obstaclesImagePercept.obstacles.push_back(obs);

    DEBUG_DRAWING("representation:ObstaclesImagePercept:image", "drawingOnImage")
    {
      RECTANGLE("representation:ObstaclesImagePercept:image",
                obs.left, obs.top, obs.right, obs.bottom, 2,
                Drawings::solidPen, ColorRGBA::orange);
    }
  }
}

void TRTMultiPerceptor::update(MLIntersectionsPercept& mlIntersectionsPercept)
{
  mlIntersectionsPercept.intersections.clear();

  if(!apply())
    return;

  static constexpr int stride = 6;
#ifdef BUILD_BOOSTER
  const std::vector<float>& outBuf = trt.hostBuffer("output0");
  const float* out = outBuf.data();
  const int maxDet = static_cast<int>(outBuf.size()) / stride;
#else
  const auto outTensor = network.output(0);
  const float* out = outTensor.data();
  const int maxDet = static_cast<int>(outTensor.size()) / stride;
#endif

  DECLARE_DEBUG_DRAWING("module:TRTMultiPerceptor:intersections:image", "drawingOnImage");

  for(int i = 0; i < maxDet; ++i)
  {
    const float conf = out[i * stride + 4];
    const int   cls  = static_cast<int>(std::round(out[i * stride + 5]));

    if(conf < intersectionConfidenceThreshold)
      continue;

    MLIntersectionsPercept::Intersection::IntersectionType type;
    if((lIntersectionClassId >= 0 && cls == lIntersectionClassId) ||
       (cornerClassId >= 0 && cls == cornerClassId))
      type = MLIntersectionsPercept::Intersection::L;
    else if(tIntersectionClassId >= 0 && cls == tIntersectionClassId)
      type = MLIntersectionsPercept::Intersection::T;
    else if(xIntersectionClassId >= 0 && cls == xIntersectionClassId)
      type = MLIntersectionsPercept::Intersection::X;
    else
      continue;

    const float x1 = out[i * stride + 0];
    const float y1 = out[i * stride + 1];
    const float x2 = out[i * stride + 2];
    const float y2 = out[i * stride + 3];
    const Vector2f centre((x1 + x2) * 0.5f, (y1 + y2) * 0.5f);

    Vector2f posOnField;
    Matrix2f cov;
    if(!theMeasurementCovariance.transformWithCovLegacy(centre, 0.f, Vector2f(0.04f, 0.06f), posOnField, cov))
      continue;

    mlIntersectionsPercept.intersections.emplace_back(type, posOnField, cov);

    const ColorRGBA colour = type == MLIntersectionsPercept::Intersection::L ? ColorRGBA::green
                           : type == MLIntersectionsPercept::Intersection::T ? ColorRGBA::yellow
                                                                             : ColorRGBA::red;
    RECTANGLE("module:TRTMultiPerceptor:intersections:image",
              static_cast<int>(x1), static_cast<int>(y1),
              static_cast<int>(x2), static_cast<int>(y2),
              2, Drawings::solidPen, colour);
  }
}

void TRTMultiPerceptor::update(PenaltyMarkPercept& penaltyMarkPercept)
{
  penaltyMarkPercept.wasSeen = false;

  if(!apply() || penaltyMarkClassId < 0)
    return;

  static constexpr int stride = 6;
#ifdef BUILD_BOOSTER
  const std::vector<float>& outBuf = trt.hostBuffer("output0");
  const float* out = outBuf.data();
  const int maxDet = static_cast<int>(outBuf.size()) / stride;
#else
  const auto outTensor = network.output(0);
  const float* out = outTensor.data();
  const int maxDet = static_cast<int>(outTensor.size()) / stride;
#endif

  DECLARE_DEBUG_DRAWING("module:TRTMultiPerceptor:penaltyMark:image", "drawingOnImage");

  float bestConf = penaltyMarkConfidenceThreshold;

  for(int i = 0; i < maxDet; ++i)
  {
    const float conf = out[i * stride + 4];
    const int   cls  = static_cast<int>(std::round(out[i * stride + 5]));

    if(cls != penaltyMarkClassId || conf < bestConf)
      continue;

    const float x1 = out[i * stride + 0];
    const float y1 = out[i * stride + 1];
    const float x2 = out[i * stride + 2];
    const float y2 = out[i * stride + 3];
    const Vector2f centre((x1 + x2) * 0.5f, (y1 + y2) * 0.5f);

    Vector2f posOnField;
    Matrix2f cov;
    if(!theMeasurementCovariance.transformWithCovLegacy(centre, 0.f, Vector2f(0.04f, 0.06f), posOnField, cov))
      continue;

    bestConf = conf;
    penaltyMarkPercept.positionInImage = centre.cast<int>();
    penaltyMarkPercept.positionOnField = posOnField;
    penaltyMarkPercept.covarianceOnField = cov;
    penaltyMarkPercept.wasSeen = true;

    RECTANGLE("module:TRTMultiPerceptor:penaltyMark:image",
              static_cast<int>(x1), static_cast<int>(y1),
              static_cast<int>(x2), static_cast<int>(y2),
              2, Drawings::solidPen, ColorRGBA::magenta);
  }
}

void TRTMultiPerceptor::update(MLCirclePercept& mlCirclePercept)
{
  mlCirclePercept.wasSeen = false;

  if(!apply() || centreCircleClassId < 0)
    return;

  static constexpr int stride = 6;
#ifdef BUILD_BOOSTER
  const std::vector<float>& outBuf = trt.hostBuffer("output0");
  const float* out = outBuf.data();
  const int maxDet = static_cast<int>(outBuf.size()) / stride;
#else
  const auto outTensor = network.output(0);
  const float* out = outTensor.data();
  const int maxDet = static_cast<int>(outTensor.size()) / stride;
#endif

  DECLARE_DEBUG_DRAWING("module:TRTMultiPerceptor:centreCircle:image", "drawingOnImage");

  float bestConf = centreCircleConfidenceThreshold;

  for(int i = 0; i < maxDet; ++i)
  {
    const float conf = out[i * stride + 4];
    const int   cls  = static_cast<int>(std::round(out[i * stride + 5]));

    if(cls != centreCircleClassId || conf < bestConf)
      continue;

    const float x1 = out[i * stride + 0];
    const float y1 = out[i * stride + 1];
    const float x2 = out[i * stride + 2];
    const float y2 = out[i * stride + 3];
    const Vector2f centre((x1 + x2) * 0.5f, (y1 + y2) * 0.5f);

    Vector2f posOnField;
    Matrix2f cov;
    if(!theMeasurementCovariance.transformWithCovLegacy(centre, 0.f, Vector2f(0.04f, 0.06f), posOnField, cov))
      continue;

    bestConf = conf;
    mlCirclePercept.pos = posOnField;
    mlCirclePercept.cov = cov;
    mlCirclePercept.wasSeen = true;

    RECTANGLE("module:TRTMultiPerceptor:centreCircle:image",
              static_cast<int>(x1), static_cast<int>(y1),
              static_cast<int>(x2), static_cast<int>(y2),
              2, Drawings::solidPen, ColorRGBA::blue);
  }
}

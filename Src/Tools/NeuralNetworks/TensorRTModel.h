/**
 * @file TensorRTModel.h
 *
 * Thin wrapper around a serialised TensorRT .engine file.
 * Only compiled when BUILD_BOOSTER is defined (Jetson Orin target).
 *
 * Usage:
 *   TensorRTModel trt;
 *   trt.load("NeuralNets/TRTVision/model.engine");   // path relative to Config/
 *   // fill input: trt.hostBuffer("images")
 *   trt.infer();
 *   // read output: trt.hostBuffer("output0")
 */

#pragma once

#ifdef BUILD_BOOSTER

#include <NvInfer.h>
#include <cuda_runtime_api.h>
#include <cuda_fp16.h>
#include <cstdint>
#include <string>
#include <vector>

/**
 * Owns an IRuntime, ICudaEngine, IExecutionContext, and all GPU/host buffers.
 *
 * hostBuffer() always holds float32 values; FP16 conversion is handled internally
 * during infer() for bindings whose dtype is kHALF.
 *
 * For fixed-shape engines call load() then infer() directly.
 * For dynamic-shape engines call setInputShape() for every input before the
 * first infer(), and again whenever the shape changes.
 */
class TensorRTModel
{
public:
  TensorRTModel() = default;
  ~TensorRTModel();

  TensorRTModel(const TensorRTModel&) = delete;
  TensorRTModel& operator=(const TensorRTModel&) = delete;

  /**
   * Deserialise the engine at Config/<relativeConfigPath>.
   * Logs the TensorRT library version on success.
   * @param relativeConfigPath  e.g. "NeuralNets/TRTVision/model.engine"
   * @return true on success.
   */
  bool load(const std::string& relativeConfigPath);

  bool isLoaded() const { return context != nullptr; }

  /** Returns true if a tensor with this name exists in the engine. */
  bool hasTensor(const std::string& name) const;

  /**
   * Override the shape of a dynamic input tensor.
   * Must be called before the first infer() when the engine uses dynamic shapes.
   * Reallocates host and device buffers if the element count changes.
   * @param name  Tensor name, e.g. "images".
   * @param dims  New dimensions, e.g. {1, 3, 448, 544}.
   * @return true on success, false if the tensor was not found or reallocation failed.
   */
  bool setInputShape(const std::string& name, const std::vector<int>& dims);

  /**
   * Host-side float32 buffer for a named tensor.
   * Fill input buffers before calling infer(); output buffers are populated after.
   */
  std::vector<float>& hostBuffer(const std::string& name);
  const std::vector<float>& hostBuffer(const std::string& name) const;

  /**
   * Copy inputs H→D (with FP32→FP16 conversion where needed), run inference,
   * copy outputs D→H (with FP16→FP32 conversion where needed).
   * Synchronous — returns after a single cudaStreamSynchronize covering all
   * async copies and the enqueue.
   * @return true on success.
   */
  bool infer();

private:
  /** Binding metadata and buffers for one engine input/output tensor. */
  struct Binding
  {
    std::string        name;
    nvinfer1::Dims     dims      = {};
    nvinfer1::DataType dtype     = nvinfer1::DataType::kFLOAT;
    bool               isInput   = false;
    std::size_t        sizeBytes = 0;        ///< bytes on device (dtype-dependent)
    void*              devicePtr = nullptr;
    std::vector<float>         hostBuf;      ///< always float32, element count = volume(dims)
    std::vector<std::uint16_t> pinnedFp16Buf; ///< staging buffer for FP16 bindings
  };

  /** TRT log bridge — forwards to OUTPUT_WARNING / OUTPUT_ERROR. */
  struct Logger : public nvinfer1::ILogger
  {
    void log(nvinfer1::ILogger::Severity severity, const char* msg) noexcept override;
  } logger;

  nvinfer1::IRuntime*          runtime = nullptr;
  nvinfer1::ICudaEngine*       engine  = nullptr;
  nvinfer1::IExecutionContext* context = nullptr;

  std::vector<Binding> bindings;
  cudaStream_t         stream = nullptr;

  /** Allocate / reallocate device and host buffers for binding i, then register the address with the context. */
  bool allocateBinding(std::size_t i);

  int findBindingIndex(const std::string& name) const;

  static std::size_t   volume(const nvinfer1::Dims& dims);
  static std::uint16_t floatToHalf(float v);
  static float         halfToFloat(std::uint16_t v);
};

#endif // BUILD_BOOSTER

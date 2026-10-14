/**
 * @file TensorRTModel.cpp
 * @see TensorRTModel.h
 */

#ifdef BUILD_BOOSTER

#include "TensorRTModel.h"
#include "Platform/BHAssert.h"
#include "Platform/File.h"
#include "Debugging/Debugging.h"

#include <fstream>
#include <cstring>

void TensorRTModel::Logger::log(nvinfer1::ILogger::Severity severity, const char* msg) noexcept
{
  switch(severity)
  {
    case nvinfer1::ILogger::Severity::kERROR:
    case nvinfer1::ILogger::Severity::kINTERNAL_ERROR:
      OUTPUT_ERROR("TensorRT: " << msg);
      break;
    case nvinfer1::ILogger::Severity::kWARNING:
      OUTPUT_WARNING("TensorRT: " << msg);
      break;
    default:
      break;
  }
}

std::size_t TensorRTModel::volume(const nvinfer1::Dims& dims)
{
  std::size_t vol = 1;
  for(int i = 0; i < dims.nbDims; ++i)
  {
    if(dims.d[i] < 0)
      return 0; // dynamic dim — shape not yet set via setInputShape
    vol *= static_cast<std::size_t>(dims.d[i]);
  }
  return vol;
}

std::uint16_t TensorRTModel::floatToHalf(float v)
{
  __half h = __float2half_rn(v);
  std::uint16_t bits;
  static_assert(sizeof(h) == sizeof(bits));
  std::memcpy(&bits, &h, sizeof(bits));
  return bits;
}

float TensorRTModel::halfToFloat(std::uint16_t v)
{
  __half h;
  static_assert(sizeof(h) == sizeof(v));
  std::memcpy(&h, &v, sizeof(h));
  return __half2float(h);
}

bool TensorRTModel::allocateBinding(std::size_t i)
{
  Binding& b = bindings[i];
  const std::size_t nElems = volume(b.dims);
  if(nElems == 0)
    return true; // dynamic shape not yet resolved — defer until setInputShape

  const std::size_t elementSize  = (b.dtype == nvinfer1::DataType::kHALF) ? 2u : 4u;
  const std::size_t newSizeBytes = nElems * elementSize;

  if(b.devicePtr && newSizeBytes != b.sizeBytes)
  {
    cudaFree(b.devicePtr);
    b.devicePtr = nullptr;
  }

  if(!b.devicePtr)
  {
    if(cudaMalloc(&b.devicePtr, newSizeBytes) != cudaSuccess)
    {
      OUTPUT_ERROR("TensorRTModel: cudaMalloc failed for '" << b.name << "'");
      return false;
    }
  }

  b.sizeBytes = newSizeBytes;
  b.hostBuf.assign(nElems, 0.f);
  if(b.dtype == nvinfer1::DataType::kHALF)
    b.pinnedFp16Buf.resize(nElems);

  context->setTensorAddress(b.name.c_str(), b.devicePtr);
  return true;
}

int TensorRTModel::findBindingIndex(const std::string& name) const
{
  for(std::size_t i = 0; i < bindings.size(); ++i)
    if(bindings[i].name == name)
      return static_cast<int>(i);
  return -1;
}

TensorRTModel::~TensorRTModel()
{
  for(auto& b : bindings)
    if(b.devicePtr)
      cudaFree(b.devicePtr);

  if(stream)
    cudaStreamDestroy(stream);

  delete context;
  delete engine;
  delete runtime;
}

bool TensorRTModel::load(const std::string& relativeConfigPath)
{
  const std::string fullPath =
      std::string(File::getBHDir()) + "/Config/" + relativeConfigPath;

  std::ifstream file(fullPath, std::ios::binary | std::ios::ate);
  if(!file.is_open())
  {
    OUTPUT_ERROR("TensorRTModel: cannot open engine file: " << fullPath);
    return false;
  }

  const std::streamsize size = file.tellg();
  file.seekg(0, std::ios::beg);
  std::vector<char> buffer(static_cast<std::size_t>(size));
  if(!file.read(buffer.data(), size))
  {
    OUTPUT_ERROR("TensorRTModel: failed to read engine file: " << fullPath);
    return false;
  }

  runtime = nvinfer1::createInferRuntime(logger);
  if(!runtime)
  {
    OUTPUT_ERROR("TensorRTModel: createInferRuntime failed");
    return false;
  }

  engine = runtime->deserializeCudaEngine(buffer.data(), static_cast<std::size_t>(size));
  if(!engine)
  {
    OUTPUT_ERROR("TensorRTModel: deserializeCudaEngine failed");
    return false;
  }

  context = engine->createExecutionContext();
  if(!context)
  {
    OUTPUT_ERROR("TensorRTModel: createExecutionContext failed");
    return false;
  }

  OUTPUT_WARNING("TensorRTModel: loaded engine '" << relativeConfigPath
      << "' (TensorRT " << NV_TENSORRT_MAJOR << '.' << NV_TENSORRT_MINOR
      << '.' << NV_TENSORRT_PATCH << ')');

  const int nbTensors = engine->getNbIOTensors();
  bindings.resize(static_cast<std::size_t>(nbTensors));

  for(int i = 0; i < nbTensors; ++i)
  {
    Binding& b = bindings[static_cast<std::size_t>(i)];
    b.name     = engine->getIOTensorName(i);
    b.dims     = engine->getTensorShape(b.name.c_str());
    b.dtype    = engine->getTensorDataType(b.name.c_str());
    b.isInput  = (engine->getTensorIOMode(b.name.c_str()) == nvinfer1::TensorIOMode::kINPUT);
    if(!allocateBinding(static_cast<std::size_t>(i)))
      return false;
  }

  if(cudaStreamCreate(&stream) != cudaSuccess)
  {
    OUTPUT_ERROR("TensorRTModel: cudaStreamCreate failed");
    return false;
  }

  return true;
}

bool TensorRTModel::hasTensor(const std::string& name) const
{
  return findBindingIndex(name) >= 0;
}

bool TensorRTModel::setInputShape(const std::string& name, const std::vector<int>& dims)
{
  const int idx = findBindingIndex(name);
  if(idx < 0)
  {
    OUTPUT_ERROR("TensorRTModel: setInputShape — tensor '" << name << "' not found");
    return false;
  }

  nvinfer1::Dims trtDims;
  trtDims.nbDims = static_cast<int>(dims.size());
  for(int i = 0; i < trtDims.nbDims; ++i)
    trtDims.d[i] = dims[static_cast<std::size_t>(i)];

  if(!context->setInputShape(name.c_str(), trtDims))
  {
    OUTPUT_ERROR("TensorRTModel: setInputShape failed for '" << name << "'");
    return false;
  }

  bindings[static_cast<std::size_t>(idx)].dims = trtDims;
  return allocateBinding(static_cast<std::size_t>(idx));
}

std::vector<float>& TensorRTModel::hostBuffer(const std::string& name)
{
  const int idx = findBindingIndex(name);
  ASSERT(idx >= 0);
  return bindings[static_cast<std::size_t>(idx)].hostBuf;
}

const std::vector<float>& TensorRTModel::hostBuffer(const std::string& name) const
{
  const int idx = findBindingIndex(name);
  ASSERT(idx >= 0);
  return bindings[static_cast<std::size_t>(idx)].hostBuf;
}

/* Copies inputs H→D (FP32→FP16 where needed), runs enqueueV3, then copies outputs D→H.
   All work is issued asynchronously on stream; one cudaStreamSynchronize at the end covers everything. */
bool TensorRTModel::infer()
{
  if(!context)
    return false;

  // H → D (inputs)
  for(auto& b : bindings)
  {
    if(!b.isInput)
      continue;

    if(b.dtype == nvinfer1::DataType::kHALF)
    {
      b.pinnedFp16Buf.resize(b.hostBuf.size());
      for(std::size_t j = 0; j < b.hostBuf.size(); ++j)
        b.pinnedFp16Buf[j] = floatToHalf(b.hostBuf[j]);
      if(cudaMemcpyAsync(b.devicePtr, b.pinnedFp16Buf.data(), b.sizeBytes,
                         cudaMemcpyHostToDevice, stream) != cudaSuccess)
      {
        OUTPUT_ERROR("TensorRTModel: input H→D copy failed for '" << b.name << "'");
        return false;
      }
    }
    else
    {
      if(cudaMemcpyAsync(b.devicePtr, b.hostBuf.data(), b.sizeBytes,
                         cudaMemcpyHostToDevice, stream) != cudaSuccess)
      {
        OUTPUT_ERROR("TensorRTModel: input H→D copy failed for '" << b.name << "'");
        return false;
      }
    }
  }

  if(!context->enqueueV3(stream))
  {
    OUTPUT_ERROR("TensorRTModel: enqueueV3 failed");
    return false;
  }

  // D → H (outputs)
  for(auto& b : bindings)
  {
    if(b.isInput)
      continue;

    if(b.dtype == nvinfer1::DataType::kHALF)
    {
      b.pinnedFp16Buf.resize(b.hostBuf.size());
      if(cudaMemcpyAsync(b.pinnedFp16Buf.data(), b.devicePtr, b.sizeBytes,
                         cudaMemcpyDeviceToHost, stream) != cudaSuccess)
      {
        OUTPUT_ERROR("TensorRTModel: output D→H copy failed for '" << b.name << "'");
        return false;
      }
    }
    else
    {
      if(cudaMemcpyAsync(b.hostBuf.data(), b.devicePtr, b.sizeBytes,
                         cudaMemcpyDeviceToHost, stream) != cudaSuccess)
      {
        OUTPUT_ERROR("TensorRTModel: output D→H copy failed for '" << b.name << "'");
        return false;
      }
    }
  }

  if(cudaStreamSynchronize(stream) != cudaSuccess)
  {
    OUTPUT_ERROR("TensorRTModel: cudaStreamSynchronize failed");
    return false;
  }

  for(auto& b : bindings)
  {
    if(b.isInput || b.dtype != nvinfer1::DataType::kHALF)
      continue;
    for(std::size_t j = 0; j < b.hostBuf.size(); ++j)
      b.hostBuf[j] = halfToFloat(b.pinnedFp16Buf[j]);
  }

  return true;
}

#endif // BUILD_BOOSTER

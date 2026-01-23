#include "vertex_detector/image_processor.hpp"
#include <filesystem>

namespace vertex_detector
{

// ============================================================================
// HELPER: Convert ONNX with external weights
// ============================================================================
static std::string convertONNXWithExternalWeights(const std::string& onnx_path)
{
  std::ifstream file(onnx_path, std::ios::binary);
  if (!file) return "";
  
  std::vector<char> header(8192);
  file.read(header.data(), header.size());
  std::string header_str(header.begin(), header.end());
  
  if (header_str.find(".onnx.data") == std::string::npos) {
    std::cout << "[ImageProcessor] Model has embedded weights" << std::endl;
    return "";
  }
  
  file.close();
  
  std::cout << "[ImageProcessor] WARNING: Model uses external weights file" << std::endl;
  std::cout << "[ImageProcessor] You need to use Python to convert it:" << std::endl;
  std::cout << "[ImageProcessor]   python3 -c \"" << std::endl;
  std::cout << "[ImageProcessor]   import onnx" << std::endl;
  std::cout << "[ImageProcessor]   from onnx.external_data_helper import convert_model_to_external_data" << std::endl;
  std::cout << "[ImageProcessor]   model = onnx.load('" << onnx_path << "')" << std::endl;
  std::cout << "[ImageProcessor]   onnx.save(model, '" << onnx_path << ".embedded')" << std::endl;
  std::cout << "[ImageProcessor]   \"" << std::endl;
  
  return "";
}

// ============================================================================
// CTOR
// ============================================================================
ImageProcessor::ImageProcessor(
  const std::string& onnx_path,
  const std::string& engine_path,
  double conf_threshold,
  bool use_fp16)
  : conf_threshold_(conf_threshold)
  , use_fp16_(use_fp16)
  , onnx_path_(onnx_path)
  , engine_path_(engine_path)
{
  channels_cache_.resize(3);
  
  if (!std::filesystem::exists(onnx_path_)) {
    throw std::runtime_error("ONNX model file not found: " + onnx_path_);
  }
  
  std::string data_file = onnx_path_ + ".data";
  if (std::filesystem::exists(data_file)) {
    std::cout << "[ImageProcessor] Found external weights file: " << data_file << std::endl;
  }
  
  loadOrBuildEngine();
  allocateBuffers();
  backend_name_ = use_fp16_ ? "TensorRT FP16" : "TensorRT FP32";
  initialized_ = true;

  std::cout << "[ImageProcessor] ✓ Ready (" << backend_name_ << ")" << std::endl;
}

// ============================================================================
// DTOR
// ============================================================================
ImageProcessor::~ImageProcessor()
{
  if (input_device_) cudaFree(input_device_);
  if (output_device_) cudaFree(output_device_);

  for (void* ptr : extra_buffers_) {
    if (ptr) cudaFree(ptr);
  }
  extra_buffers_.clear();

  delete[] input_host_;
  delete[] output_host_;
  if (stream_) cudaStreamDestroy(stream_);
  if (context_) delete context_;
  if (engine_) delete engine_;
  if (runtime_) delete runtime_;
}

// ============================================================================
// LOAD OR BUILD ENGINE
// ============================================================================
void ImageProcessor::loadOrBuildEngine()
{
  if (std::ifstream(engine_path_).good())
  {
    std::cout << "[ImageProcessor] Loading cached engine..." << std::endl;
    try {
      loadEngine();
      std::cout << "[ImageProcessor] ✓ Engine loaded successfully" << std::endl;
      return;
    } catch (const std::exception& e) {
      std::cout << "[ImageProcessor] Failed to load engine: " << e.what() << std::endl;
      std::cout << "[ImageProcessor] Will rebuild from ONNX..." << std::endl;
      std::filesystem::remove(engine_path_);
    }
  }
  
  std::cout << "[ImageProcessor] Building engine from ONNX..." << std::endl;
  buildEngine();
}

// ============================================================================
// BUILD ENGINE - Z OPTIMIZATION PROFILE
// ============================================================================
void ImageProcessor::buildEngine()
{
  auto builder = nvinfer1::createInferBuilder(logger_);
  if (!builder) {
    throw std::runtime_error("Failed to create TensorRT builder");
  }

  const auto explicitBatch = 1U << static_cast<uint32_t>(
    nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
  auto network = builder->createNetworkV2(explicitBatch);
  if (!network) {
    delete builder;
    throw std::runtime_error("Failed to create TensorRT network");
  }

  auto parser = nvonnxparser::createParser(*network, logger_);
  if (!parser) {
    delete network;
    delete builder;
    throw std::runtime_error("Failed to create ONNX parser");
  }

  std::cout << "[ImageProcessor] Parsing ONNX file: " << onnx_path_ << std::endl;
  
  if (!parser->parseFromFile(
        onnx_path_.c_str(),
        static_cast<int>(nvinfer1::ILogger::Severity::kINFO)))
  {
    std::cerr << "[ImageProcessor] ONNX parsing errors:" << std::endl;
    for (int i = 0; i < parser->getNbErrors(); ++i) {
      std::cerr << "  - " << parser->getError(i)->desc() << std::endl;
    }
    delete parser;
    delete network;
    delete builder;
    throw std::runtime_error("ONNX parse failed");
  }

  std::cout << "[ImageProcessor] ✓ ONNX parsed successfully" << std::endl;

  auto config = builder->createBuilderConfig();
  if (!config) {
    delete parser;
    delete network;
    delete builder;
    throw std::runtime_error("Failed to create builder config");
  }

  config->setMemoryPoolLimit(
    nvinfer1::MemoryPoolType::kWORKSPACE, 1024ULL * 1024 * 1024);

  if (use_fp16_ && builder->platformHasFastFp16()) {
    std::cout << "[ImageProcessor] Enabling FP16 mode" << std::endl;
    config->setFlag(nvinfer1::BuilderFlag::kFP16);
  }

  if (network->getNbInputs() == 0) {
    delete config;
    delete parser;
    delete network;
    delete builder;
    throw std::runtime_error("Network has no inputs");
  }

  auto input = network->getInput(0);
  auto input_dims = input->getDimensions();
  
  std::cout << "[ImageProcessor] Input dimensions: ";
  for (int i = 0; i < input_dims.nbDims; ++i) {
    std::cout << input_dims.d[i];
    if (i < input_dims.nbDims - 1) std::cout << "x";
  }
  std::cout << std::endl;

  bool has_dynamic_shape = false;
  for (int i = 0; i < input_dims.nbDims; ++i) {
    if (input_dims.d[i] == -1) {
      has_dynamic_shape = true;
      break;
    }
  }

  if (has_dynamic_shape) {
    std::cout << "[ImageProcessor] Detected dynamic shapes, creating optimization profile..." << std::endl;
    
    auto profile = builder->createOptimizationProfile();
    
    for (int i = 0; i < network->getNbInputs(); ++i) {
      auto inp = network->getInput(i);
      auto dims = inp->getDimensions();
      
      nvinfer1::Dims min_dims = dims;
      nvinfer1::Dims opt_dims = dims;
      nvinfer1::Dims max_dims = dims;
      
      for (int d = 0; d < dims.nbDims; ++d) {
        if (dims.d[d] == -1) {
          if (d == 0) {
            min_dims.d[d] = 1;
            opt_dims.d[d] = 1;
            max_dims.d[d] = 1;
          } else {
            min_dims.d[d] = 320;
            opt_dims.d[d] = 320;
            max_dims.d[d] = 640;
          }
        }
      }
      
      profile->setDimensions(inp->getName(), 
                            nvinfer1::OptProfileSelector::kMIN, min_dims);
      profile->setDimensions(inp->getName(), 
                            nvinfer1::OptProfileSelector::kOPT, opt_dims);
      profile->setDimensions(inp->getName(), 
                            nvinfer1::OptProfileSelector::kMAX, max_dims);
    }
    
    config->addOptimizationProfile(profile);
    std::cout << "[ImageProcessor] ✓ Optimization profile added" << std::endl;
  }

  // Ustaw wymiary
  input_h_ = input_dims.d[2] > 0 ? input_dims.d[2] : 320;
  input_w_ = input_dims.d[3] > 0 ? input_dims.d[3] : 320;
  
  if (network->getNbOutputs() == 0) {
    delete config;
    delete parser;
    delete network;
    delete builder;
    throw std::runtime_error("Network has no outputs");
  }
  
  auto output_dims = network->getOutput(0)->getDimensions();
  
  if (output_dims.nbDims == 3) {
    output_h_ = output_dims.d[1] > 0 ? output_dims.d[1] : 320;
    output_w_ = output_dims.d[2] > 0 ? output_dims.d[2] : 320;
  } else if (output_dims.nbDims == 4) {
    output_h_ = output_dims.d[2] > 0 ? output_dims.d[2] : 320;
    output_w_ = output_dims.d[3] > 0 ? output_dims.d[3] : 320;
  } else {
    output_h_ = 320;
    output_w_ = 320;
  }

  std::cout << "[ImageProcessor] Network dimensions:" << std::endl;
  std::cout << "  Input: " << input_w_ << "x" << input_h_ << std::endl;
  std::cout << "  Output: " << output_w_ << "x" << output_h_ << std::endl;
  std::cout << "[ImageProcessor] Building engine (this may take a few minutes)..." << std::endl;

  auto serialized = builder->buildSerializedNetwork(*network, *config);
  if (!serialized) {
    delete config;
    delete parser;
    delete network;
    delete builder;
    throw std::runtime_error("Engine build failed");
  }

  std::cout << "[ImageProcessor] ✓ Engine built successfully" << std::endl;
  std::cout << "[ImageProcessor] Saving engine to: " << engine_path_ << std::endl;

  std::ofstream out(engine_path_, std::ios::binary);
  if (!out) {
    delete serialized;
    delete config;
    delete parser;
    delete network;
    delete builder;
    throw std::runtime_error("Failed to open engine file for writing");
  }
  out.write(reinterpret_cast<const char*>(serialized->data()), serialized->size());
  out.close();

  runtime_ = nvinfer1::createInferRuntime(logger_);
  if (!runtime_) {
    delete serialized;
    delete config;
    delete parser;
    delete network;
    delete builder;
    throw std::runtime_error("Failed to create inference runtime");
  }

  engine_ = runtime_->deserializeCudaEngine(serialized->data(), serialized->size());
  if (!engine_) {
    delete runtime_;
    delete serialized;
    delete config;
    delete parser;
    delete network;
    delete builder;
    throw std::runtime_error("Failed to deserialize engine");
  }

  context_ = engine_->createExecutionContext();
  if (!context_) {
    delete engine_;
    delete runtime_;
    delete serialized;
    delete config;
    delete parser;
    delete network;
    delete builder;
    throw std::runtime_error("Failed to create execution context");
  }

  delete serialized;
  delete parser;
  delete config;
  delete network;
  delete builder;

  std::cout << "[ImageProcessor] ✓ Engine built & saved" << std::endl;
}

// ============================================================================
// LOAD ENGINE
// ============================================================================
void ImageProcessor::loadEngine()
{
  std::ifstream file(engine_path_, std::ios::binary);
  if (!file) {
    throw std::runtime_error("Failed to open engine file: " + engine_path_);
  }

  file.seekg(0, std::ifstream::end);
  size_t size = file.tellg();
  file.seekg(0, std::ifstream::beg);

  if (size == 0) {
    throw std::runtime_error("Engine file is empty: " + engine_path_);
  }

  std::vector<char> buffer(size);
  file.read(buffer.data(), size);
  file.close();

  runtime_ = nvinfer1::createInferRuntime(logger_);
  if (!runtime_) {
    throw std::runtime_error("Failed to create inference runtime");
  }

  engine_ = runtime_->deserializeCudaEngine(buffer.data(), size);
  if (!engine_) {
    delete runtime_;
    throw std::runtime_error("Failed to deserialize engine");
  }

  context_ = engine_->createExecutionContext();
  if (!context_) {
    delete engine_;
    delete runtime_;
    throw std::runtime_error("Failed to create execution context");
  }

  std::string input_name = "image";
  std::string output_name = "scores";
  
  auto inDims = engine_->getTensorShape(input_name.c_str());
  auto outDims = engine_->getTensorShape(output_name.c_str());

  input_h_ = inDims.d[2];
  input_w_ = inDims.d[3];
  
  // SuperPoint może mieć output [B, H, W] lub [B, 1, H, W]
  if (outDims.nbDims == 3) {
    output_h_ = outDims.d[1];
    output_w_ = outDims.d[2];
  } else if (outDims.nbDims == 4) {
    output_h_ = outDims.d[2];
    output_w_ = outDims.d[3];
  }

  std::cout << "[ImageProcessor] Engine loaded:" << std::endl;
  std::cout << "  Input tensor: " << input_name << " - " << input_w_ << "x" << input_h_ << std::endl;
  std::cout << "  Output tensor: " << output_name << " - " << output_w_ << "x" << output_h_ << std::endl;
}

// ============================================================================
// BUFFERS 
// ============================================================================
void ImageProcessor::allocateBuffers()
{
  int nbBindings = engine_->getNbIOTensors();
  
  std::cout << "[ImageProcessor] Allocating buffers for " << nbBindings << " IO tensors" << std::endl;
  
  for (int i = 0; i < nbBindings; ++i)
  {
    const char* name = engine_->getIOTensorName(i);
    auto mode = engine_->getTensorIOMode(name);
    auto dims = engine_->getTensorShape(name);
    
    size_t vol = 1;
    for (int d = 0; d < dims.nbDims; ++d) {
      if (dims.d[d] > 0) vol *= dims.d[d];
    }
    size_t size_bytes = vol * sizeof(float);
    
    std::string tensor_name(name);
    
    if (mode == nvinfer1::TensorIOMode::kINPUT) {
      if (tensor_name == "image") {
        input_size_ = size_bytes;
        cudaMalloc(&input_device_, input_size_);
        input_host_ = new float[vol];
        std::cout << "[ImageProcessor]   INPUT allocated: " << size_bytes << " bytes" << std::endl;
      }
    } else {
      if (tensor_name == "scores") {
        output_size_ = size_bytes;
        cudaMalloc(&output_device_, output_size_);
        output_host_ = new float[vol];
        std::cout << "[ImageProcessor]   SCORES allocated: " << size_bytes << " bytes" << std::endl;
      } else {
        void* extra_ptr = nullptr;
        cudaMalloc(&extra_ptr, size_bytes);
        extra_buffers_.push_back(extra_ptr);
        std::cout << "[ImageProcessor]   " << tensor_name << " allocated: " << size_bytes << " bytes" << std::endl;
      }
    }
  }

  cudaStreamCreate(&stream_);
  std::cout << "[ImageProcessor] ✓ All buffers allocated" << std::endl;
}

// ============================================================================
// PREPROCESS - Grayscale dla SuperPoint
// ============================================================================
cv::Mat ImageProcessor::preprocess(const cv::Mat& img)
{
  cv::Mat working_img;
  if (scale_factor_ < 1.0) {
    cv::resize(img, scaled_cache_, cv::Size(), scale_factor_, scale_factor_, cv::INTER_LINEAR);
    working_img = scaled_cache_;
  } else {
    working_img = img;
  }

  cv::resize(working_img, scaled_cache_, {input_w_, input_h_}, 0, 0, cv::INTER_LINEAR);
  
  cv::Mat gray;
  if (scaled_cache_.channels() == 3) {
    cv::cvtColor(scaled_cache_, gray, cv::COLOR_BGR2GRAY);
  } else {
    gray = scaled_cache_;
  }
  
  gray.convertTo(rgb_cache_, CV_32F, 1.0 / 255.0);

  int hw = input_w_ * input_h_;
  memcpy(input_host_, rgb_cache_.data, hw * sizeof(float));

  return rgb_cache_;
}

// ============================================================================
// INFERENCE 
// ============================================================================
std::vector<Vertex> ImageProcessor::processImage(const cv::Mat& image)
{
  preprocess(image);

  cudaMemcpyAsync(input_device_, input_host_, input_size_,
                  cudaMemcpyHostToDevice, stream_);

  int nbBindings = engine_->getNbIOTensors();
  int extra_idx = 0;

  for (int i = 0; i < nbBindings; ++i)
  {
    const char* name = engine_->getIOTensorName(i);
    std::string tensor_name(name);
    auto mode = engine_->getTensorIOMode(name);
    
    if (mode == nvinfer1::TensorIOMode::kINPUT) {
      if (tensor_name == "image") {
        context_->setTensorAddress(name, input_device_);
      }
    } else {
      if (tensor_name == "scores") {
        context_->setTensorAddress(name, output_device_);
      } else {
        if (extra_idx < extra_buffers_.size()) {
          context_->setTensorAddress(name, extra_buffers_[extra_idx++]);
        }
      }
    }
  }

  bool success = context_->enqueueV3(stream_);
  if (!success) {
    throw std::runtime_error("Inference failed");
  }

  cudaMemcpyAsync(output_host_, output_device_, output_size_,
                  cudaMemcpyDeviceToHost, stream_);
  cudaStreamSynchronize(stream_);

  return extractVertices(output_host_, image.size());
}

// ============================================================================
// POSTPROCESS - SuperPoint
// ============================================================================
std::vector<Vertex> ImageProcessor::extractVertices(
  const float* heatmap,
  const cv::Size& orig)
{
  std::vector<Vertex> out;
  out.reserve(max_vertices_);
  
  float effective_width = orig.width * scale_factor_;
  float effective_height = orig.height * scale_factor_;
  
  float sx = effective_width / output_w_;
  float sy = effective_height / output_h_;

  double adjusted_threshold = std::min(conf_threshold_, 0.015);
  
  for (int y = 1; y < output_h_ - 1; ++y)
  {
    for (int x = 1; x < output_w_ - 1; ++x)
    {
      int idx = y * output_w_ + x;
      float v = heatmap[idx];

      if (v < adjusted_threshold) continue;

      // Local Maximum Check
      if (v >= heatmap[idx - output_w_ - 1] && v >= heatmap[idx - output_w_] && 
          v >= heatmap[idx - output_w_ + 1] && v >= heatmap[idx - 1] && 
          v >= heatmap[idx + 1] && v >= heatmap[idx + output_w_ - 1] && 
          v >= heatmap[idx + output_w_] && v >= heatmap[idx + output_w_ + 1])
      {
        // Subpixel Refinement
        float sum_v = 0.0f;
        float weighted_x = 0.0f;
        float weighted_y = 0.0f;

        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            float val = heatmap[(y + dy) * output_w_ + (x + dx)];
            sum_v += val;
            weighted_x += val * (x + dx);
            weighted_y += val * (y + dy);
          }
        }

        float sub_x = weighted_x / sum_v;
        float sub_y = weighted_y / sum_v;

        float final_x = sub_x * sx;
        float final_y = sub_y * sy;
        
        if (scale_factor_ < 1.0) {
          final_x /= scale_factor_;
          final_y /= scale_factor_;
        }
        
        out.push_back({final_x, final_y, v});
        
        if (out.size() >= static_cast<size_t>(max_vertices_)) {
          return out;
        }
      }
    }
  }

  return out;
}

} // namespace vertex_detector
#pragma once

#include <opencv2/opencv.hpp>
#include <NvInfer.h>
#include <NvOnnxParser.h>
#include <cuda_runtime_api.h>
#include "vertex_detector/vertex.hpp"
#include <vector>
#include <string>
#include <fstream>
#include <iostream>

namespace vertex_detector
{

class ImageProcessor
{
public:
  ImageProcessor(
    const std::string& onnx_path,
    const std::string& engine_path,
    double conf_threshold,
    bool use_fp16 = true
  );

  ~ImageProcessor();

  std::vector<Vertex> processImage(const cv::Mat& image);
  bool isInitialized() const { return initialized_; }
  std::string getBackendName() const { return backend_name_; }

  // Nowe metody optymalizacyjne
  void setScaleFactor(double scale) { scale_factor_ = scale; }
  void setMaxVertices(int max_verts) { max_vertices_ = max_verts; }
  double getScaleFactor() const { return scale_factor_; }

private:
  std::vector<void*> extra_buffers_;

  // TensorRT
  nvinfer1::IRuntime* runtime_{nullptr};
  nvinfer1::ICudaEngine* engine_{nullptr};
  nvinfer1::IExecutionContext* context_{nullptr};

  // CUDA
  void* input_device_{nullptr};
  void* output_device_{nullptr};
  float* input_host_{nullptr};
  float* output_host_{nullptr};
  cudaStream_t stream_{};

  // Shapes
  int input_w_{0};
  int input_h_{0};
  int output_w_{0};
  int output_h_{0};

  size_t input_size_{0};
  size_t output_size_{0};

  // Params
  double conf_threshold_;
  bool use_fp16_;
  bool initialized_{false};
  std::string backend_name_;

  // Optymalizacje
  double scale_factor_{1.0};  // Skalowanie rozdzielczości
  int max_vertices_{500};     // Maksymalna liczba wierzchołków

  // Cache dla preprocessing
  mutable cv::Mat scaled_cache_;
  mutable cv::Mat rgb_cache_;
  mutable std::vector<cv::Mat> channels_cache_;

  // Paths
  std::string onnx_path_;
  std::string engine_path_;

  // Internal
  void loadOrBuildEngine();
  void buildEngine();
  void loadEngine();
  void allocateBuffers();

  cv::Mat preprocess(const cv::Mat& image);
  std::vector<Vertex> extractVertices(const float* heatmap, const cv::Size& orig);

  // Logger
  class Logger : public nvinfer1::ILogger
  {
    void log(Severity severity, const char* msg) noexcept override
    {
      if (severity <= Severity::kWARNING)
        std::cout << "[TensorRT] " << msg << std::endl;
    }
  } logger_;
};

} // namespace vertex_detector
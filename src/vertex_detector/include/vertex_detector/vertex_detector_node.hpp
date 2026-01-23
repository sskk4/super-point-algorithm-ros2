#pragma once
#include "vertex_detector/vertex.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <opencv2/opencv.hpp>
#include <memory>
#include <atomic>
#include <chrono>

namespace vertex_detector
{
  class ImageProcessor;
  class NMSFilter;

  class VertexDetectorNode : public rclcpp::Node
  {
  public:
    explicit VertexDetectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~VertexDetectorNode();

  private:
    // === INICJALIZACJA ===
    void initializeParameters();
    void initializeProcessors();
    void initializePublishers();
    void initializeSubscribers();

    // === CALLBACKI ===
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

    // === PUBLIKACJA ===
    void publishDetections(
      const std::vector<Vertex>& vertices,
      const std_msgs::msg::Header& header
    );
    
    void publishDebugVisualization(
      const sensor_msgs::msg::Image::SharedPtr original_msg,
      const std::vector<Vertex>& vertices
    );

    // === OPTYMALIZACJE ===
    void adjustPerformance();
    bool shouldProcessFrame();

    // === KOMPONENTY ===
    std::unique_ptr<ImageProcessor> image_processor_;
    std::unique_ptr<NMSFilter> nms_filter_;

    // === ROS2 INTERFEJSY ===
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detection_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;

    // === PARAMETRY ===
    std::string model_path_;
    double confidence_threshold_;
    int top_k_detections_;
    float nms_radius_;
    bool enable_debug_visualization_;
    std::string camera_frame_id_;
    bool use_gpu_;
    bool use_fp16_;
    
    // === OPTYMALIZACJE WYDAJNOŚCI ===
    int frame_skip_count_{0};
    int frame_skip_interval_{2};           // Przetwarzaj co N-tą klatkę
    double scale_factor_{1.0};             // Skalowanie rozdzielczości
    bool auto_adjust_quality_{false};      // Automatyczne dostosowanie jakości
    double target_fps_{30.0};              // Docelowa prędkość przetwarzania
    int max_vertices_{500};                // Maksymalna liczba wierzchołków
    bool use_roi_{false};                  // Czy używać ROI
    cv::Rect roi_rect_;                    // Region of Interest
    
    // === DANE ===
    sensor_msgs::msg::CameraInfo::SharedPtr latest_camera_info_;
    
    // === STATYSTYKI I MONITORING ===
    std::atomic<uint64_t> frames_processed_{0};
    std::atomic<uint64_t> total_vertices_detected_{0};
    std::atomic<uint64_t> frames_skipped_{0};
    
    std::chrono::high_resolution_clock::time_point last_adjustment_time_;
    std::vector<double> processing_times_;  
    const size_t PERF_BUFFER_SIZE = 30;
    
    mutable cv::Mat image_cache_;
  };

} // namespace vertex_detector
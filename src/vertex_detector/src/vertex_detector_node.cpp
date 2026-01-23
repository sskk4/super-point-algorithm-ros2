#include "vertex_detector/vertex_detector_node.hpp"
#include "vertex_detector/image_processor.hpp"
#include "vertex_detector/nms_filter.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace vertex_detector
{

// ============================================================================
// KONSTRUKTOR 
// ============================================================================
VertexDetectorNode::VertexDetectorNode(const rclcpp::NodeOptions & options) 
  : Node("vertex_detector", options),
    frames_processed_(0),
    total_vertices_detected_(0),
    frames_skipped_(0)
{
  RCLCPP_INFO(get_logger(), "Starting Vertex Detector Node...");

  last_adjustment_time_ = std::chrono::high_resolution_clock::now();
  processing_times_.reserve(PERF_BUFFER_SIZE);

  initializeParameters();
  initializeProcessors();
  initializePublishers();
  initializeSubscribers();

  RCLCPP_INFO(get_logger(), "✓ Vertex Detector Node ready!");
}

VertexDetectorNode::~VertexDetectorNode()
{
  RCLCPP_INFO(get_logger(), 
    "Shutting down. Processed %lu frames, skipped %lu, detected %lu vertices total",
    frames_processed_.load(), frames_skipped_.load(), total_vertices_detected_.load());
}

// ============================================================================
// INICJALIZACJA PARAMETRÓW
// ============================================================================
void VertexDetectorNode::initializeParameters()
{
  declare_parameter("model_path", "models/vertex_net.onnx");
  declare_parameter("confidence_threshold", 0.2);
  declare_parameter("top_k", 200);
  declare_parameter("nms_radius", 20.0);
  declare_parameter("enable_debug_visualization", false);
  declare_parameter("use_fp16", false);
  
  // Nowe parametry optymalizacyjne
  declare_parameter("frame_skip_interval", 2);
  declare_parameter("scale_factor", 1.0);
  declare_parameter("auto_adjust_quality", false);
  declare_parameter("target_fps", 30.0);
  declare_parameter("max_vertices", 500);
  declare_parameter("use_roi", false);
  declare_parameter("roi_x", 0);
  declare_parameter("roi_y", 0);
  declare_parameter("roi_width", 640);
  declare_parameter("roi_height", 480);

  use_fp16_ = get_parameter("use_fp16").as_bool();
  model_path_ = get_parameter("model_path").as_string();
  confidence_threshold_ = get_parameter("confidence_threshold").as_double();
  top_k_detections_ = get_parameter("top_k").as_int();
  nms_radius_ = get_parameter("nms_radius").as_double();
  enable_debug_visualization_ = get_parameter("enable_debug_visualization").as_bool();
  
  frame_skip_interval_ = get_parameter("frame_skip_interval").as_int();
  scale_factor_ = get_parameter("scale_factor").as_double();
  auto_adjust_quality_ = get_parameter("auto_adjust_quality").as_bool();
  target_fps_ = get_parameter("target_fps").as_double();
  max_vertices_ = get_parameter("max_vertices").as_int();
  use_roi_ = get_parameter("use_roi").as_bool();
  
  if (use_roi_) {
    roi_rect_.x = get_parameter("roi_x").as_int();
    roi_rect_.y = get_parameter("roi_y").as_int();
    roi_rect_.width = get_parameter("roi_width").as_int();
    roi_rect_.height = get_parameter("roi_height").as_int();
  }

  RCLCPP_INFO(get_logger(), "Parameters loaded:");
  RCLCPP_INFO(get_logger(), "  Model: %s", model_path_.c_str());
  RCLCPP_INFO(get_logger(), "  Confidence: %.2f", confidence_threshold_);
  RCLCPP_INFO(get_logger(), "  Top-K: %d", top_k_detections_);
  RCLCPP_INFO(get_logger(), "  NMS Radius: %.1f px", nms_radius_);
  RCLCPP_INFO(get_logger(), "  Frame Skip: every %d frames", frame_skip_interval_);
  RCLCPP_INFO(get_logger(), "  Scale Factor: %.2f", scale_factor_);
  RCLCPP_INFO(get_logger(), "  Max Vertices: %d", max_vertices_);
  RCLCPP_INFO(get_logger(), "  Auto Adjust Quality: %s", auto_adjust_quality_ ? "yes" : "no");
  if (use_roi_) {
    RCLCPP_INFO(get_logger(), "  ROI: [%d, %d, %d, %d]", 
      roi_rect_.x, roi_rect_.y, roi_rect_.width, roi_rect_.height);
  }
}

// ============================================================================
// INICJALIZACJA PROCESORÓW
// ============================================================================
void VertexDetectorNode::initializeProcessors()
{
  std::string engine_path =
    model_path_.substr(0, model_path_.find_last_of('.')) + ".engine";

  image_processor_ = std::make_unique<ImageProcessor>(
    model_path_,
    engine_path,
    confidence_threshold_,
    false
  );

  if (!image_processor_ || !image_processor_->isInitialized()) {
    RCLCPP_FATAL(get_logger(), "Failed to initialize ImageProcessor!");
    throw std::runtime_error("ImageProcessor initialization failed");
  }

  // Ustaw parametry optymalizacji
  image_processor_->setScaleFactor(scale_factor_);
  image_processor_->setMaxVertices(max_vertices_);

  nms_filter_ = std::make_unique<NMSFilter>(nms_radius_);

  RCLCPP_INFO(
    get_logger(),
    "✓ Processors initialized using backend: %s",
    image_processor_->getBackendName().c_str()
  );
}

// ============================================================================
// INICJALIZACJA PUBLISHERÓW
// ============================================================================
void VertexDetectorNode::initializePublishers()
{
  detection_pub_ = create_publisher<vision_msgs::msg::Detection2DArray>(
    "vertex_detections", 
    rclcpp::QoS(10).reliable()
  );

  if (enable_debug_visualization_) {
    debug_pub_ = create_publisher<sensor_msgs::msg::Image>(
      "vertex_debug_image",
      10);
    RCLCPP_INFO(get_logger(), "✓ Debug visualization enabled");
  }

  RCLCPP_INFO(get_logger(), "✓ Publishers initialized");
}

// ============================================================================
// INICJALIZACJA SUBSCRIBERA
// ============================================================================
void VertexDetectorNode::initializeSubscribers()
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    "camera/image_raw",
    qos,
    std::bind(&VertexDetectorNode::imageCallback, this, std::placeholders::_1)
  );

  RCLCPP_INFO(get_logger(), "✓ Subscribed to camera/image_raw");
}

// ============================================================================
// SPRAWDZENIE CZY PRZETWARZAĆ KLATKĘ
// ============================================================================
bool VertexDetectorNode::shouldProcessFrame()
{
  if (++frame_skip_count_ % (frame_skip_interval_ + 1) != 0) {
    frames_skipped_++;
    return false;
  }
  return true;
}

// ============================================================================
// AUTOMATYCZNE DOSTOSOWANIE WYDAJNOŚCI
// ============================================================================
void VertexDetectorNode::adjustPerformance()
{
  if (!auto_adjust_quality_ || processing_times_.size() < PERF_BUFFER_SIZE) {
    return;
  }

  // Oblicz średni czas przetwarzania
  double avg_time = 0.0;
  for (double t : processing_times_) {
    avg_time += t;
  }
  avg_time /= processing_times_.size();

  double target_time = 1000.0 / target_fps_; // ms

  // Jeśli przetwarzanie trwa zbyt długo
  if (avg_time > target_time * 1.2) {
    // Zwiększ frame skip
    if (frame_skip_interval_ < 5) {
      frame_skip_interval_++;
      RCLCPP_INFO(get_logger(), 
        "Performance adjustment: frame_skip_interval increased to %d", 
        frame_skip_interval_);
    }
    // Lub zmniejsz skalę
    else if (scale_factor_ > 0.5) {
      scale_factor_ *= 0.9;
      image_processor_->setScaleFactor(scale_factor_);
      RCLCPP_INFO(get_logger(), 
        "Performance adjustment: scale_factor decreased to %.2f", 
        scale_factor_);
    }
  }
  // Jeśli mamy zapas wydajności
  else if (avg_time < target_time * 0.7 && frame_skip_interval_ > 0) {
    frame_skip_interval_--;
    RCLCPP_INFO(get_logger(), 
      "Performance adjustment: frame_skip_interval decreased to %d", 
      frame_skip_interval_);
  }

  processing_times_.clear();
}

// ============================================================================
// GŁÓWNY CALLBACK
// ============================================================================
void VertexDetectorNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  // Frame skipping
  if (!shouldProcessFrame()) {
    return;
  }

  auto start = std::chrono::high_resolution_clock::now();

  try {
    // 1. Konwersja ROS Image → OpenCV Mat
    cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

    // 2. ROI jeśli włączone
    cv::Mat working_image = image;
    if (use_roi_ && roi_rect_.width > 0 && roi_rect_.height > 0) {
      // Sprawdź czy ROI mieści się w obrazie
      if (roi_rect_.x + roi_rect_.width <= image.cols &&
          roi_rect_.y + roi_rect_.height <= image.rows) {
        working_image = image(roi_rect_);
      }
    }

    // 3. Przetwarzanie przez CNN
    std::vector<Vertex> vertices = image_processor_->processImage(working_image);

    // 4. Przesunięcie współrzędnych jeśli użyto ROI
    if (use_roi_ && working_image.data != image.data) {
      for (auto& v : vertices) {
        v.x += roi_rect_.x;
        v.y += roi_rect_.y;
      }
    }

    // 5. Filtrowanie duplikatów (NMS) - używamy szybszej metody
    vertices = nms_filter_->filterGridBased(vertices);

    // 6. Wybór top-K najlepszych
    vertices = nms_filter_->selectTopK(vertices, top_k_detections_);

    // 7. Publikacja wyników
    publishDetections(vertices, msg->header);

    // 8. Opcjonalna wizualizacja
    if (enable_debug_visualization_) {
      publishDebugVisualization(msg, vertices);
    }

    frames_processed_++;
    total_vertices_detected_ += vertices.size();

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    
    // Zbieranie statystyk wydajności
    processing_times_.push_back(duration.count());
    if (processing_times_.size() >= PERF_BUFFER_SIZE) {
      adjustPerformance();
    }

    if (frames_processed_ % 100 == 0) {
      double avg_vertices = static_cast<double>(total_vertices_detected_) / frames_processed_;
      RCLCPP_INFO(get_logger(), 
        "Stats: %lu frames processed, %lu skipped, avg %.1f vertices/frame, last frame: %ld ms",
        frames_processed_.load(), frames_skipped_.load(), 
        avg_vertices, static_cast<long>(duration.count())
      );
    }

  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Processing error: %s", e.what());
  }
}

// ============================================================================
// PUBLIKACJA DETEKCJI
// ============================================================================
void VertexDetectorNode::publishDetections(
  const std::vector<Vertex>& vertices,
  const std_msgs::msg::Header& header)
{
  vision_msgs::msg::Detection2DArray msg;
  msg.header = header;
  msg.detections.reserve(vertices.size());

  for (const auto& v : vertices) {
    vision_msgs::msg::Detection2D det;
    det.bbox.center.position.x = v.x;
    det.bbox.center.position.y = v.y;
    det.bbox.size_x = 3.0;
    det.bbox.size_y = 3.0;
    
    vision_msgs::msg::ObjectHypothesisWithPose hyp;
    hyp.hypothesis.score = v.score;
    det.results.push_back(hyp);

    msg.detections.push_back(det);
  }

  detection_pub_->publish(msg);
}

// ============================================================================
// PUBLIKACJA WIZUALIZACJI DEBUG
// ============================================================================
void VertexDetectorNode::publishDebugVisualization(
  const sensor_msgs::msg::Image::SharedPtr original_msg,
  const std::vector<Vertex>& vertices)
{
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(original_msg, "bgr8");

  // Rysuj ROI jeśli włączone
  if (use_roi_) {
    cv::rectangle(cv_ptr->image, roi_rect_, cv::Scalar(255, 255, 0), 2);
  }

  // Rysuj wierzchołki
  for (const auto& v : vertices) {
    cv::circle(cv_ptr->image, cv::Point(v.x, v.y), 4, cv::Scalar(0, 255, 0), -1);
    
    std::string score_text = std::to_string(static_cast<int>(v.score * 100));
    cv::putText(cv_ptr->image, score_text, 
                cv::Point(v.x + 5, v.y - 5),
                cv::FONT_HERSHEY_SIMPLEX, 0.3, 
                cv::Scalar(0, 255, 0), 1);
  }

  // Dodaj informacje o parametrach
  std::string info = "Skip:" + std::to_string(frame_skip_interval_) + 
                     " Scale:" + std::to_string(static_cast<int>(scale_factor_ * 100)) + "%";
  cv::putText(cv_ptr->image, info, cv::Point(10, 20),
              cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);

  debug_pub_->publish(*cv_ptr->toImageMsg());
}

} // namespace vertex_detector

// Rejestracja jako komponent ROS 2
RCLCPP_COMPONENTS_REGISTER_NODE(vertex_detector::VertexDetectorNode)

// Punkt wejścia dla wersji standalone
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions();
  
  try {
    auto node = std::make_shared<vertex_detector::VertexDetectorNode>(options);
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("vertex_detector"), "Fatal error: %s", e.what());
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
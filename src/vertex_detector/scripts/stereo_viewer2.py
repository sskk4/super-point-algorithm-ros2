#!/usr/bin/env python3
"""
Stereo Vision Analyzer with Advanced Computer Vision Analysis
Version: 2.1 - Fixed dimension issues
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import psutil
import os
from dataclasses import dataclass
from typing import Tuple, List, Optional, Dict
from enum import Enum
import threading
from queue import Queue
import json
from datetime import datetime
import statistics

class ViewMode(Enum):
    STEREO = "stereo"
    DEPTH = "depth"
    ANNOTATED = "annotated"
    OVERLAY = "overlay"
    FEATURE_POINTS = "feature_points"
    DISPARITY_MAP = "disparity_map"

@dataclass
class FeaturePoint:
    x: int
    y: int
    confidence: float = 0.0
    depth: float = 0.0
    timestamp: float = 0.0

@dataclass
class SystemMetrics:
    cpu_percent: float
    memory_percent: float
    gpu_load: float
    cpu_temp: float
    gpu_temp: float
    fps: float
    latency_ms: float
    frame_width: int = 0
    frame_height: int = 0

class StereoAnalyzer(Node):
    """Advanced Stereo Vision Analyzer with Computer Vision Metrics"""
    
    # Constants
    MIN_VERTICAL_TOLERANCE = 8
    DISP_SCALE = 4
    MAX_DISPARITY = 200
    STATUS_BAR_HEIGHT = 60
    WINDOW_NAME = 'Stereo Vision Analyzer v2.1'
    METRICS_HISTORY_SIZE = 100
    DISPLAY_WIDTH = 1280  # Fixed display width
    
    # Color codes
    COLOR_MATCH = (0, 255, 0)
    COLOR_ERROR = (0, 0, 255)
    COLOR_INFO = (255, 255, 0)
    COLOR_TELEMETRY = (200, 200, 255)
    COLOR_RECORD = (0, 165, 255)  # Orange for recording
    
    def __init__(self):
        super().__init__('stereo_analyzer')
        self.bridge = CvBridge()
        
        # Image buffers
        self.cam0_img = None
        self.cam1_img = None
        self.last_cam0_time = 0
        self.last_cam1_time = 0
        self.original_size = (0, 0)  # Store original image size
        
        # Processing parameters
        self.show_matches = True
        self.view_mode = ViewMode.STEREO
        self.auto_tune = True
        self.record_data = False
        self.save_interval = 100  # frames
        
        # Performance tracking
        self.frame_count = 0
        self.fps_history = []
        self.processing_times = []
        self.last_time = time.time()
        self.fps = 0.0
        
        # Analysis data
        self.feature_points: List[FeaturePoint] = []
        self.disparity_map = None
        self.depth_map = None
        self.match_accuracy = 0.0
        self.reprojection_error = 0.0
        
        # System paths (Jetson specific)
        self.gpu_load_path = "/sys/devices/platform/17000000.gpu/load"
        self.temp_cpu_path = "/sys/class/thermal/thermal_zone0/temp"
        self.temp_gpu_path = "/sys/class/thermal/thermal_zone1/temp"
        
        # Data logging
        self.log_queue = Queue()
        self.log_thread = threading.Thread(target=self._log_worker)
        self.log_thread.daemon = True
        self.log_thread.start()
        
        # Metrics history
        self.metrics_history: List[SystemMetrics] = []
        
        # Feature detectors (using ORB for better performance)
        self.detector = cv2.ORB_create(nfeatures=1000)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        
        # Camera calibration (should be loaded from file in production)
        self.camera_matrix = np.array([[1000, 0, 640], [0, 1000, 360], [0, 0, 1]])
        self.baseline = 0.12  # meters
        
        # Initialize window
        cv2.namedWindow(self.WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.WINDOW_NAME, self.DISPLAY_WIDTH, 720)
        
        # Create subscriptions
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.sub_cam0 = self.create_subscription(
            Image, '/cam0/vertex_debug_image', 
            self.cam0_callback, qos
        )
        self.sub_cam1 = self.create_subscription(
            Image, '/cam1/vertex_debug_image',
            self.cam1_callback, qos
        )
        
        # Timer for display and analysis
        self.timer = self.create_timer(0.033, self.display_callback)
        
        # Keyboard control info
        self.get_logger().info('Stereo Analyzer Online')
        self.get_logger().info('Controls:')
        self.get_logger().info('  [M] - Toggle matches')
        self.get_logger().info('  [V] - Cycle view modes')
        self.get_logger().info('  [A] - Auto-tune parameters')
        self.get_logger().info('  [R] - Start/stop recording')
        self.get_logger().info('  [S] - Save current frame')
        self.get_logger().info('  [+/-] - Adjust disparity range')
        self.get_logger().info('  [Q] - Quit')

    def _log_worker(self):
        """Background worker for logging data"""
        while True:
            data = self.log_queue.get()
            if data is None:
                break
            self._save_log_entry(data)
            self.log_queue.task_done()

    def get_system_metrics(self) -> SystemMetrics:
        """Collect comprehensive system metrics"""
        try:
            cpu_percent = psutil.cpu_percent()
            memory_percent = psutil.virtual_memory().percent
            
            # Jetson specific metrics
            gpu_load = self._read_file(self.gpu_load_path, scale=0.1)
            cpu_temp = self._read_file(self.temp_cpu_path, scale=0.001)
            gpu_temp = self._read_file(self.temp_gpu_path, scale=0.001)
            
            # Calculate latency
            current_time = time.time()
            latency = (current_time - min(self.last_cam0_time, self.last_cam1_time)) * 1000
            
            return SystemMetrics(
                cpu_percent=cpu_percent,
                memory_percent=memory_percent,
                gpu_load=gpu_load,
                cpu_temp=cpu_temp,
                gpu_temp=gpu_temp,
                fps=self.fps,
                latency_ms=latency,
                frame_width=self.original_size[0],
                frame_height=self.original_size[1]
            )
        except Exception as e:
            self.get_logger().warn(f"Error reading metrics: {e}")
            return SystemMetrics(0, 0, 0, 0, 0, 0, 0, 0, 0)

    def _read_file(self, path: str, scale: float = 1.0) -> float:
        """Read value from system file with error handling"""
        try:
            with open(path, 'r') as f:
                return float(f.read().strip()) * scale
        except:
            return 0.0

    def cam0_callback(self, msg):
        """Callback for camera 0"""
        self.cam0_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.last_cam0_time = time.time()
        if self.cam0_img is not None:
            self.original_size = self.cam0_img.shape[:2]

    def cam1_callback(self, msg):
        """Callback for camera 1"""
        self.cam1_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.last_cam1_time = time.time()

    def detect_and_match_features(self, img0, img1) -> Tuple[List[cv2.KeyPoint], List[cv2.KeyPoint], List[cv2.DMatch]]:
        """Detect and match features between images"""
        gray0 = cv2.cvtColor(img0, cv2.COLOR_BGR2GRAY)
        gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
        
        # Detect keypoints and descriptors
        kp0, desc0 = self.detector.detectAndCompute(gray0, None)
        kp1, desc1 = self.detector.detectAndCompute(gray1, None)
        
        if desc0 is None or desc1 is None:
            return [], [], []
        
        # Match features
        matches = self.matcher.match(desc0, desc1)
        
        # Sort matches by distance
        matches = sorted(matches, key=lambda x: x.distance)
        
        # Apply ratio test for better matches
        good_matches = []
        for match in matches[:50]:  # Limit to top 50 matches
            if match.distance < 50:  # Distance threshold for ORB
                good_matches.append(match)
        
        return kp0, kp1, good_matches

    def compute_disparity_map(self, img0, img1):
        """Compute disparity map from stereo images"""
        gray0 = cv2.cvtColor(img0, cv2.COLOR_BGR2GRAY)
        gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
        
        # Stereo block matching
        stereo = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=16 * 4,  # Reduced for performance
            blockSize=5,  # Smaller block size
            P1=8 * 3 * 5 ** 2,
            P2=32 * 3 * 5 ** 2,
            disp12MaxDiff=1,
            uniquenessRatio=5,
            speckleWindowSize=50,
            speckleRange=16
        )
        
        disparity = stereo.compute(gray0, gray1).astype(np.float32) / 16.0
        
        # Post-processing
        disparity = cv2.medianBlur(disparity, 3)
        
        return disparity

    def compute_depth_from_disparity(self, disparity):
        """Convert disparity map to depth map"""
        # Avoid division by zero
        disparity = np.where(disparity == 0, 0.1, disparity)
        
        # Depth = (focal_length * baseline) / disparity
        focal_length = self.camera_matrix[0, 0]
        depth = (focal_length * self.baseline) / disparity
        
        # Clip depth to reasonable range
        depth = np.clip(depth, 0.1, 50.0)
        
        return depth

    def analyze_matches(self, img0, img1):
        """Advanced match analysis with accuracy metrics"""
        green_mask0 = cv2.inRange(img0, (0, 200, 0), (100, 255, 100))
        green_mask1 = cv2.inRange(img1, (0, 200, 0), (100, 255, 100))
        
        contours0, _ = cv2.findContours(green_mask0, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours1, _ = cv2.findContours(green_mask1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        centers0 = []
        centers1 = []
        
        for contour in contours0:
            if cv2.contourArea(contour) > 10:
                M = cv2.moments(contour)
                if M["m00"] > 0:
                    centers0.append((
                        int(M["m10"] / M["m00"]),
                        int(M["m01"] / M["m00"]),
                        cv2.contourArea(contour)
                    ))
        
        for contour in contours1:
            if cv2.contourArea(contour) > 10:
                M = cv2.moments(contour)
                if M["m00"] > 0:
                    centers1.append((
                        int(M["m10"] / M["m00"]),
                        int(M["m01"] / M["m00"]),
                        cv2.contourArea(contour)
                    ))
        
        # Match centers based on vertical position and area
        matched_pairs = []
        total_error = 0
        valid_matches = 0
        
        for c0 in centers0:
            best_match = None
            min_y_diff = self.MIN_VERTICAL_TOLERANCE
            
            for c1 in centers1:
                y_diff = abs(c0[1] - c1[1])
                area_ratio = min(c0[2], c1[2]) / max(c0[2], c1[2])
                
                if y_diff < min_y_diff and area_ratio > 0.7:
                    best_match = c1
                    min_y_diff = y_diff
            
            if best_match:
                disparity = c0[0] - best_match[0]
                if 0 < disparity < self.MAX_DISPARITY:
                    matched_pairs.append((c0, best_match, disparity))
                    total_error += min_y_diff
                    valid_matches += 1
        
        # Calculate accuracy metrics
        if valid_matches > 0:
            self.match_accuracy = (valid_matches / max(len(centers0), len(centers1))) * 100
            self.reprojection_error = total_error / valid_matches
        else:
            self.match_accuracy = 0
            self.reprojection_error = 0
        
        return matched_pairs

    def create_status_overlay(self, metrics: SystemMetrics) -> np.ndarray:
        """Create comprehensive status overlay with dynamic width"""
        overlay_width = self.DISPLAY_WIDTH
        overlay = np.zeros((self.STATUS_BAR_HEIGHT, overlay_width, 3), dtype=np.uint8)
        
        # System metrics
        info_lines = [
            f"FPS: {self.fps:.1f} | Matches: {self.match_accuracy:.1f}% | Error: {self.reprojection_error:.2f}px",
            f"CPU: {metrics.cpu_percent:.1f}% | RAM: {metrics.memory_percent:.1f}% | GPU: {metrics.gpu_load:.1f}%",
            f"Temp: CPU={metrics.cpu_temp:.1f}°C | GPU={metrics.gpu_temp:.1f}°C | Latency: {metrics.latency_ms:.1f}ms",
            f"Mode: {self.view_mode.value} | Frame: {self.frame_count} | Size: {metrics.frame_width}x{metrics.frame_height} {'| REC' if self.record_data else ''}"
        ]
        
        for i, line in enumerate(info_lines):
            y_pos = 15 + i * 15
            color = self.COLOR_RECORD if "REC" in line else self.COLOR_TELEMETRY
            cv2.putText(overlay, line, (10, y_pos), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
        
        # Add color-coded indicators
        indicator_x = overlay_width - 20
        if metrics.cpu_temp > 80:
            cv2.circle(overlay, (indicator_x, 15), 5, (0, 0, 255), -1)
        elif metrics.cpu_temp > 60:
            cv2.circle(overlay, (indicator_x, 15), 5, (0, 255, 255), -1)
        else:
            cv2.circle(overlay, (indicator_x, 15), 5, (0, 255, 0), -1)
        
        return overlay

    def resize_to_display(self, image: np.ndarray) -> np.ndarray:
        """Resize image to fit display width while maintaining aspect ratio"""
        if image is None:
            return None
            
        height, width = image.shape[:2]
        if width == self.DISPLAY_WIDTH:
            return image
            
        scale = self.DISPLAY_WIDTH / width
        new_height = int(height * scale)
        
        return cv2.resize(image, (self.DISPLAY_WIDTH, new_height))

    def process_frame(self) -> Optional[np.ndarray]:
        """Process stereo frame based on current view mode"""
        if self.cam0_img is None or self.cam1_img is None:
            return None
        
        start_time = time.time()
        
        # Get original images
        img0 = self.cam0_img.copy()
        img1 = self.cam1_img.copy()
        
        output = None
        
        if self.view_mode == ViewMode.STEREO:
            output = self._create_stereo_view(img0, img1)
        elif self.view_mode == ViewMode.DEPTH:
            output = self._create_depth_view(img0, img1)
        elif self.view_mode == ViewMode.ANNOTATED:
            output = self._create_annotated_view(img0, img1)
        elif self.view_mode == ViewMode.FEATURE_POINTS:
            output = self._create_feature_view(img0, img1)
        elif self.view_mode == ViewMode.DISPARITY_MAP:
            output = self._create_disparity_view(img0, img1)
        else:  # OVERLAY
            output = self._create_overlay_view(img0, img1)
        
        # Resize for display
        if output is not None:
            output = self.resize_to_display(output)
        
        # Track processing time
        proc_time = (time.time() - start_time) * 1000
        self.processing_times.append(proc_time)
        if len(self.processing_times) > 100:
            self.processing_times.pop(0)
        
        return output

    def _create_stereo_view(self, img0, img1) -> np.ndarray:
        """Create side-by-side stereo view"""
        # Resize images to same height if needed
        h0, w0 = img0.shape[:2]
        h1, w1 = img1.shape[:2]
        
        if h0 != h1:
            target_height = min(h0, h1)
            img0 = cv2.resize(img0, (w0, target_height))
            img1 = cv2.resize(img1, (w1, target_height))
        
        return np.hstack([img0, img1])

    def _create_depth_view(self, img0, img1) -> np.ndarray:
        """Create depth visualization"""
        # Use smaller images for disparity computation for performance
        scale = 0.5
        small0 = cv2.resize(img0, (0, 0), fx=scale, fy=scale)
        small1 = cv2.resize(img1, (0, 0), fx=scale, fy=scale)
        
        disparity = self.compute_disparity_map(small0, small1)
        depth = self.compute_depth_from_disparity(disparity)
        
        # Resize back to original size
        depth = cv2.resize(depth, (img0.shape[1], img0.shape[0]))
        
        # Normalize for visualization
        depth_norm = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX)
        depth_colored = cv2.applyColorMap(depth_norm.astype(np.uint8), cv2.COLORMAP_JET)
        
        return depth_colored

    def _create_annotated_view(self, img0, img1) -> np.ndarray:
        """Create view with annotated matches"""
        combined = self._create_stereo_view(img0, img1)
        
        if self.show_matches:
            # Use smaller images for analysis
            scale = 0.5
            small0 = cv2.resize(img0, (0, 0), fx=scale, fy=scale)
            small1 = cv2.resize(img1, (0, 0), fx=scale, fy=scale)
            
            matches = self.analyze_matches(small0, small1)
            offset_x = small0.shape[1]
            
            for c0, c1, disparity in matches:
                # Scale coordinates back to original size
                x1, y1 = int(c0[0] / scale), int(c0[1] / scale)
                x2, y2 = int(c1[0] / scale) + offset_x, int(c1[1] / scale)
                
                color_intensity = int(255 * (1 - disparity / self.MAX_DISPARITY))
                color = (0, color_intensity, 255 - color_intensity)
                
                cv2.line(combined, (x1, y1), (x2, y2), color, 1)
                cv2.circle(combined, (x1, y1), 3, self.COLOR_MATCH, -1)
                cv2.circle(combined, (x2, y2), 3, self.COLOR_MATCH, -1)
                
                # Add disparity text
                cv2.putText(combined, f'{disparity:.0f}', (x1 - 10, y1 - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.3, self.COLOR_INFO, 1)
        
        return combined

    def _create_feature_view(self, img0, img1) -> np.ndarray:
        """Show feature points detection"""
        kp0, kp1, matches = self.detect_and_match_features(img0, img1)
        
        # Draw keypoints
        img0_kp = cv2.drawKeypoints(img0, kp0, None, color=self.COLOR_MATCH)
        img1_kp = cv2.drawKeypoints(img1, kp1, None, color=self.COLOR_MATCH)
        
        # Draw matches
        matched_img = cv2.drawMatches(img0_kp, kp0, img1_kp, kp1, 
                                     matches[:20], None, 
                                     matchColor=self.COLOR_MATCH,
                                     singlePointColor=self.COLOR_INFO)
        
        # Add match count
        cv2.putText(matched_img, f'Matches: {len(matches)}', (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, self.COLOR_INFO, 2)
        
        return matched_img

    def _create_disparity_view(self, img0, img1) -> np.ndarray:
        """Show disparity map visualization"""
        # Use smaller images for disparity computation
        scale = 0.5
        small0 = cv2.resize(img0, (0, 0), fx=scale, fy=scale)
        small1 = cv2.resize(img1, (0, 0), fx=scale, fy=scale)
        
        disparity = self.compute_disparity_map(small0, small1)
        
        # Resize back to original size
        disparity = cv2.resize(disparity, (img0.shape[1], img0.shape[0]))
        
        # Normalize and colorize
        disparity_norm = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX)
        disparity_colored = cv2.applyColorMap(disparity_norm.astype(np.uint8), cv2.COLORMAP_JET)
        
        # Overlay on original image
        overlay = cv2.addWeighted(img0, 0.6, disparity_colored, 0.4, 0)
        
        # Add disparity statistics
        valid_disp = disparity[disparity > 0]
        if len(valid_disp) > 0:
            mean_disp = np.mean(valid_disp)
            std_disp = np.std(valid_disp)
            
            stats_text = f'Mean: {mean_disp:.1f}, Std: {std_disp:.1f}'
            cv2.putText(overlay, stats_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.COLOR_INFO, 2)
        
        return overlay

    def _create_overlay_view(self, img0, img1) -> np.ndarray:
        """Create comprehensive overlay view"""
        base = self._create_stereo_view(img0, img1)
        
        # Use smaller images for disparity computation
        scale = 0.5
        small0 = cv2.resize(img0, (0, 0), fx=scale, fy=scale)
        small1 = cv2.resize(img1, (0, 0), fx=scale, fy=scale)
        
        disparity = self.compute_disparity_map(small0, small1)
        
        # Create heatmap overlay
        heatmap = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX)
        heatmap_colored = cv2.applyColorMap(heatmap.astype(np.uint8), cv2.COLORMAP_JET)
        
        # Resize heatmap to match base image size
        heatmap_resized = cv2.resize(heatmap_colored, (base.shape[1], base.shape[0]))
        
        # Blend images
        overlay = cv2.addWeighted(base, 0.7, heatmap_resized, 0.3, 0)
        
        return overlay

    def _save_log_entry(self, data: Dict):
        """Save analysis data to file"""
        if not os.path.exists('logs'):
            os.makedirs('logs')
        
        timestamp = datetime.now().strftime('%Y%m%d')
        filename = f'logs/stereo_analysis_{timestamp}.json'
        
        # Read existing data
        existing_data = []
        if os.path.exists(filename):
            with open(filename, 'r') as f:
                try:
                    existing_data = json.load(f)
                except:
                    existing_data = []
        
        # Add new entry
        existing_data.append(data)
        
        # Save (keep last 1000 entries)
        if len(existing_data) > 1000:
            existing_data = existing_data[-1000:]
        
        with open(filename, 'w') as f:
            json.dump(existing_data, f, indent=2)

    def display_callback(self):
        """Main display callback"""
        # Process frame
        processed_frame = self.process_frame()
        
        if processed_frame is not None:
            # Get system metrics
            metrics = self.get_system_metrics()
            self.metrics_history.append(metrics)
            if len(self.metrics_history) > self.METRICS_HISTORY_SIZE:
                self.metrics_history.pop(0)
            
            # Create status overlay with correct width
            status_bar = self.create_status_overlay(metrics)
            
            # Resize processed frame to match status bar width if needed
            frame_width = processed_frame.shape[1]
            if frame_width != self.DISPLAY_WIDTH:
                processed_frame = self.resize_to_display(processed_frame)
            
            # Combine frame with status bar
            display_frame = np.vstack([processed_frame, status_bar])
            
            # Display
            cv2.imshow(self.WINDOW_NAME, display_frame)
            
            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF
            self._handle_keyboard(key)
            
            # Update FPS calculation
            self.frame_count += 1
            current_time = time.time()
            if current_time - self.last_time >= 1.0:
                self.fps = self.frame_count / (current_time - self.last_time)
                self.fps_history.append(self.fps)
                if len(self.fps_history) > 100:
                    self.fps_history.pop(0)
                
                self.frame_count = 0
                self.last_time = current_time
            
            # Record data if enabled
            if self.record_data and self.frame_count % self.save_interval == 0:
                log_data = {
                    'timestamp': time.time(),
                    'fps': self.fps,
                    'match_accuracy': self.match_accuracy,
                    'reprojection_error': self.reprojection_error,
                    'metrics': {
                        'cpu': metrics.cpu_percent,
                        'memory': metrics.memory_percent,
                        'gpu': metrics.gpu_load,
                        'cpu_temp': metrics.cpu_temp,
                        'gpu_temp': metrics.gpu_temp,
                        'latency_ms': metrics.latency_ms
                    },
                    'frame_size': f"{metrics.frame_width}x{metrics.frame_height}",
                    'frame_count': self.frame_count
                }
                self.log_queue.put(log_data)

    def _handle_keyboard(self, key):
        """Handle keyboard input"""
        if key == ord('q'):
            self.get_logger().info('Shutting down...')
            cv2.destroyAllWindows()
            rclpy.shutdown()
        elif key == ord('m'):
            self.show_matches = not self.show_matches
            self.get_logger().info(f'Show matches: {self.show_matches}')
        elif key == ord('v'):
            modes = list(ViewMode)
            current_idx = modes.index(self.view_mode)
            self.view_mode = modes[(current_idx + 1) % len(modes)]
            self.get_logger().info(f'View mode: {self.view_mode.value}')
        elif key == ord('a'):
            self.auto_tune = not self.auto_tune
            self.get_logger().info(f'Auto-tune: {self.auto_tune}')
        elif key == ord('r'):
            self.record_data = not self.record_data
            self.get_logger().info(f'Recording: {self.record_data}')
        elif key == ord('s'):
            self._save_current_frame()
        elif key == ord('+'):
            self.MAX_DISPARITY = min(self.MAX_DISPARITY + 10, 400)
            self.get_logger().info(f'Max disparity: {self.MAX_DISPARITY}')
        elif key == ord('-'):
            self.MAX_DISPARITY = max(self.MAX_DISPARITY - 10, 50)
            self.get_logger().info(f'Max disparity: {self.MAX_DISPARITY}')

    def _save_current_frame(self):
        """Save current frame to file"""
        if self.cam0_img is not None and self.cam1_img is not None:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            
            if not os.path.exists('captures'):
                os.makedirs('captures')
            
            cv2.imwrite(f'captures/cam0_{timestamp}.png', self.cam0_img)
            cv2.imwrite(f'captures/cam1_{timestamp}.png', self.cam1_img)
            
            # Save processed frame if available
            processed_frame = self.process_frame()
            if processed_frame is not None:
                cv2.imwrite(f'captures/processed_{timestamp}.png', processed_frame)
            
            self.get_logger().info(f'Saved frames with timestamp: {timestamp}')

    def destroy_node(self):
        """Cleanup on node destruction"""
        self.log_queue.put(None)
        self.log_thread.join()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = StereoAnalyzer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
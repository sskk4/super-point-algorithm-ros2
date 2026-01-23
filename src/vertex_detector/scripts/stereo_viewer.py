#!/usr/bin/env python3
"""
Stereo Vision Viewer for ROS2
Wyświetla strumienie z dwóch kamer wraz z wizualizacją dopasowań i głębi.
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
from dataclasses import dataclass
from typing import List, Tuple, Optional
from enum import Enum


class ViewMode(Enum):
    """Tryby wyświetlania"""
    STEREO = "stereo"
    DEPTH = "depth"


@dataclass
class SystemMetrics:
    """Metryki systemowe"""
    fps: float = 0.0
    gpu_load: float = 0.0
    cpu_temp: float = 0.0
    gpu_temp: float = 0.0
    ram_usage: float = 0.0


@dataclass
class MatchPair:
    """Para dopasowanych punktów"""
    left_point: Tuple[int, int]
    right_point: Tuple[int, int]
    disparity: int

    @property
    def depth_estimate(self) -> float:
        """Prosta estymacja głębi (wymaga kalibracji)"""
        return max(1, self.disparity)


class StereoViewer(Node):
    """
    Node ROS2 do wizualizacji stereo vision
    
    Subskrybuje obrazy z dwóch kamer i wyświetla je wraz z:
    - Dopasowaniami punktów charakterystycznych
    - Mapą głębi
    - Metrykami systemowymi
    """
    
    # Konfiguracja
    WINDOW_NAME = 'Stereo Vertex System V6'
    DISPLAY_REFRESH_RATE = 0.033  # ~30 FPS
    EPIPOLAR_TOLERANCE = 8  # piksele
    MAX_DISPARITY = 200  # piksele
    
    # Ścieżki systemowe dla Jetson Orin
    GPU_LOAD_PATH = "/sys/devices/platform/17000000.gpu/load"
    CPU_TEMP_PATH = "/sys/class/thermal/thermal_zone0/temp"
    GPU_TEMP_PATH = "/sys/class/thermal/thermal_zone1/temp"
    
    # Zakres kolorów dla detekcji zielonych punktów (BGR)
    GREEN_LOWER = (0, 200, 0)
    GREEN_UPPER = (100, 255, 100)

    def __init__(self):
        super().__init__('stereo_viewer_v6')
        
        # Inicjalizacja komponentów
        self.bridge = CvBridge()
        self.cam0_img: Optional[np.ndarray] = None
        self.cam1_img: Optional[np.ndarray] = None
        
        # Stan aplikacji
        self.show_matches = True
        self.view_mode = ViewMode.STEREO
        self.metrics = SystemMetrics()
        
        # Licznik FPS
        self._frame_count = 0
        self._last_fps_update = time.time()
        
        # Setup UI
        cv2.namedWindow(self.WINDOW_NAME, cv2.WINDOW_NORMAL)
        
        # Setup ROS2 subscribers
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.sub_cam0 = self.create_subscription(
            Image, '/cam0/vertex_debug_image', self._cam0_callback, qos
        )
        self.sub_cam1 = self.create_subscription(
            Image, '/cam1/vertex_debug_image', self._cam1_callback, qos
        )
        
        # Timer dla wyświetlania
        self.timer = self.create_timer(self.DISPLAY_REFRESH_RATE, self._display_callback)
        
        self.get_logger().info(
            f'{self.WINDOW_NAME} Online\n'
            'Klawisze: [M] Toggle Matches, [V] Toggle View Mode, [Q] Quit'
        )

    def _cam0_callback(self, msg: Image) -> None:
        """Callback dla lewej kamery"""
        try:
            self.cam0_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Błąd konwersji cam0: {e}')

    def _cam1_callback(self, msg: Image) -> None:
        """Callback dla prawej kamery"""
        try:
            self.cam1_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Błąd konwersji cam1: {e}')

    def _read_system_value(self, path: str, scale: float = 1.0) -> float:
        """Odczytuje wartość z pliku systemowego"""
        try:
            with open(path, 'r') as f:
                return float(f.read().strip()) * scale
        except (FileNotFoundError, ValueError, PermissionError):
            return 0.0

    def _update_system_metrics(self) -> None:
        """Aktualizuje metryki systemowe"""
        self.metrics.gpu_load = self._read_system_value(self.GPU_LOAD_PATH, 0.1)
        self.metrics.cpu_temp = self._read_system_value(self.CPU_TEMP_PATH, 0.001)
        self.metrics.gpu_temp = self._read_system_value(self.GPU_TEMP_PATH, 0.001)
        self.metrics.ram_usage = psutil.virtual_memory().percent

    def _update_fps(self) -> None:
        """Aktualizuje licznik FPS"""
        self._frame_count += 1
        current_time = time.time()
        elapsed = current_time - self._last_fps_update
        
        if elapsed > 1.0:
            self.metrics.fps = self._frame_count / elapsed
            self._frame_count = 0
            self._last_fps_update = current_time

    def _detect_green_points(self, img: np.ndarray) -> List[Tuple[int, int]]:
        """
        Wykrywa zielone punkty narysowane przez węzeł C++
        
        Returns:
            Lista centroidów (x, y)
        """
        mask = cv2.inRange(img, self.GREEN_LOWER, self.GREEN_UPPER)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        centers = []
        for contour in contours:
            moments = cv2.moments(contour)
            if moments["m00"] != 0:
                cx = int(moments["m10"] / moments["m00"])
                cy = int(moments["m01"] / moments["m00"])
                centers.append((cx, cy))
        
        return centers

    def _find_stereo_matches(self, img0: np.ndarray, img1: np.ndarray) -> List[MatchPair]:
        """
        Znajduje dopasowania między punktami z dwóch obrazów
        
        Używa ograniczenia epipolarnego i disparities dla stereo vision.
        """
        points_left = self._detect_green_points(img0)
        points_right = self._detect_green_points(img1)
        
        matches = []
        
        for left_pt in points_left:
            best_match = None
            min_vertical_diff = self.EPIPOLAR_TOLERANCE
            
            for right_pt in points_right:
                # Sprawdź ograniczenie epipolarne (punkty na tej samej wysokości)
                vertical_diff = abs(left_pt[1] - right_pt[1])
                
                # Oblicz dysparity (punkt w lewym obrazie powinien być na prawo)
                disparity = left_pt[0] - right_pt[0]
                
                # Filtruj poprawne dopasowania
                if (vertical_diff < min_vertical_diff and 
                    0 < disparity < self.MAX_DISPARITY):
                    min_vertical_diff = vertical_diff
                    best_match = right_pt
            
            if best_match is not None:
                matches.append(MatchPair(
                    left_point=left_pt,
                    right_point=best_match,
                    disparity=left_pt[0] - best_match[0]
                ))
        
        return matches

    def _render_stereo_view(self, matches: List[MatchPair]) -> np.ndarray:
        """Renderuje widok stereo z dopasowaniami"""
        img_left = self.cam0_img.copy()
        img_right = self.cam1_img.copy()
        
        h, w = img_left.shape[:2]
        canvas = np.hstack([img_left, img_right])
        
        if self.show_matches:
            for i, match in enumerate(matches):
                # Generuj unikalny kolor dla każdej pary
                hue = (i * 45) % 180
                color_hsv = np.uint8([[[hue, 255, 255]]])
                color_bgr = cv2.cvtColor(color_hsv, cv2.COLOR_HSV2BGR)[0][0]
                color = tuple(map(int, color_bgr))
                
                # Punkt w prawym obrazie przesunięty o szerokość lewego
                right_pt_shifted = (match.right_point[0] + w, match.right_point[1])
                
                # Rysuj linię łączącą
                cv2.line(canvas, match.left_point, right_pt_shifted, color, 2)
                cv2.circle(canvas, match.left_point, 5, color, -1)
                cv2.circle(canvas, right_pt_shifted, 5, color, -1)
        
        return canvas

    def _render_depth_view(self, matches: List[MatchPair]) -> np.ndarray:
        """Renderuje widok z mapą głębi"""
        # Przyciemniony lewy obraz jako tło
        img_depth = (self.cam0_img.copy() * 0.3).astype(np.uint8)
        
        for match in matches:
            # Mapuj disparities na kolory (Jet colormap)
            # Większa disparity = bliżej kamery = czerwony
            normalized_disp = int(np.clip(match.disparity * 4, 0, 255))
            color_map = cv2.applyColorMap(
                np.array([[normalized_disp]], dtype=np.uint8), 
                cv2.COLORMAP_JET
            )
            color = tuple(map(int, color_map[0][0]))
            
            # Rysuj punkt głębi
            cv2.circle(img_depth, match.left_point, 8, color, -1)
            cv2.circle(img_depth, match.left_point, 8, (255, 255, 255), 1)
            
            # Dodaj wartość disparity jako tekst
            text_pos = (match.left_point[0] + 10, match.left_point[1] - 8)
            cv2.putText(
                img_depth, f'{match.disparity}px', text_pos,
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1
            )
        
        # Dodaj prawą kamerę obok
        canvas = np.hstack([img_depth, self.cam1_img])
        return canvas

    def _render_status_bar(self, canvas_width: int, match_count: int) -> np.ndarray:
        """Renderuje pasek statusu z metrykami"""
        status_bar = np.zeros((60, canvas_width, 3), dtype=np.uint8)
        
        # Definicja statystyk do wyświetlenia
        h, w = self.cam0_img.shape[:2] if self.cam0_img is not None else (0, 0)
        
        stats = [
            (f"FPS: {self.metrics.fps:.1f}", (10, 25), (100, 255, 100)),
            (f"Resolution: {w}x{h}", (10, 50), (180, 180, 180)),
            (f"GPU: {self.metrics.gpu_load:.0f}%", (220, 25), (100, 255, 100)),
            (f"Temp CPU/GPU: {self.metrics.cpu_temp:.1f}°C / {self.metrics.gpu_temp:.1f}°C", 
             (220, 50), (100, 200, 255)),
            (f"RAM: {self.metrics.ram_usage:.1f}%", (550, 25), (255, 180, 100)),
            (f"Matches: {match_count}", (550, 50), (255, 255, 100)),
            (f"Mode: {self.view_mode.value.upper()}", (750, 25), (255, 255, 255)),
            (f"Show Lines: {'ON' if self.show_matches else 'OFF'}", (750, 50), 
             (100, 255, 100) if self.show_matches else (150, 150, 150))
        ]
        
        for text, position, color in stats:
            cv2.putText(
                status_bar, text, position,
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA
            )
        
        return status_bar

    def _handle_keyboard_input(self) -> None:
        """Obsługuje wejście z klawiatury"""
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q'):
            self.get_logger().info('Shutting down...')
            rclpy.shutdown()
        elif key == ord('m'):
            self.show_matches = not self.show_matches
            self.get_logger().info(f'Match display: {"ON" if self.show_matches else "OFF"}')
        elif key == ord('v'):
            self.view_mode = (ViewMode.DEPTH if self.view_mode == ViewMode.STEREO 
                            else ViewMode.STEREO)
            self.get_logger().info(f'View mode: {self.view_mode.value}')

    def _display_callback(self) -> None:
        """Główna pętla wyświetlania (wywoływana przez timer)"""
        # Sprawdź czy mamy obrazy z obu kamer
        if self.cam0_img is None or self.cam1_img is None:
            return
        
        # Aktualizuj metryki
        self._update_fps()
        self._update_system_metrics()
        
        # Znajdź dopasowania
        matches = self._find_stereo_matches(self.cam0_img, self.cam1_img)
        
        # Renderuj odpowiedni widok
        if self.view_mode == ViewMode.STEREO:
            canvas = self._render_stereo_view(matches)
        else:
            canvas = self._render_depth_view(matches)
        
        # Dodaj pasek statusu
        status_bar = self._render_status_bar(canvas.shape[1], len(matches))
        final_image = np.vstack([canvas, status_bar])
        
        # Wyświetl
        cv2.imshow(self.WINDOW_NAME, final_image)
        
        # Obsłuż klawiaturę
        self._handle_keyboard_input()


def main(args=None):
    """Główna funkcja programu"""
    rclpy.init(args=args)
    node = StereoViewer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received')
    finally:
        cv2.destroyAllWindows()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
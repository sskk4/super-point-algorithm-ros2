import os
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    # =========================================================================
    # ŚCIEŻKI
    # =========================================================================
    pkg_share = FindPackageShare('vertex_detector')
    model_path = PathJoinSubstitution([pkg_share, 'models', 'superpoint_320.onnx'])
    config_path = PathJoinSubstitution([pkg_share, 'config', 'detector_params.yaml'])
    
    
    # =========================================================================
    # VERTEX DETECTOR - CAM0 
    # =========================================================================
    vertex_detector_cam0 = Node(
        package='vertex_detector',
        executable='vertex_detector_standalone',
        name='vertex_detector_cam0',
        output='screen',
        remappings=[
            ('camera/image_raw', 'cam0/image_raw'),
            ('camera/camera_info', 'cam0/camera_info'),
            ('vertex_detections', 'cam0/vertex_detections'),
            ('vertex_debug_image', 'cam0/vertex_debug_image'),
        ],
        parameters=[
            config_path,
            {
                'model_path': model_path,
                'enable_debug_visualization': True,
                'use_gpu': True,
                'use_fp16': True,
            }
        ]
    )
    
    # =========================================================================
    # VERTEX DETECTOR - CAM1 
    # =========================================================================
    vertex_detector_cam1 = Node(
        package='vertex_detector',
        executable='vertex_detector_standalone',
        name='vertex_detector_cam1',
        output='screen',
        remappings=[
            ('camera/image_raw', 'cam1/image_raw'),
            ('camera/camera_info', 'cam1/camera_info'),
            ('vertex_detections', 'cam1/vertex_detections'),
            ('vertex_debug_image', 'cam1/vertex_debug_image'),
        ],
        parameters=[
            config_path,
            {
                'model_path': model_path,
                'enable_debug_visualization': True,
                'use_gpu': True,
                'use_fp16': True,
            }
        ]
    )
    
    return launch.LaunchDescription([
        vertex_detector_cam0,
        vertex_detector_cam1,
    ])



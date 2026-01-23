import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    #KAMERA 1 (prawa)
    argus_mono_first_node = ComposableNode(
        name='argus_mono_first',
        package='isaac_ros_argus_camera',
        plugin='nvidia::isaac_ros::argus::ArgusMonoNode',
        remappings=[('left/image_raw', 'cam1/image_raw'), 
                    ('left/camera_info', 'cam1/camera_info')],
        parameters=[{
            'camera_id': 0,
            'module_id': 0,
            'mode': 5,
            'fsync_type': 0,
        }]
    )
    
    #KAMERA 2 (lewa)
    argus_mono_second_node = ComposableNode(
        name='argus_mono_second',
        package='isaac_ros_argus_camera',
        plugin='nvidia::isaac_ros::argus::ArgusMonoNode',
        remappings=[('left/image_raw', 'cam0/image_raw'), 
                    ('left/camera_info', 'cam0/camera_info')],
        parameters=[{
            'camera_id': 1,
            'module_id': 1,
            'mode': 5,
            'fsync_type': 0,
        }]
    )  
    
    #KONTERNER
    argus_mono_container = ComposableNodeContainer(
        name='argus_mono_container',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[argus_mono_first_node, argus_mono_second_node],
        namespace='',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'],
    )
    return launch.LaunchDescription([argus_mono_container])

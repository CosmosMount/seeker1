import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, PushRosNamespace
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    # 1. 声明启动参数 (替代 <arg>)
    declare_namespace_arg = DeclareLaunchArgument(
        'namespace', default_value='camera',
        description='ROS namespace to launch the nodes in.')

    declare_cam_name_arg = DeclareLaunchArgument(
        'input_camera_name', default_value='fisheye1',
        description='Topic prefix for the camera.')
    
    declare_scale_arg = DeclareLaunchArgument(
        'scale', default_value='1.0',
        description='Scale factor for output resolution.')

    # 2. 定义组件容器 (替代 nodelet manager)
    container = ComposableNodeContainer(
            name='image_undistort_container',
            # 命名空间将在 GroupAction 中被推入
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            output='screen',
            composable_node_descriptions=[
                
                # 3. 定义 ImageUndistortNodelet 组件
                ComposableNode(
                    package='image_undistort',
                    # 插件名必须匹配 C++ 中的 RCLCPP_COMPONENTS_REGISTER_NODE
                    plugin='image_undistort::ImageUndistortNodelet', 
                    name='image_undistort_node', # <node name="...">
                    
                    # 4. 迁移 <param>
                    # 注意：'input_camera_info_from_ros_params' 默认为 false，
                    # 这与 ROS 1 版本一致 (即它将订阅话题)
                    parameters=[
                        {
                            'scale': LaunchConfiguration('scale'),
                            'output_camera_info_source': 'match_input',
                        }
                    ],
                    
                    # 5. 迁移 <remap>
                    remappings=[
                        ('input/image', [LaunchConfiguration('input_camera_name'), '/image_raw']),
                        ('input/camera_info', [LaunchConfiguration('input_camera_name'), '/camera_info']),
                        ('output/image', [LaunchConfiguration('input_camera_name'), '_rect/image']),
                        ('output/camera_info', [LaunchConfiguration('input_camera_name'), '_rect/camera_info']),
                    ]
                )
            ]
    )
    
    # 6. 迁移 <group ns="...">
    namespaced_container_group = GroupAction(
        actions=[
            PushRosNamespace(LaunchConfiguration('namespace')),
            container,
        ]
    )

    # 7. 返回 LaunchDescription
    return LaunchDescription([
        declare_namespace_arg,
        declare_cam_name_arg,
        declare_scale_arg,
        
        namespaced_container_group
    ])
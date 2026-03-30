import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    # 1. 声明启动参数 (替代 <arg>)
    declare_first_cam_arg = DeclareLaunchArgument(
        'first_camera_name', default_value='cam00',
        description='Topic prefix for the first camera.')
    
    declare_second_cam_arg = DeclareLaunchArgument(
        'second_camera_name', default_value='cam01',
        description='Topic prefix for the second camera.')

    # (这些参数在 ROS 1 文件中未被使用，但我们保留它们以保持一致)
    declare_play_bag_arg = DeclareLaunchArgument(
        'play_bag', default_value='true',
        description='(Unused in this script) Play a bag file.')
    
    declare_bag_file_arg = DeclareLaunchArgument(
        'bag_file', default_value='/home/z/Datasets/KITTI/2011_09_26/2011_09_26_drive_0035_sync/data.bag',
        description='(Unused in this script) Path to the bag file.')

    # 2. 定义组件容器 (替代 nodelet manager)
    container = ComposableNodeContainer(
            name='disparity_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            output='screen',
            composable_node_descriptions=[
                
                # 3. 定义 DepthNodelet 组件 (替代 <node type="depth_node">)
                ComposableNode(
                    package='image_undistort',
                    # 插件名必须匹配 C++ 中的 RCLCPP_COMPONENTS_REGISTER_NODE
                    plugin='image_undistort::DepthNodelet', 
                    name='depth_node', # <node name="...">
                    
                    # 4. 迁移 <remap>
                    remappings=[
                        ('rect/first/image', [LaunchConfiguration('first_camera_name'), '/image_raw']),
                        ('rect/second/image', [LaunchConfiguration('second_camera_name'), '/image_raw']),
                        ('rect/first/camera_info', [LaunchConfiguration('first_camera_name'), '/camera_info']),
                        ('rect/second/camera_info', [LaunchConfiguration('second_camera_name'), '/camera_info']),
                        
                        # (可以添加输出话题的重映射)
                        # ('disparity/image', '/my_disparity'),
                        # ('pointcloud', '/my_pointcloud'),
                    ],
                    
                    # 5. (可选) 添加参数
                    #    README.md 中 depth_node 的所有参数都可以在这里设置
                    # parameters=[{
                    #     'use_sgbm': True,
                    #     'sad_window_size': 11,
                    #     # ...
                    # }]
                )
            ]
    )

    # 6. 返回 LaunchDescription
    return LaunchDescription([
        declare_first_cam_arg,
        declare_second_cam_arg,
        declare_play_bag_arg,
        declare_bag_file_arg,
        container
    ])
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    
    # 1. 定义 ROS 2 组件容器 (替代 nodelet manager)
    container = ComposableNodeContainer(
            name='stereo_proc',  # ROS 1 manager 的名字
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            output='screen',
            
            # 2. 定义要加载到此容器的组件 (替代 nodelet load)
            composable_node_descriptions=[
                ComposableNode(
                    package='disparity_image_proc',
                    plugin='stereo_image_proc::DepthImageNodelet', # C++ 中注册的插件类名
                    name='disparity_image_proc', # ROS 1 nodelet 的名字
                    
                    # 3. 话题重映射 (替代 <remap> 标签)
                    remappings=[
                        ('/right/camera_info', '/multisense_sl/camera/right/camera_info'),
                        ('/left/camera_info', '/multisense_sl/camera/left/camera_info'),
                        ('/left/image_rect_color', '/multisense_sl/camera/left/image_rect_color'),
                        ('/disparity', '/multisense_sl/camera/disparity'),
                        ('/depth_image', '/multisense_sl/camera/left/depth_image'),
                    ]
                )
            ]
    )
    
    # 4. 返回 LaunchDescription
    return LaunchDescription([
        container
    ])
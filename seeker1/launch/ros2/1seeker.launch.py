import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    # 1. 声明启动参数 (替代 <arg>)
    declare_use_image_transport_arg = DeclareLaunchArgument(
        'use_image_transport', default_value='true')
    
    declare_pub_disparity_img_arg = DeclareLaunchArgument(
        'pub_disparity_img', default_value='true')

    declare_pub_disparity_arg = DeclareLaunchArgument(
        'pub_disparity', default_value='true')

    declare_pub_imu_arg = DeclareLaunchArgument(
        'pub_imu', default_value='true')
    
    declare_time_sync_arg = DeclareLaunchArgument(
        'time_sync', default_value='true')

    # 2. 定义组件容器 (替代 nodelet manager)
    container = ComposableNodeContainer(
            name='seeker_node_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            output='screen',
            composable_node_descriptions=[
                
                # 3. 定义 Seeker 组件
                #    (替代 <node ... args="standalone seeker/SeekRosNode">)
                ComposableNode(
                    package='seeker', # 您的包名 (来自 package.xml)
                    # 插件名必须匹配 C++ 中的 RCLCPP_COMPONENTS_REGISTER_NODE
                    plugin='SeekRosNode', 
                    name='seeker_node', # <node name="...">
                    
                    # 4. 迁移 <param>
                    parameters=[
                        {
                            'use_image_transport': LaunchConfiguration('use_image_transport'),
                            'pub_disparity_img': LaunchConfiguration('pub_disparity_img'),
                            'pub_disparity': LaunchConfiguration('pub_disparity'),
                            'pub_imu': LaunchConfiguration('pub_imu'),
                            'time_sync': LaunchConfiguration('time_sync'),
                            'imu_link': 'imu', # 硬编码的参数
                        }
                    ],
                    
                    # (此 launch 文件没有 <remap>, 但可以在这里添加)
                    # remappings=[
                    #     ('/fisheye/left/image_raw', '/my_topic/image'),
                    # ]
                )
            ]
    )

    # 5. 返回 LaunchDescription
    return LaunchDescription([
        declare_use_image_transport_arg,
        declare_pub_disparity_img_arg,
        declare_pub_disparity_arg,
        declare_pub_imu_arg,
        declare_time_sync_arg,
        
        container
    ])
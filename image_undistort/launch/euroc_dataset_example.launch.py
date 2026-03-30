import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    # 1. 声明启动参数 (替代 <arg>)
    declare_first_cam_arg = DeclareLaunchArgument(
        'first_camera_name', default_value='cam0',
        description='Topic prefix for the first camera.')
    
    declare_second_cam_arg = DeclareLaunchArgument(
        'second_camera_name', default_value='cam1',
        description='Topic prefix for the second camera.')

    declare_scale_arg = DeclareLaunchArgument(
        'scale', default_value='1.0',
        description='Scale factor for output resolution.')

    declare_process_nth_frame_arg = DeclareLaunchArgument(
        'process_every_nth_frame', default_value='1',
        description='Process every Nth frame.')

    # 路径在 ROS 2 中通常通过包共享目录解析
    # 为保持一致性，我们暂时保留硬编码路径
    declare_calib_path_arg = DeclareLaunchArgument(
        'stereo_params_camchain',
        default_value='/home/z/catkin_ws/src/mav_tools/mav_startup/parameters/mavs/ibis/camchain_p23023.yaml',
        description='Path to Kalibr camchain YAML file.')

    # 2. 定义组件容器 (替代 nodelet manager)
    container = ComposableNodeContainer(
            name='stereo_undistort_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            output='screen',
            composable_node_descriptions=[
                
                # 3. 定义 StereoUndistortNodelet 组件
                ComposableNode(
                    package='image_undistort',
                    # 插件名必须匹配 C++ 中的 RCLCPP_COMPONENTS_REGISTER_NODE
                    plugin='image_undistort::StereoUndistortNodelet', 
                    name='stereo_undistort', # <node name="...">
                    
                    # 4. 迁移 <param> 和 <rosparam>
                    #    ROS 2 parameters 列表可以同时接受字典和 YAML 文件路径
                    parameters=[
                        {
                            'input_camera_info_from_ros_params': True,
                            'first_camera_namespace': LaunchConfiguration('first_camera_name'),
                            'second_camera_namespace': LaunchConfiguration('second_camera_name'),
                            'scale': LaunchConfiguration('scale'),
                            'process_every_nth_frame': LaunchConfiguration('process_every_nth_frame'),
                        },
                        # 5. 这是 ROS 2 加载 <rosparam file=...> 的方式
                        LaunchConfiguration('stereo_params_camchain')
                    ],
                    
                    # 6. 迁移 <remap>
                    remappings=[
                        ('raw/first/image', [LaunchConfiguration('first_camera_name'), '/image_raw']),
                        ('raw/second/image', [LaunchConfiguration('second_camera_name'), '/image_raw']),
                    ]
                )
            ]
    )

    # 7. 返回 LaunchDescription
    return LaunchDescription([
        declare_first_cam_arg,
        declare_second_cam_arg,
        declare_scale_arg,
        declare_process_nth_frame_arg,
        declare_calib_path_arg,
        container
    ])
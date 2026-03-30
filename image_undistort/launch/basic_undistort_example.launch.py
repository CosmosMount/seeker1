import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    # 1. 声明启动参数 (替代 <arg>)
    declare_cam_name_arg = DeclareLaunchArgument(
        'input_camera_name', default_value='cam0',
        description='Input camera name topic prefix.')
    
    declare_scale_arg = DeclareLaunchArgument(
        'scale', default_value='1.0',
        description='Scale factor for output resolution.')

    # 注意：这个路径是您本地的，在 ROS 2 中，我们通常会
    # 将 YAML 放在 'install/pkg_name/share/pkg_name/config' 下
    # 但为了 1:1 迁移，我们保留了加载外部文件的功能
    declare_calib_path_arg = DeclareLaunchArgument(
        'calib_path',
        default_value='/home/z/Desktop/cal/camchain-.2018-04-25-20-16-31.yaml',
        description='Path to Kalibr camchain YAML file.')

    # 2. 定义组件容器 (替代 ROS 1 的 nodelet manager)
    container = ComposableNodeContainer(
            name='image_undistort_container',
            namespace='',
            package='rclcpp_components', # 这是 ROS 2 的标准容器包
            executable='component_container',
            output='screen',
            composable_node_descriptions=[
                # 3. 定义要加载的组件 (替代 <node type="image_undistort_node">)
                ComposableNode(
                    package='image_undistort',
                    # 插件名必须匹配 C++ 中的 RCLCPP_COMPONENTS_REGISTER_NODE 宏
                    plugin='image_undistort::ImageUndistortNodelet', 
                    name='image_undistort_node', # <node name="...">
                    
                    # 4. 迁移 <param> 和 <rosparam>
                    #    ROS 2 parameters 参数可以是一个列表，
                    #    它能同时接受字典和指向 YAML 文件的 LaunchConfiguration
                    parameters=[
                        {
                            'input_camera_namespace': 'cam0',
                            'input_camera_info_from_ros_params': True,
                            'scale': LaunchConfiguration('scale'),
                            'output_camera_info_source': 'auto_generated',
                        },
                        # 5. 这是 ROS 2 加载 <rosparam file=...> 的方式
                        LaunchConfiguration('calib_path')
                    ],
                    
                    # 6. 迁移 <remap>
                    #    (注意：使用列表来拼接 LaunchConfiguration 和字符串)
                    remappings=[
                        ('input/image', [LaunchConfiguration('input_camera_name'), '/image_raw'])
                    ]
                )
            ]
    )

    # 7. 返回完整的 LaunchDescription
    return LaunchDescription([
        declare_cam_name_arg,
        declare_scale_arg,
        declare_calib_path_arg,
        container
    ])
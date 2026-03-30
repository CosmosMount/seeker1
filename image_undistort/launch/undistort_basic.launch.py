import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, PushRosNamespace
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    # 1. 声明启动参数 (替代 <arg>)
    declare_mav_name_arg = DeclareLaunchArgument(
        'mav_name', default_value='elster',
        description='MAV name, used for default namespace.')
    
    declare_namespace_arg = DeclareLaunchArgument(
        'namespace', default_value=LaunchConfiguration('mav_name'),
        description='ROS namespace to launch the nodes in.')

    declare_cam_name_arg = DeclareLaunchArgument(
        'input_camera_name', default_value='cam0',
        description='Topic prefix for the camera.')
    
    declare_scale_arg = DeclareLaunchArgument(
        'scale', default_value='1.0',
        description='Scale factor for output resolution.')

    # 2. 迁移 $(find mav_startup)
    # 假设 'mav_startup' 也是一个 ROS 2 包
    mav_startup_share_dir = get_package_share_directory('mav_startup')
    default_calib_path = os.path.join(
        mav_startup_share_dir, 'parameters', 'mavs', 'elster', 'camchain_p23037.yaml')

    declare_calib_path_arg = DeclareLaunchArgument(
        'calib_path',
        default_value=default_calib_path,
        description='Path to Kalibr camchain YAML file.')

    # 3. 定义组件容器 (替代 nodelet manager)
    container = ComposableNodeContainer(
            name='image_undistort_container',
            # 命名空间将在 GroupAction 中被推入
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            output='screen',
            composable_node_descriptions=[
                
                # 4. 定义 ImageUndistortNodelet 组件
                ComposableNode(
                    package='image_undistort',
                    # 插件名必须匹配 C++ 中的 RCLCPP_COMPONENTS_REGISTER_NODE
                    plugin='image_undistort::ImageUndistortNodelet', 
                    name='image_undistort_node', # <node name="...">
                    
                    # 5. 迁移 <param> 和 <rosparam>
                    parameters=[
                        {
                            'input_camera_namespace': LaunchConfiguration('input_camera_name'),
                            'input_camera_info_from_ros_params': True,
                            'scale': LaunchConfiguration('scale'),
                        },
                        # 6. 这是 ROS 2 加载 <rosparam file=...> 的方式
                        LaunchConfiguration('calib_path')
                    ],
                    
                    # 7. 迁移 <remap>
                    remappings=[
                        ('input/image', [LaunchConfiguration('input_camera_name'), '/image_raw']),
                    ]
                )
            ]
    )
    
    # 8. 迁移 <group ns="...">
    namespaced_container_group = GroupAction(
        actions=[
            PushRosNamespace(LaunchConfiguration('namespace')),
            container,
        ]
    )

    # 9. 返回 LaunchDescription
    return LaunchDescription([
        declare_mav_name_arg,
        declare_namespace_arg,
        declare_cam_name_arg,
        declare_scale_arg,
        declare_calib_path_arg,
        
        namespaced_container_group
    ])
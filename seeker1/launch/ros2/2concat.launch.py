import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, PushRosNamespace
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    # 1. 声明所有 <arg>
    declare_manager_arg = DeclareLaunchArgument(
        'manager', default_value='manager',
        description='The name of the nodelet manager to load into.')
        
    declare_standalone_arg = DeclareLaunchArgument(
        'standalone', default_value='false',
        description='Launch components as standalone nodes.')
        
    # ... (声明所有其他 arg: concat_method, blend_weight, etc.) ...
    declare_concat_method_arg = DeclareLaunchArgument('concat_method', default_value='3')
    declare_blend_weight_arg = DeclareLaunchArgument('blend_weight', default_value='0.4')
    # ...

    seeker_share_dir = get_package_share_directory('seeker')
    default_calib_path = os.path.join(
        seeker_share_dir, 'config', LaunchConfiguration('config'), 'kalibr_cam_chain.yaml')

    declare_calib_path_arg = DeclareLaunchArgument(
        'stereo_params_camchain',
        default_value=default_calib_path,
        description='Path to Kalibr camchain YAML file.')

    # 2. 定义两个组件
    
    # --- 组件 1: SeekRosNode ---
    seeker_component = ComposableNode(
        package='seeker',
        plugin='SeekRosNode',
        name='seeker_node',
        parameters=[{
            'use_image_transport': False,
            'pub_disparity_img': True,
            'pub_disparity': True,
            'pub_imu': True,
            'time_sync': True,
            'imu_link': 'imu',
        }],
    )

    # --- 组件 2: ImageConcatNodelet (!! 缺失 !!) ---
    # 假设插件名为 'image_undistort::ImageConcatNodelet'
    concat_component = ComposableNode(
        package='image_undistort', # (或提供此插件的任何包)
        plugin='image_undistort::ImageConcatNodelet', # !! 插件名需要确认 !!
        name='concat_node',
        parameters=[
            {
                'input_camera_info_from_ros_params': True,
                'first_camera_namespace': LaunchConfiguration('front_left_camera_name'),
                'second_camera_namespace': LaunchConfiguration('front_right_camera_name'),
                'third_camera_namespace': LaunchConfiguration('back_right_camera_name'),
                'fourth_camera_namespace': LaunchConfiguration('back_left_camera_name'),
                'process_every_nth_frame': LaunchConfiguration('process_every_nth_frame'),
                'concat_method': LaunchConfiguration('concat_method'),
                'blend_weight': LaunchConfiguration('blend_weight'),
                # ... (所有其他 concat 参数)
            },
            LaunchConfiguration('stereo_params_camchain') # 加载 YAML
        ],
        remappings=[
            ('raw/first/image', '/fisheye/left/image_raw'),
            ('raw/second/image', '/fisheye/right/image_raw'),
            ('raw/third/image', '/fisheye/bright/image_raw'),
            ('raw/fourth/image', '/fisheye/bleft/image_raw'),
            ('/rect/image', [LaunchConfiguration('undistort_node_name'), '/image']),
        ]
    )

    # 3. 迁移 ROS 1 的 standalone/manager 逻辑
    
    # 这是一个高级启动函数，它会检查 'standalone' 参数
    def launch_setup(context, *args, **kwargs):
        if context.launch_configurations['standalone'] == 'true':
            # 启动两个 *独立* 的容器，每个容器加载一个组件
            # (这模拟了 'args="standalone..."')
            container1 = ComposableNodeContainer(
                name='seeker_node_container',
                namespace='',
                package='rclcpp_components',
                executable='component_container',
                composable_node_descriptions=[seeker_component],
                output='screen',
            )
            container2 = ComposableNodeContainer(
                name='concat_node_container',
                namespace='',
                package='rclcpp_components',
                executable='component_container',
                composable_node_descriptions=[concat_component],
                output='screen',
            )
            return [container1, container2]
        
        else:
            # 启动一个 *共享* 容器，加载两个组件
            # (这模拟了 'args="load..."' 和外部管理器)
            container = ComposableNodeContainer(
                name=LaunchConfiguration('manager'),
                namespace='',
                package='rclcpp_components',
                executable='component_container',
                output='screen',
                composable_node_descriptions=[
                    seeker_component,
                    concat_component
                ],
            )
            return [container]

    # 4. 返回 LaunchDescription
    return LaunchDescription([
        # 声明所有 <arg>
        declare_manager_arg,
        declare_standalone_arg,
        declare_concat_method_arg,
        declare_blend_weight_arg,
        declare_calib_path_arg,
        # ... (所有其他 arg)
        
        # OpaqueFunction 允许我们在 launch 时执行 python 逻辑
        OpaqueFunction(function=launch_setup)
    ])
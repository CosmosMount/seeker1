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
        'mav_name', default_value='ibis',
        description='MAV name, used for default namespace.')
    
    declare_namespace_arg = DeclareLaunchArgument(
        'namespace', default_value=LaunchConfiguration('mav_name'),
        description='ROS namespace to launch the nodes in.')

    declare_process_nth_frame_arg = DeclareLaunchArgument(
        'process_every_nth_frame', default_value='1',
        description='Process every Nth frame.')

    # 找到 mav_startup 包的路径来定位标定文件
    # 假设 'mav_startup' 也是一个 ROS 2 包
    mav_startup_share_dir = get_package_share_directory('mav_startup')
    default_calib_path = os.path.join(
        mav_startup_share_dir, 'parameters', 'mavs', 'ibis', 'camchain_p22031.yaml')

    declare_calib_path_arg = DeclareLaunchArgument(
        'stereo_params_camchain',
        default_value=default_calib_path,
        description='Path to Kalibr camchain YAML file.')
    
    # 2. 定义组件容器
    # 我们将把所有组件加载到这个容器中
    container = ComposableNodeContainer(
            name='dense_stereo_container',
            # 关键：将容器本身放入命名空间
            namespace=LaunchConfiguration('namespace'),
            package='rclcpp_components',
            executable='component_container',
            output='screen',
            
            # 3. 声明要组合的 *所有* 组件
            composable_node_descriptions=[
                
                # --- 组件 1: StereoUndistortNodelet ---
                # (对应原 <node> 上的参数)
                ComposableNode(
                    package='image_undistort',
                    plugin='image_undistort::StereoUndistortNodelet', 
                    name='stereo_undistort_node', # (原 name="dense_stereo")
                    
                    parameters=[
                        {
                            'input_camera_info_from_ros_params': True,
                            'first_camera_namespace': 'cam0',
                            'second_camera_namespace': 'cam1',
                            'first_output_frame': 'cam0_rect',
                            'second_output_frame': 'cam1_rect',
                            'scale': 1.0, # 注意：ROS 1 示例是 1.0，不是 arg
                            'process_every_nth_frame': LaunchConfiguration('process_every_nth_frame'),
                        },
                        # 加载 Kalibr 标定文件
                        LaunchConfiguration('stereo_params_camchain')
                    ],
                    
                    # 外部话题重映射
                    remappings=[
                        ('raw/first/image', 'cam0/image_raw'),
                        ('raw/second/image', 'cam1/image_raw'),
                        ('raw/first/camera_info', 'cam0/camera_info'),
                        ('raw/second/camera_info', 'cam1/camera_info'),
                        
                        # 内部话题（保持默认，以便 DepthNodelet 可以找到它们）
                        # 'rect/first/image',
                        # 'rect/second/image',
                        # 'rect/first/camera_info',
                        # 'rect/second/camera_info',
                    ]
                ),
                
                # --- 组件 2: DepthNodelet ---
                # (对应原 <param name="depth/...">)
                ComposableNode(
                    package='image_undistort',
                    plugin='image_undistort::DepthNodelet', 
                    # 关键：我们给它命名 "depth_node"，
                    # 但参数 *不* 需要 'depth.' 前缀
                    name='depth_node',
                    
                    parameters=[{
                        'use_sgbm': True,
                        'do_median_blur': False,
                        'use_mode_HH': True,
                        # ... (可以从 image_undistort/README.md 添加更多参数)
                    }],
                    
                    # 话题重映射
                    # (它会自动订阅 'rect/...' 话题，
                    # 由于在同一容器中，这将是零拷贝)
                    remappings=[
                        # 'rect/first/image',
                        # 'rect/second/image',
                        # ...
                        # 最终输出
                        # ('disparity/image', '/seeker/disparity_image'),
                        # ('pointcloud', '/seeker/pointcloud'),
                    ]
                )
            ]
    )

    # 4. 模拟 <group ns="...">
    #    我们创建一个 GroupAction 并使用 PushRosNamespace
    #    来确保 *所有* 在此组内的节点 (即我们的容器) 都在命名空间中。
    #    (注意：我们也在容器上设置了 namespace，这是双重保险)
    namespaced_container_group = GroupAction(
        actions=[
            PushRosNamespace(LaunchConfiguration('namespace')),
            container,
        ]
    )

    # 5. 返回 LaunchDescription
    return LaunchDescription([
        declare_mav_name_arg,
        declare_namespace_arg,
        declare_process_nth_frame_arg,
        declare_calib_path_arg,
        
        namespaced_container_group
    ])
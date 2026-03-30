import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer, PushRosNamespace, Node
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

    declare_play_bag_arg = DeclareLaunchArgument(
        'play_bag', default_value='true',
        description='Set to true to play the bag file.')

    # 假设 'mav_startup' 也是一个 ROS 2 包
    mav_startup_share_dir = get_package_share_directory('mav_startup')
    default_calib_path = os.path.join(
        mav_startup_share_dir, 'parameters', 'mavs', 'ibis', 'camchain_p22031.yaml')

    declare_calib_path_arg = DeclareLaunchArgument(
        'stereo_params_camchain',
        default_value=default_calib_path,
        description='Path to Kalibr camchain YAML file.')

    declare_bag_file_arg = DeclareLaunchArgument(
        'bag_file', 
        default_value='/home/z/Datasets/2017-05-23-honggerberg/ibis-erl-2017-06-16-11-54-43.bag',
        description='Path to the bag file.')
    
    # 2. 迁移 <node pkg="rosbag" ...>
    #    使用 ExecuteProcess 启动 ros2 bag play
    rosbag_play_node = ExecuteProcess(
        cmd=['ros2', 'bag', 'play',
             '-r', '0.5', '--clock', LaunchConfiguration('bag_file')],
        output='screen',
        # 3. 迁移 if="$(arg play_bag)"
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('play_bag'), "' == 'true'"])
        )
    )

    # 4. 定义组件容器 (这是 ROS 2 的 "nodelet manager")
    dense_stereo_container = ComposableNodeContainer(
            name='dense_stereo_container',
            namespace='', # 命名空间将在 GroupAction 中应用
            package='rclcpp_components',
            executable='component_container',
            output='screen',
            
            # 5. [核心] 声明所有要组合的组件
            composable_node_descriptions=[
                
                # --- 组件 1: StereoUndistortNodelet ---
                ComposableNode(
                    package='image_undistort',
                    plugin='image_undistort::StereoUndistortNodelet', 
                    name='dense_stereo', # ROS 1 <node name="...">
                    
                    parameters=[
                        {
                            'input_camera_info_from_ros_params': True,
                            'first_camera_namespace': 'cam2',
                            'second_camera_namespace': 'cam3',
                            'first_output_frame': 'cam2_rect',
                            'second_output_frame': 'cam3_rect',
                            'scale': 1.0,
                            'rename_radtan_plumb_bob': True,
                            'process_every_nth_frame': LaunchConfiguration('process_every_nth_frame'),
                        },
                        LaunchConfiguration('stereo_params_camchain') # 加载 Kalibr YAML
                    ],
                    
                    remappings=[
                        ('raw/first/image', 'cam2/image_raw'),
                        ('raw/second/image', 'cam3/image_raw'),
                        ('raw/first/camera_info', 'cam2/camera_info'),
                        ('raw/second/camera_info', 'cam3/camera_info'),
                        # (它将发布 rect/... 话题)
                    ]
                ),
                
                # --- 组件 2: DepthNodelet ---
                ComposableNode(
                    package='image_undistort',
                    plugin='image_undistort::DepthNodelet', 
                    name='depth_node',
                    
                    # (此 launch 文件没有为 depth 传递特定参数, 
                    #  但我们可以在这里添加，例如:)
                    # parameters=[{'use_sgbm': True, ...}]
                    
                    # 6. [关键] 显式重映射，以连接两个组件并修复 ROS 1 话题不匹配
                    remappings=[
                        # 订阅 StereoUndistortNodelet 的输出
                        ('rect/first/image', 'dense_stereo/rect/first/image'),
                        ('rect/second/image', 'dense_stereo/rect/second/image'),
                        ('rect/first/camera_info', 'dense_stereo/rect/first/camera_info'),
                        ('rect/second/camera_info', 'dense_stereo/rect/second/camera_info'),
                        
                        # 重映射输出，以匹配 voxblox_node 的期望
                        ('pointcloud', 'dense_stereo/pointcloud'),
                        ('freespace_pointcloud', 'dense_stereo/freespace_pointcloud'),
                    ]
                )
            ]
    )

    # 7. 迁移 <node name="voxblox_node" ...>
    #    这是一个独立的标准节点
    voxblox_node = Node(
        package='voxblox_ros',
        executable='voxblox_node',
        name='voxblox_node',
        output='screen',
        arguments=['-alsologtostderr'],
        
        # 8. 迁移 voxblox 的 <param>
        parameters=[{
            'tsdf_voxel_size': 0.05,
            'tsdf_voxels_per_side': 16,
            'voxel_carving_enabled': True,
            'color_mode': 'colors',
            'max_ray_length_m': 5.0,
            'use_tf_transforms': True,
            'verbose': True,
            'world_frame': 'arena',
            'update_mesh_every_n_sec': 0.5,
            'generate_esdf': False,
            'method': 'merged',
            'use_const_weight': False,
        }],
        
        # 9. 迁移 voxblox 的 <remap>
        remappings=[
            ('pointcloud', 'dense_stereo/pointcloud'),
            ('freespace_pointcloud', 'dense_stereo/freespace_pointcloud'),
        ]
    )

    # 10. 迁移 <group ns="...">
    #     将容器和 voxblox 节点都放入命名空间
    namespaced_nodes_group = GroupAction(
        actions=[
            PushRosNamespace(LaunchConfiguration('namespace')),
            dense_stereo_container,
            voxblox_node,
        ]
    )

    # 11. 返回 LaunchDescription
    return LaunchDescription([
        declare_mav_name_arg,
        declare_namespace_arg,
        declare_process_nth_frame_arg,
        declare_play_bag_arg,
        declare_calib_path_arg,
        declare_bag_file_arg,
        
        rosbag_play_node,
        namespaced_nodes_group
    ])
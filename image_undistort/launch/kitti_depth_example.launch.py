import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    # 1. 声明启动参数 (替代 <arg>)
    declare_first_cam_arg = DeclareLaunchArgument(
        'first_camera_name', default_value='cam00',
        description='Topic prefix for the first camera.')
    
    declare_second_cam_arg = DeclareLaunchArgument(
        'second_camera_name', default_value='cam01',
        description='Topic prefix for the second camera.')

    declare_play_bag_arg = DeclareLaunchArgument(
        'play_bag', default_value='true',
        description='Set to true to play the bag file.')
    
    declare_bag_file_arg = DeclareLaunchArgument(
        'bag_file', 
        default_value='/home/z/Datasets/KITTI/2011_09_26/2011_09_26_drive_0035_sync/data.bag',
        description='Path to the bag file.')

    # 2. 迁移 <node pkg="rosbag" ...>
    rosbag_play_node = ExecuteProcess(
        cmd=['ros2', 'bag', 'play',
             '--clock', LaunchConfiguration('bag_file')],
        output='screen',
        # 3. 迁移 if="$(arg play_bag)"
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('play_bag'), "' == 'true'"])
        )
    )

    # 4. 迁移 <node name="depth_node" ...>
    #    我们启动一个容器，并加载 DepthNodelet 组件
    depth_container = ComposableNodeContainer(
            name='depth_node_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            output='screen',
            composable_node_descriptions=[
                ComposableNode(
                    package='image_undistort',
                    plugin='image_undistort::DepthNodelet', 
                    name='depth_node',
                    
                    remappings=[
                        # 迁移 <remap>
                        ('rect/first/image', [LaunchConfiguration('first_camera_name'), '/image_raw']),
                        ('rect/second/image', [LaunchConfiguration('second_camera_name'), '/image_raw']),
                        ('rect/first/camera_info', [LaunchConfiguration('first_camera_name'), '/camera_info']),
                        ('rect/second/camera_info', [LaunchConfiguration('second_camera_name'), '/camera_info']),
                        
                        # [迁移修正]
                        # 将此组件的输出重映射到 voxblox 期望的全局话题
                        ('pointcloud', '/pointcloud'),
                        ('freespace_pointcloud', '/freespace_pointcloud'),
                    ],
                    
                    # (可以从 README.md 添加 DepthNodelet 的参数)
                    # parameters=[{'use_sgbm': True, ...}]
                )
            ]
    )

    # 5. 迁移 <node name="voxblox_node" ...>
    voxblox_node = Node(
        package='voxblox_ros',
        executable='voxblox_node',
        name='voxblox_node',
        output='screen',
        arguments=['-alsologtostderr'],
        
        # 6. 迁移 <param>
        parameters=[{
            'use_freespace_pointcloud': True,
            'tsdf_voxel_size': 0.2,
            'tsdf_voxels_per_side': 16,
            'voxel_carving_enabled': True,
            'color_mode': 'colors',
            'max_ray_length_m': 30.0,
            'use_tf_transforms': True,
            'verbose': True,
            'world_frame': 'world',
            'output_mesh_as_pcl_mesh': True,
            'update_mesh_every_n_sec': 1.0,
            'mesh_filename': '/home/z/Desktop/mesh.ply',
            'generate_esdf': False,
            'method': 'merged',
            'use_const_weight': False,
        }],
        
        # 7. 迁移 <remap>
        #    (现在这些重映射是正确的，因为 depth_container 已发布到全局)
        remappings=[
            ('pointcloud', '/pointcloud'),
            ('freespace_pointcloud', '/freespace_pointcloud'),
        ]
    )

    # 8. 返回 LaunchDescription
    return LaunchDescription([
        declare_first_cam_arg,
        declare_second_cam_arg,
        declare_play_bag_arg,
        declare_bag_file_arg,
        
        rosbag_play_node,
        depth_container,
        voxblox_node
    ])
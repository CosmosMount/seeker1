import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    # ========================================================================
    # 1. 声明启动参数
    # ========================================================================
    
    # 相机命名
    declare_cam0_arg = DeclareLaunchArgument('front_left_camera_name', default_value='cam0')
    declare_cam1_arg = DeclareLaunchArgument('front_right_camera_name', default_value='cam1')
    declare_cam2_arg = DeclareLaunchArgument('back_right_camera_name', default_value='cam2')
    declare_cam3_arg = DeclareLaunchArgument('back_left_camera_name', default_value='cam3')
    
    # 处理参数
    declare_scale_arg = DeclareLaunchArgument('scale', default_value='0.1')
    declare_process_rate_arg = DeclareLaunchArgument('process_every_nth_frame', default_value='1')
    declare_publish_tf_arg = DeclareLaunchArgument('publish_tf', default_value='true')
    
    # Seeker 驱动参数
    declare_use_img_trans_arg = DeclareLaunchArgument('use_image_transport', default_value='true') # 注意：这里默认是 true
    declare_pub_disp_img_arg = DeclareLaunchArgument('pub_disparity_img', default_value='true')
    declare_pub_disp_arg = DeclareLaunchArgument('pub_disparity', default_value='true')
    declare_pub_imu_arg = DeclareLaunchArgument('pub_imu', default_value='true')
    declare_time_sync_arg = DeclareLaunchArgument('time_sync', default_value='true')
    
    # 配置文件路径
    seeker_share_dir = get_package_share_directory('seeker')
    declare_config_arg = DeclareLaunchArgument('config', default_value='seeker_omni_depth')
    
    default_yaml_path = os.path.join(
        seeker_share_dir, 'config', 'seeker_omni_depth', 'kalibr_cam_chain.yaml'
    )
    declare_yaml_path_arg = DeclareLaunchArgument(
        'stereo_params_camchain',
        default_value=default_yaml_path,
        description='Path to Kalibr camchain YAML file'
    )

    # ========================================================================
    # 2. 辅助函数：生成组件配置
    # ========================================================================

    # 生成 StereoUndistortNodelet 配置
    def get_undistort_node(name, first_ns, second_ns, first_out, second_out, remappings):
        return ComposableNode(
            package='image_undistort',
            plugin='image_undistort::StereoUndistortNodelet',
            name=name,
            parameters=[
                {
                    'publish_tf': LaunchConfiguration('publish_tf'),
                    'input_camera_info_from_ros_params': True,
                    'first_camera_namespace': first_ns,
                    'second_camera_namespace': second_ns,
                    'scale': LaunchConfiguration('scale'),
                    'process_every_nth_frame': LaunchConfiguration('process_every_nth_frame'),
                    'first_output_frame': first_out,
                    'second_output_frame': second_out,
                },
                LaunchConfiguration('stereo_params_camchain')
            ],
            remappings=remappings
        )

    # 生成 PointCloud2 组件配置
    # 我们使用标准的 stereo_image_proc::PointCloudNode
    def get_pointcloud_node(name, remap_prefix):
        return ComposableNode(
            package='stereo_image_proc', # 使用标准包
            plugin='stereo_image_proc::PointCloudNode', # 标准点云组件
            name=name,
            parameters=[{'approximate_sync': True}],
            remappings=[
                # 输入: 连接到 undistort 节点的输出
                ('left/image_rect_color',  [remap_prefix, '/left/image_raw']),
                ('left/camera_info',       [remap_prefix, '/left/camera_info']),
                ('right/camera_info',      [remap_prefix, '/right/camera_info']),
                ('disparity',              [remap_prefix, '/disparity']), # 假设 disparity 话题也需要连接
                # 输出
                ('points2',                [remap_prefix, '/points2']),
            ]
        )

    # ========================================================================
    # 3. 定义所有组件
    # ========================================================================

    # --- A. 驱动组件 ---
    seeker_node = ComposableNode(
        package='seeker',
        plugin='SeekRosNode',
        name='seeker_node',
        parameters=[{
            'use_image_transport': LaunchConfiguration('use_image_transport'),
            'pub_disparity_img': LaunchConfiguration('pub_disparity_img'),
            'pub_disparity': LaunchConfiguration('pub_disparity'),
            'pub_imu': LaunchConfiguration('pub_imu'),
            'time_sync': LaunchConfiguration('time_sync'),
            'imu_link': 'imu',
        }]
    )

    # --- B. Front (Undistort + PointCloud) ---
    front_undistort = get_undistort_node(
        'front_stereo_undistort',
        LaunchConfiguration('front_left_camera_name'),
        LaunchConfiguration('front_right_camera_name'),
        'depth0', 'depth0',
        [
            ('raw/first/image', '/fisheye/left/image_raw'),
            ('raw/second/image', '/fisheye/right/image_raw'),
            ('rect/first/camera_info', '/front/left/camera_info'),
            ('rect/second/camera_info', '/front/right/camera_info'),
            ('rect/first/image', '/front/left/image_raw'),
            ('rect/second/image', '/front/right/image_raw'),
            ('raw/first/camera_info', '/front/raw/first/camera_info'),
            ('raw/second/camera_info', '/front/raw/second/camera_info'),
        ]
    )
    front_pcl = get_pointcloud_node('front_point_cloud2', '/front')

    # --- C. Right (Undistort + PointCloud) ---
    right_undistort = get_undistort_node(
        'right_stereo_undistort',
        LaunchConfiguration('front_right_camera_name'),
        LaunchConfiguration('back_right_camera_name'),
        'cam2r', 'cam3l',
        [
            ('raw/first/image', '/fisheye/right/image_raw'),
            ('raw/second/image', '/fisheye/bright/image_raw'),
            ('rect/first/camera_info', '/right/left/camera_info'),
            ('rect/second/camera_info', '/right/right/camera_info'),
            ('rect/first/image', '/right/left/image_raw'),
            ('rect/second/image', '/right/right/image_raw'),
            ('raw/first/camera_info', '/right/raw/first/camera_info'),
            ('raw/second/camera_info', '/right/raw/second/camera_info'),
        ]
    )
    right_pcl = get_pointcloud_node('right_point_cloud2', '/right')

    # --- D. Left (Undistort + PointCloud) ---
    left_undistort = get_undistort_node(
        'left_stereo_undistort',
        LaunchConfiguration('back_left_camera_name'),
        LaunchConfiguration('front_left_camera_name'),
        'cam6r', 'cam7l',
        [
            ('raw/first/image', '/fisheye/bleft/image_raw'),
            ('raw/second/image', '/fisheye/left/image_raw'),
            ('rect/first/camera_info', '/left/left/camera_info'),
            ('rect/second/camera_info', '/left/right/camera_info'),
            ('rect/first/image', '/left/left/image_raw'),
            ('rect/second/image', '/left/right/image_raw'),
            ('raw/first/camera_info', '/left/raw/first/camera_info'),
            ('raw/second/camera_info', '/left/raw/second/camera_info'),
        ]
    )
    left_pcl = get_pointcloud_node('left_point_cloud2', '/left')

    # --- E. Back (Undistort + PointCloud) ---
    back_undistort = get_undistort_node(
        'back_stereo_undistort',
        LaunchConfiguration('back_right_camera_name'),
        LaunchConfiguration('back_left_camera_name'),
        'cam4r', 'cam5l',
        [
            ('raw/first/image', '/fisheye/bright/image_raw'),
            ('raw/second/image', '/fisheye/bleft/image_raw'),
            ('rect/first/camera_info', '/back/left/camera_info'),
            ('rect/second/camera_info', '/back/right/camera_info'),
            ('rect/first/image', '/back/left/image_raw'),
            ('rect/second/image', '/back/right/image_raw'),
            ('raw/first/camera_info', '/back/raw/first/camera_info'),
            ('raw/second/camera_info', '/back/raw/second/camera_info'),
        ]
    )
    back_pcl = get_pointcloud_node('back_point_cloud2', '/back')

    # ========================================================================
    # 4. 创建容器并添加所有 9 个组件
    # ========================================================================
    container = ComposableNodeContainer(
        name='point_cloud_manager',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt', # 多线程容器
        output='screen',
        composable_node_descriptions=[
            seeker_node,
            front_undistort, front_pcl,
            right_undistort, right_pcl,
            left_undistort, left_pcl,
            back_undistort, back_pcl
        ]
    )

    return LaunchDescription([
        declare_cam0_arg, declare_cam1_arg, declare_cam2_arg, declare_cam3_arg,
        declare_scale_arg, declare_process_rate_arg, declare_publish_tf_arg,
        declare_use_img_trans_arg, declare_pub_disp_img_arg, declare_pub_disp_arg,
        declare_pub_imu_arg, declare_time_sync_arg, 
        declare_config_arg, declare_yaml_path_arg,
        
        container
    ])
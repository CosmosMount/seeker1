import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def resolve_default_camchain_path():
    seeker_share_dir = get_package_share_directory('seeker')
    workspace_root = os.path.abspath(
        os.path.join(seeker_share_dir, '..', '..', '..', '..')
    )
    candidates = [
        os.path.join(workspace_root, 'src', 'seeker1', 'seeker1', 'config', 'kalibr_cam_chain.yaml'),
        os.path.join(seeker_share_dir, 'config', 'kalibr_cam_chain.yaml'),
        os.path.join(seeker_share_dir, 'config', 'seeker_omni_depth', 'kalibr_cam_chain.yaml'),
    ]

    for path in candidates:
        if os.path.exists(path):
            return path

    return candidates[0]

def generate_launch_description():

    # ========================================================================
    # 1. 声明启动参数 (Arguments)
    # ========================================================================
    
    # 相机命名参数
    declare_cam0_arg = DeclareLaunchArgument('front_left_camera_name', default_value='cam0')
    declare_cam1_arg = DeclareLaunchArgument('front_right_camera_name', default_value='cam1')
    declare_cam2_arg = DeclareLaunchArgument('back_right_camera_name', default_value='cam2')
    declare_cam3_arg = DeclareLaunchArgument('back_left_camera_name', default_value='cam3')
    
    # 处理参数
    declare_scale_arg = DeclareLaunchArgument('scale', default_value='0.1') # ROS 1 默认为 0.1
    declare_process_rate_arg = DeclareLaunchArgument('process_every_nth_frame', default_value='1')
    declare_publish_tf_arg = DeclareLaunchArgument('publish_tf', default_value='true')
    
    # Seeker 驱动参数
    declare_use_img_trans_arg = DeclareLaunchArgument('use_image_transport', default_value='false')
    declare_pub_disp_img_arg = DeclareLaunchArgument('pub_disparity_img', default_value='true')
    declare_pub_disp_arg = DeclareLaunchArgument('pub_disparity', default_value='true')
    declare_pub_imu_arg = DeclareLaunchArgument('pub_imu', default_value='true')
    declare_time_sync_arg = DeclareLaunchArgument('time_sync', default_value='true')
    
    
    # 构建配置文件的完整路径
    # 在 Python launch 中，复杂的路径拼接比较繁琐，这里我们简化处理：
    # 假设 config 文件夹已安装到 share/seeker/config 下
    
    # 如果您需要动态路径，可以使用 PythonExpression 或 PathJoinSubstitution
    # 默认读取当前包 share 下的 config/kalibr_cam_chain.yaml
    default_yaml_path = "/home/iadc/Desktop/seeker_ws/src/seeker1/seeker1/config/kalibr_cam_chain.yaml"
    
    declare_yaml_path_arg = DeclareLaunchArgument(
        'stereo_params_camchain',
        default_value=default_yaml_path,
        description='Path to Kalibr camchain YAML file'
    )

    # ========================================================================
    # 2. 定义组件配置
    # ========================================================================

    def get_undistort_params(first_cam_ns, second_cam_ns, first_out_frame, second_out_frame):
        return [
            {
                'publish_tf': LaunchConfiguration('publish_tf'),
                'input_camera_info_from_ros_params': True,
                # --- 关键修改：改为空字符串，因为 YAML 里已经平铺了 ---
                'first_camera_namespace': '', 
                'second_camera_namespace': '',
                # ----------------------------------------------
                'scale': LaunchConfiguration('scale'),
                'process_every_nth_frame': LaunchConfiguration('process_every_nth_frame'),
                'first_output_frame': first_out_frame,
                'second_output_frame': second_out_frame,
            },
            LaunchConfiguration('stereo_params_camchain') 
        ]

    # --- 组件 A: Seeker 驱动 ---
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

    # --- 组件 B: 前向立体解畸变 (Front) ---
    front_undistort = ComposableNode(
        package='image_undistort',
        plugin='image_undistort::StereoUndistortNodelet',
        name='front_stereo_undistort',
        parameters=get_undistort_params(
            LaunchConfiguration('front_left_camera_name'),
            LaunchConfiguration('front_right_camera_name'),
            'depth0', 'depth0'
        ),
        remappings=[
            # 输入 (来自 Seeker)
            ('raw/first/image', '/fisheye/left/image_raw'),
            ('raw/second/image', '/fisheye/right/image_raw'),
            # 输出
            ('rect/first/camera_info', '/front/left/camera_info'),
            ('rect/second/camera_info', '/front/right/camera_info'),
            ('rect/first/image', '/front/left/image_raw'),
            ('rect/second/image', '/front/right/image_raw'),
            ('raw/first/camera_info', '/front/raw/first/camera_info'),
            ('raw/second/camera_info', '/front/raw/second/camera_info'),
        ]
    )

    # --- 组件 C: 右侧立体解畸变 (Right) ---
    right_undistort = ComposableNode(
        package='image_undistort',
        plugin='image_undistort::StereoUndistortNodelet',
        name='right_stereo_undistort',
        parameters=get_undistort_params(
            LaunchConfiguration('front_right_camera_name'),
            LaunchConfiguration('back_right_camera_name'),
            'cam2r', 'cam3l'
        ),
        remappings=[
            # 输入
            ('raw/first/image', '/fisheye/right/image_raw'),
            ('raw/second/image', '/fisheye/bright/image_raw'),
            # 输出
            ('rect/first/camera_info', '/right/left/camera_info'),
            ('rect/second/camera_info', '/right/right/camera_info'),
            ('rect/first/image', '/right/left/image_raw'),
            ('rect/second/image', '/right/right/image_raw'),
            ('raw/first/camera_info', '/right/raw/first/camera_info'),
            ('raw/second/camera_info', '/right/raw/second/camera_info'),
        ]
    )

    # --- 组件 D: 左侧立体解畸变 (Left) ---
    left_undistort = ComposableNode(
        package='image_undistort',
        plugin='image_undistort::StereoUndistortNodelet',
        name='left_stereo_undistort',
        parameters=get_undistort_params(
            LaunchConfiguration('back_left_camera_name'),
            LaunchConfiguration('front_left_camera_name'),
            'cam6r', 'cam7l'
        ),
        remappings=[
            # 输入
            ('raw/first/image', '/fisheye/bleft/image_raw'),
            ('raw/second/image', '/fisheye/left/image_raw'),
            # 输出
            ('rect/first/camera_info', '/left/left/camera_info'),
            ('rect/second/camera_info', '/left/right/camera_info'),
            ('rect/first/image', '/left/left/image_raw'),
            ('rect/second/image', '/left/right/image_raw'),
            ('raw/first/camera_info', '/left/raw/first/camera_info'),
            ('raw/second/camera_info', '/left/raw/second/camera_info'),
        ]
    )

    # --- 组件 E: 后向立体解畸变 (Back) ---
    back_undistort = ComposableNode(
        package='image_undistort',
        plugin='image_undistort::StereoUndistortNodelet',
        name='back_stereo_undistort',
        parameters=get_undistort_params(
            LaunchConfiguration('back_right_camera_name'),
            LaunchConfiguration('back_left_camera_name'),
            'cam4r', 'cam5l'
        ),
        remappings=[
            # 输入
            ('raw/first/image', '/fisheye/bright/image_raw'),
            ('raw/second/image', '/fisheye/bleft/image_raw'),
            # 输出
            ('rect/first/camera_info', '/back/left/camera_info'),
            ('rect/second/camera_info', '/back/right/camera_info'),
            ('rect/first/image', '/back/left/image_raw'),
            ('rect/second/image', '/back/right/image_raw'),
            ('raw/first/camera_info', '/back/raw/first/camera_info'),
            ('raw/second/camera_info', '/back/raw/second/camera_info'),
        ]
    )

    # ========================================================================
    # 3. 创建容器并添加所有组件
    # ========================================================================
    container = ComposableNodeContainer(
        name='undistort_manager',
        namespace='',
        package='rclcpp_components',
        executable='component_container', # 推荐使用多线程容器 component_container_mt
        output='screen',
        composable_node_descriptions=[
            seeker_node,
            front_undistort,
            right_undistort,
            left_undistort,
            back_undistort
        ]
    )

    return LaunchDescription([
        declare_cam0_arg, declare_cam1_arg, declare_cam2_arg, declare_cam3_arg,
        declare_scale_arg, declare_process_rate_arg, declare_publish_tf_arg,
        declare_use_img_trans_arg, declare_pub_disp_img_arg, declare_pub_disp_arg,
        declare_pub_imu_arg, declare_time_sync_arg, 
        declare_yaml_path_arg,
        
        container
    ])
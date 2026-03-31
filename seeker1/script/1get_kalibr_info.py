import usb.core
import usb.util
import numpy as np
import io
import signal
import threading
import time
import struct
from ctypes import *
import asyncio
import ruamel.yaml
import sys
import os

ruamelyaml = ruamel.yaml.YAML()
ruamelyaml.default_flow_style = None
np.set_printoptions(suppress =True)


def get_default_camchain_path():
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    return os.path.join(project_root, 'config', 'kalibr_cam_chain.yaml')


def as_ros2_parameter_file(param_dict):
    return {'/**': {'ros__parameters': param_dict}}

VENDOR_ID = 0x2207
PRODUCT_ID = 0x0000

dev = usb.core.find(idVendor=VENDOR_ID,idProduct=PRODUCT_ID)

ep2 = dev[0].interfaces()[6].endpoints()[2]
ep3 = dev[0].interfaces()[6].endpoints()[3]

def inv_T(T):
    # 求逆操作并保持左下三个元素为0
    R = T[:3, :3]
    t = T[:3, 3]
    R_inv = np.linalg.inv(R)
    t_inv = np.dot(-R_inv, t)
    T_inv = np.mat(np.zeros((4, 4)))
    T_inv[:3, :3] = R_inv
    T_inv[:3, 3] = t_inv
    T_inv[3, 3] = 1
    return T_inv

class ParamHeader(Structure):
    _fields_ = [
        ("reserve", c_uint64),
        ("sec", c_uint64),
        ("nsec", c_uint64),
        ("seq", c_uint32),
        ("reserve2", c_uint32),
    ]
    _pack_ = 1

# 定义相机校准结构体
class ParamCamCali(Structure):
    _fields_ = [
        ("cam_id", c_uint8),
        ("camera_model", c_uint32),
        ("T_cam_imu_se3_qw", c_double),
        ("T_cam_imu_se3_qx", c_double),
        ("T_cam_imu_se3_qy", c_double),
        ("T_cam_imu_se3_qz", c_double),
        ("T_cam_imu_se3_x", c_double),
        ("T_cam_imu_se3_y", c_double),
        ("T_cam_imu_se3_z", c_double),
        ("T_cn_cnm1_se3_qw", c_double),
        ("T_cn_cnm1_se3_qx", c_double),
        ("T_cn_cnm1_se3_qy", c_double),
        ("T_cn_cnm1_se3_qz", c_double),
        ("T_cn_cnm1_se3_x", c_double),
        ("T_cn_cnm1_se3_y", c_double),
        ("T_cn_cnm1_se3_z", c_double),
        ("distortion_coeffs_k1", c_double),
        ("distortion_coeffs_k2", c_double),
        ("distortion_coeffs_p1", c_double),
        ("distortion_coeffs_p2", c_double),
        ("distortion_coeffs_k3", c_double),
        ("intrinsics_xi", c_double),
        ("intrinsics_fx", c_double),
        ("intrinsics_fy", c_double),
        ("intrinsics_cx", c_double),
        ("intrinsics_cy", c_double),
        ("resolution_width", c_uint32),
        ("resolution_height", c_uint32),
    ]
    _pack_ = 1

# 定义设备校准结构体
class ParamDevCali(Structure):
    _fields_ = [
        # 假设 param_header_t 是一个已经定义的结构体，这里需要替换为实际的定义
        ("header", ParamHeader),  # 示例：用64字节的字符数组代替
        ("cali_flag", c_uint64),
        ("fx", c_uint32),
        ("fy", c_uint32),
        ("cam", ParamCamCali * 4),
    ]
    _pack_ = 1

# 定义设备参数联合体
class DevParamData(Union):
    _fields_ = [
        ("cali", ParamDevCali),
        ("pad", c_uint8*1688)
    ]
    _pack_ = 1
    # _align_ = 1

# 定义设备参数结构体
class DeviceParam(Structure):
    _fields_ = [
        # 假设 dev_param_type_t 是一个枚举或整数类型，这里需要替换为实际的定义
        ("protocol_version", c_uint8),  # 示例：用整数代替
        ("protocol_type", c_uint8),  # 示例：用整数代替
        ("reverse1", c_uint8),  # 示例：用整数代替
        ("reverse2", c_uint8),  # 示例：用整数代替
        ("type", c_uint32),  # 示例：用整数代替
        # ("resver1", c_uint32),
        ("param", DevParamData),
    ]
    _pack_ = 1

def quaternion_to_rotation_matrix(qw, qx, qy, qz, tx, ty, tz):
    m00 = 1 - 2*qy**2 - 2*qz**2
    m01 = 2*qx*qy - 2*qz*qw
    m02 = 2*qx*qz + 2*qy*qw
    m10 = 2*qx*qy + 2*qz*qw
    m11 = 1 - 2*qx**2 - 2*qz**2
    m12 = 2*qy*qz - 2*qx*qw
    m20 = 2*qx*qz - 2*qy*qw
    m21 = 2*qy*qz + 2*qx*qw
    m22 = 1 - 2*qx**2 - 2*qy**2
    return np.array([
        [m00, m01, m02, tx],
        [m10, m11, m12, ty],
        [m20, m21, m22, tz],
        [0,   0,   0,   1]
    ], dtype=np.float64)

def get_camera_info(cam):
    output = dict()
    
    # --- 关键修改 1：将 4x4 矩阵扁平化为 16 元素的列表 ---
    t_cam_imu = quaternion_to_rotation_matrix(
        cam.T_cam_imu_se3_qw, cam.T_cam_imu_se3_qx,
        cam.T_cam_imu_se3_qy, cam.T_cam_imu_se3_qz,
        cam.T_cam_imu_se3_x, cam.T_cam_imu_se3_y, cam.T_cam_imu_se3_z
    )
    output["T_cam_imu"] = t_cam_imu.flatten().tolist()
    
    t_imu_cam = inv_T(np.mat(t_cam_imu))
    output["T_imu_cam"] = np.array(t_imu_cam).flatten().tolist()
    
    t_cn_cnm1 = quaternion_to_rotation_matrix(
        cam.T_cn_cnm1_se3_qw, cam.T_cn_cnm1_se3_qx,
        cam.T_cn_cnm1_se3_qy, cam.T_cn_cnm1_se3_qz,
        cam.T_cn_cnm1_se3_x, cam.T_cn_cnm1_se3_y, cam.T_cn_cnm1_se3_z
    )
    output["T_cn_cnm1"] = t_cn_cnm1.flatten().tolist()

    # --- 关键修改 2：确保所有数值都是 float (带 .0) ---
    output["distortion_coeffs"] = [float(cam.distortion_coeffs_k1), float(cam.distortion_coeffs_k2),
                                  float(cam.distortion_coeffs_p1), float(cam.distortion_coeffs_p2),
                                  float(cam.distortion_coeffs_k3)]
    
    if (cam.camera_model == 4):
        # omni 模型需要 xi, fx, fy, cx, cy
        output["intrinsics"] = [float(cam.intrinsics_xi), float(cam.intrinsics_fx),
                                float(cam.intrinsics_fy), float(cam.intrinsics_cx),
                                float(cam.intrinsics_cy)]
    else:
        output["intrinsics"] = [float(cam.intrinsics_fx), float(cam.intrinsics_fy),
                                float(cam.intrinsics_cx), float(cam.intrinsics_cy)]
    
    # 强制分辨率为 float 列表，防止 ROS 2 插件解析失败
    output["resolution"] = [float(cam.resolution_width), float(cam.resolution_height)]

    if (cam.camera_model == 4):
        output["camera_model"] = "omni"
        output["distortion_model"] = "radtan"
    elif (cam.camera_model == 0):
        output["camera_model"] = "pinhole"
        output["distortion_model"] = "radtan"
        
    output["timeshift_cam_imu"] = 0.0
    return output

def get_kalibr_yaml(cali, path='/tmp/kalibr_cam_chain.yaml'):
    # 1. 提取四个相机的数据（扁平化矩阵已经在 get_camera_info 处理过了）
    c0 = get_camera_info(cali.cam[0])
    c1 = get_camera_info(cali.cam[1])
    c2 = get_camera_info(cali.cam[2])
    c3 = get_camera_info(cali.cam[3])

    # 2. 针对 launch 文件中的 4 个 StereoUndistortNodelet 节点名
    # 每个节点直接获得它需要的 intrinsics, distortion_coeffs 等
    # 这样节点一启动就能在“根目录”下找到参数，不需要再去 cam0/ 找
    final_data = {
        "/front_stereo_undistort": {
            "ros__parameters": {**c0, "first_camera_namespace": "", "second_camera_namespace": ""}
        },
        "/right_stereo_undistort": {
            "ros__parameters": {**c1, "first_camera_namespace": "", "second_camera_namespace": ""}
        },
        "/back_stereo_undistort": {
            "ros__parameters": {**c2, "first_camera_namespace": "", "second_camera_namespace": ""}
        },
        "/left_stereo_undistort": {
            "ros__parameters": {**c3, "first_camera_namespace": "", "second_camera_namespace": ""}
        }
    }

    print(f"Generating flattened ROS 2 parameters to {path}...")
    with open(path, 'w+') as g:
        ruamelyaml.dump(final_data, g)

async def cmd_get_cali_cam():
    device_param = DeviceParam(type=1234)
    device_param.protocol_version = 1
    device_param.protocol_type = 2
    device_param.type = 1145241863
    byte_array = bytearray(device_param)
    print("cmd_get_cali_cam")
    dev.write(ep3, byte_array, timeout=500)

def task2(output_path):
    test = 0
    asyncio.run(cmd_get_cali_cam())

    while (1):
        binary_data = dev.read(ep2, 2048, timeout=500)
        data = DeviceParam.from_buffer_copy(binary_data)

        if (data.type == 1145241863): # calicam
            print("get_kalibr_yaml")
            get_kalibr_yaml(data.param.cali, output_path)
            break

if __name__ == "__main__":
    output_path = sys.argv[1] if len(sys.argv) > 1 else get_default_camchain_path()
    thread2 = threading.Thread(target=task2, args=(output_path,))
    thread2.start()
    thread2.join()

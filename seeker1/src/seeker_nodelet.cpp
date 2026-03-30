#include "seeker.hpp"
#include <rclcpp/rclcpp.hpp> // 替换 ros/ros.h
#include <rclcpp_components/register_node_macro.hpp> // 替换 pluginlib
#include <sensor_msgs/msg/image.hpp> // .h -> /msg/.hpp
#include <sensor_msgs/msg/imu.hpp> // .h -> /msg/.hpp
#include <sensor_msgs/msg/camera_info.hpp> // .h -> /msg/.hpp
#include <image_transport/image_transport.hpp> // .h -> .hpp
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <stereo_msgs/msg/disparity_image.hpp> // .h -> /msg/.hpp
#include <std_msgs/msg/header.hpp> // 需要显式包含
#include <memory> // 用于 std::make_unique
#include <cassert>
class SeekRosNodeimpl {
public:
    // 构造函数接收 rclcpp::Node 指针，而不是两个 NodeHandle
    SeekRosNodeimpl(rclcpp::Node* node) :
        node_ptr_(node) {

        // 1. 读取参数 (使用 ROS 2 API: declare_parameter)
        use_image_transport_ = node_ptr_->declare_parameter<bool>("use_image_transport", true);
        pub_disparity_img_ = node_ptr_->declare_parameter<bool>("pub_disparity_img", false);
        pub_disparity_ = node_ptr_->declare_parameter<bool>("pub_disparity", true);
        pub_imu_ = node_ptr_->declare_parameter<bool>("pub_imu", true);
        time_sync_ = node_ptr_->declare_parameter<bool>("time_sync", true);
        imu_link_ = node_ptr_->declare_parameter<std::string>("imu_link", "imu_link");
        imu_topic_ = node_ptr_->declare_parameter<std::string>("imu_topic", "imu_data_raw");

        // 2. 初始化image_transport
        if (use_image_transport_) {
            std::shared_ptr<rclcpp::Node> node_shared(node_ptr_ , [](rclcpp::Node*){});
            it_ = std::make_unique<image_transport::ImageTransport>(node_shared);
        }

        // 3. 初始化设备 (无ROS API，保持不变)
        std::vector<seeker_device_t> devices = seek.find_devices();
        if (devices.empty()) {
            RCLCPP_ERROR(node_ptr_->get_logger(), "No Seeker Devices Found");
            return;
        }
        seek.open(devices[0]);

        // 4. 设置回调 (无ROS API，保持不变)
        seek.set_event_callback(std::bind(&SeekRosNodeimpl::onEvent, this, std::placeholders::_1, std::placeholders::_2));
        seek.set_mjpeg_callback(std::bind(&SeekRosNodeimpl::onMjpeg, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        seek.set_depth_callback(std::bind(&SeekRosNodeimpl::onDepth, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        if (time_sync_) {
            seek.set_timer_callback(std::bind(&SeekRosNodeimpl::onTimer, this, std::placeholders::_1, std::placeholders::_2));
        }

        // 5. 获取设备信息 (无ROS API，保持不变)
        seek.get_dev_info(sdev);

        // 6. 初始化发布者 (使用 ROS 2 API: create_publisher)
        const std::vector<std::string> image_topics = {
            "/fisheye/left/image_raw",
            "/fisheye/right/image_raw",
            "/fisheye/bright/image_raw",
            "/fisheye/bleft/image_raw",
        };
        const std::vector<std::string> depth_topics = {
            "/front/disparity/image_raw",
            "/right/disparity/image_raw",
            "/back/disparity/image_raw",
            "/left/disparity/image_raw"
        };
        const std::vector<std::string> disparity_topics = {
            "/front/disparity",
            "/right/disparity",
            "/back/disparity",
            "/left/disparity"
        };

        // 图像发布者
        for (size_t i = 0; i < sdev.dev_info.rgb_camera_number; ++i) {
            if (use_image_transport_) {
                image_pubs_it_.push_back(it_->advertise(image_topics[i], 1));
            } else {
                image_pubs_ros_.push_back(node_ptr_->create_publisher<sensor_msgs::msg::Image>(image_topics[i], 10));
            }
        }

        // 深度和视差发布者
        for (size_t i = 0; i < sdev.dev_info.depth_camera_number; ++i) {
            if (pub_disparity_img_) {
                depth_pubs_.push_back(node_ptr_->create_publisher<sensor_msgs::msg::Image>(depth_topics[i], 10));
            }
            if (pub_disparity_) {
                disparity_pubs_.push_back(node_ptr_->create_publisher<stereo_msgs::msg::DisparityImage>(disparity_topics[i], 10));
            }
        }

        // IMU发布者
        if (pub_imu_) {
            imu_pub_ = node_ptr_->create_publisher<sensor_msgs::msg::Imu>(imu_topic_, 200);
        }

        // 7. 启动设备流 (无ROS API，保持不变)
        seek.start_event_stream();
        seek.start_image_stream();
        seek.start_depth_stream();
    }

    ~SeekRosNodeimpl() {
        seek.stop_event_stream();
        seek.stop_image_stream();
        seek.stop_depth_stream();
        seek.close();
    }

private:
    void onDepth(event_header_t& pheader, const uint8_t* data, int len) {
        const int depth_camera_number = sdev.dev_info.depth_camera_number;
        const int height = sdev.dev_info.depth_resolution_height/depth_camera_number;
        const int width = sdev.dev_info.depth_resolution_width;
        
        std::vector<cv::Mat> images(depth_camera_number);
        for (int i = 0; i < depth_camera_number; i++) {
            images.at(i) = cv::Mat(height, width, CV_16UC1, (void *)(data+i*height*width*2));
        }
        std_msgs::msg::Header header;
        // 使用 rclcpp::Time, RCL_ROS_TIME 确保使用 ROS 时钟源
        // header.stamp = rclcpp::Time(pheader.sec, pheader.nsec, RCL_ROS_TIME); 
        // header.seq = pheader.seq;
        
        // 发布视差图像 (Image)
        for (size_t i = 0; i < depth_camera_number; ++i) {
            header.frame_id = "depth" + std::to_string(i);
            // cv_bridge::CvImage(...).toImageMsg() 在 ROS 2 中返回 std::unique_ptr
            auto img_msg = cv_bridge::CvImage(header, "16UC1", images[i]).toImageMsg();
            depth_pubs_[i]->publish(*img_msg);
        }
        
        // 发布disparity (DisparityImage)
        for (size_t i = 0; i < depth_camera_number; ++i) {
            const int DPP = 256/4;
            const double inv_dpp = 1.0 / DPP;
            
            // 使用 std::make_unique 创建消息
            auto disparity = std::make_unique<stereo_msgs::msg::DisparityImage>();
            
            sensor_msgs::msg::Image& dimage = disparity->image;
            dimage.height = images[i].rows;
            dimage.width = images[i].cols;
            dimage.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
            dimage.step = dimage.width * sizeof(float);
            dimage.data.resize(dimage.step * dimage.height);
            cv::Mat_<float> dmat(dimage.height, dimage.width, (float*)&dimage.data[0], dimage.step);
            images[i].convertTo(dmat, dmat.type(), inv_dpp, 0);
            
            // 使用 RCLCPP_ASSERT 并传入 logger
            // RCLCPP_ASSERT(node_ptr_->get_logger(), dmat.data == &dimage.data[0]);
            assert(dmat.data == &dimage.data[0]);

            // Stereo eventeters
            disparity->f = 320.0;
            disparity->t = 0.04625;

            // Disparity search range
            disparity->min_disparity = 0.0;
            disparity->max_disparity = 192;
            disparity->delta_d = inv_dpp;
            header.frame_id = "depth" + std::to_string(i);
            disparity->header = header;
            
            // 使用 std::move 发布 unique_ptr
            disparity_pubs_[i]->publish(std::move(disparity));
        }
        return;
    }

    void onEvent(event_header_t& header, device_event_t& event) {
        if (event.type == EVENT_TYPE_SENSOR_CUSTOM && pub_imu_) {
            auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();
            imu_msg->header.stamp = rclcpp::Time(header.sec, header.nsec, RCL_ROS_TIME);
            imu_msg->header.frame_id = imu_link_;
            
            imu_msg->angular_velocity.x = event.event.sensor_custom.angular_velocity_x;
            imu_msg->angular_velocity.y = event.event.sensor_custom.angular_velocity_y;
            imu_msg->angular_velocity.z = event.event.sensor_custom.angular_velocity_z;

            imu_msg->linear_acceleration.x = event.event.sensor_custom.linear_acceleration_x;
            imu_msg->linear_acceleration.y = event.event.sensor_custom.linear_acceleration_y;
            imu_msg->linear_acceleration.z = event.event.sensor_custom.linear_acceleration_z;

            imu_pub_->publish(std::move(imu_msg));
        }
    }

    void onImage(event_header_t& eheader, cv::Mat& frame) {
        const int cam_num = sdev.dev_info.rgb_camera_number;
        const int h = frame.rows / cam_num;
        const int w = frame.cols;
        std_msgs::msg::Header header;
        header.stamp = rclcpp::Time(eheader.sec, eheader.nsec);
        // [已修正] 删除 header.seq = eheader.seq; (ROS 2 没有 seq)

        for (int i = 0; i < cam_num; i++) {
            header.frame_id = "cam" + std::to_string(i);;
            
            // [已修正] 填回具体的裁剪逻辑，而不是 ...
            auto img_msg = cv_bridge::CvImage(
                header, 
                "bgr8", 
                frame(cv::Rect(0, i * h, w, h)) // 这里是核心逻辑：裁剪出第 i 个相机的图像
            ).toImageMsg();
            
            if (use_image_transport_) {
                image_pubs_it_[i].publish(*img_msg);
            } else {
                // 明确调用 publish(unique_ptr)
                image_pubs_ros_[i]->publish(*img_msg);
            }
        }
    }

    void onMjpeg(event_header_t& pheader, const uint8_t* data, int len) {
        cv::Mat frame = cv::imdecode(
            cv::Mat(1, len, CV_8UC1, 
                const_cast<uint8_t*>(data)), 
            cv::IMREAD_COLOR
        );
        onImage(pheader, frame);
    }

    bool onTimer(std::pair<uint64_t, uint64_t>& timer_get, std::pair<uint64_t, uint64_t>& timer_set) {
        // 使用 rclcpp::Node 的时钟
        rclcpp::Time now = node_ptr_->get_clock()->now();
        timer_set.first = now.seconds();
        timer_set.second = now.nanoseconds() % 1000000000; // 取纳秒部分
        return true;
    }

    // 成员变量: 替换 ros::NodeHandle
    rclcpp::Node* node_ptr_;

    SEEKNS::SEEKER seek;
    seeker_device_t sdev;
    std::unique_ptr<image_transport::ImageTransport> it_;
    
    // 成员变量: 替换 ros::Publisher
    std::vector<image_transport::Publisher> image_pubs_it_;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> image_pubs_ros_;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> depth_pubs_;
    std::vector<rclcpp::Publisher<stereo_msgs::msg::DisparityImage>::SharedPtr> disparity_pubs_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    
    // 参数
    bool use_image_transport_;
    bool pub_disparity_img_;
    bool pub_disparity_;
    bool pub_imu_;
    bool time_sync_;
    std::string imu_link_;
    std::string imu_topic_;
};

// -----------------------------------------------------------------------------
// ROS 2 Component 封装 (替换 Nodelet 封装)
// -----------------------------------------------------------------------------

// 移除 nodelet.h 和 pluginlib.h

// 继承自 rclcpp::Node
class SeekRosNode : public rclcpp::Node {
public:
    // onInit() 替换为构造函数
    explicit SeekRosNode(const rclcpp::NodeOptions & options)
        : Node("seeker_nodelet", options) { // "seeker_nodelet" 是节点名
        try {
            // 将 'this' (即 rclcpp::Node*) 传递给实现类
            v4l2_image_publish_ = std::make_unique<SeekRosNodeimpl>(this);
        } catch (const std::runtime_error& e) { // 捕获 const 引用
            // 使用 ROS 2 日志
            RCLCPP_ERROR(this->get_logger(), "%s", e.what());
        }
    }

private:
    std::unique_ptr<SeekRosNodeimpl> v4l2_image_publish_;
};

// 使用 ROS 2 宏注册 Component
RCLCPP_COMPONENTS_REGISTER_NODE(SeekRosNode)
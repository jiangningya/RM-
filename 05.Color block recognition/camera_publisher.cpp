#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <string>

using namespace std::chrono_literals;
using namespace cv;

class CameraPublisherNode : public rclcpp::Node {
public:
    CameraPublisherNode() : Node("camera_publisher"), 
                           save_interval_(1s),  // 保存间隔：1秒/张
                           last_save_time_(this->now()) {  // 上次保存时间
        // 创建图像发布者（保留原功能，可选）
        image_pub_ = image_transport::create_publisher(this, "camera/image_raw");

        // 初始化摄像头
        if (!init_camera()) {
            RCLCPP_FATAL(this->get_logger(), "摄像头初始化失败");
            rclcpp::shutdown();
            return;
        }

        // 定时器：33ms/帧（30fps采集）
        timer_ = this->create_wall_timer(
            33ms,
            std::bind(&CameraPublisherNode::capture_and_publish, this)
        );

        // 保存图片的路径（固定路径，方便检测节点读取）
        save_path_ = "/home/zhoujiang/桌面/camera_live.jpg";  // 替换为你的路径
        RCLCPP_INFO(this->get_logger(), "摄像头发布节点已启动！");
        RCLCPP_INFO(this->get_logger(), "实时保存图片路径：%s", save_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "图像发布话题（可选）：/camera/image_raw");
    }

    ~CameraPublisherNode() {
        cap_.release();
        RCLCPP_INFO(this->get_logger(), "摄像头发布节点已关闭");
    }

private:
    bool init_camera() {
        // 原初始化逻辑不变（尝试打开摄像头+设置参数）
        for (int i = 0; i < 4; i++) {
            cap_.open(i, CAP_V4L2);
            if (cap_.isOpened()) {
                RCLCPP_INFO(this->get_logger(), "成功打开摄像头设备 /dev/video%d", i);
                break;
            }
        }
        
        if (!cap_.isOpened()) {
            RCLCPP_FATAL(this->get_logger(), "无法打开任何摄像头设备！");
            RCLCPP_FATAL(this->get_logger(), "解决方法：1. sudo chmod 666 /dev/video0  2. 关闭占用程序");
            return false;
        }

        // 尝试设置参数（YUYV格式，640x480，30fps）
        cap_.set(CAP_PROP_FOURCC, VideoWriter::fourcc('Y', 'U', 'Y', 'V'));
        cap_.set(CAP_PROP_FRAME_WIDTH, 640);
        cap_.set(CAP_PROP_FRAME_HEIGHT, 480);
        cap_.set(CAP_PROP_FPS, 30);
        cap_.set(CAP_PROP_BUFFERSIZE, 1);  // 减少缓冲区，降低延迟

        // 验证参数
        double fps = cap_.get(CAP_PROP_FPS);
        int w = cap_.get(CAP_PROP_FRAME_WIDTH);
        int h = cap_.get(CAP_PROP_FRAME_HEIGHT);
        RCLCPP_INFO(this->get_logger(), "摄像头实际参数：%dx%d @ %.1f fps", w, h, fps);

        // 测试捕获一帧
        Mat test_frame;
        cap_ >> test_frame;
        if (test_frame.empty()) {
            RCLCPP_ERROR(this->get_logger(), "摄像头测试捕获失败");
            return false;
        }
        return true;
    }

    void capture_and_publish() {
        Mat frame;
        cap_ >> frame;  // 采集一帧图像
        
        if (frame.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "捕获到空帧，忽略");
            return;
        }

        // 核心：定时保存图像到本地（每save_interval_保存一次）
        auto now = this->now();
        if ((now - last_save_time_) >= save_interval_) {
            bool save_success = imwrite(save_path_, frame);  // 保存为JPG
            if (save_success) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                   "已保存图片到：%s", save_path_.c_str());
            } else {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                    "保存图片失败！检查路径权限");
            }
            last_save_time_ = now;  // 更新上次保存时间
        }

        // 保留原发布功能（可选，不影响文件保存）
        auto ros_msg = cv_bridge::CvImage(
            std_msgs::msg::Header(), "bgr8", frame
        ).toImageMsg();
        ros_msg->header.stamp = this->now();
        ros_msg->header.frame_id = "camera_frame";
        image_pub_.publish(*ros_msg);
    }

    VideoCapture cap_;
    image_transport::Publisher image_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string save_path_;  // 图片保存路径
    rclcpp::Duration save_interval_;  // 保存间隔
    rclcpp::Time last_save_time_;  // 上次保存时间
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraPublisherNode>());
    rclcpp::shutdown();
    return 0;
}

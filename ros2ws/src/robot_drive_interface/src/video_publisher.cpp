#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class VideoPublisher final : public rclcpp::Node {
public:
    VideoPublisher() : Node("video_publisher") {
        pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30),
            std::bind(&VideoPublisher::timer_callback, this)
        );

        cap_.open(0);

        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Camera not found");
            return;
        }

        // FORCE resolution to 640x480
        // cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        // cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

        // (Optional but recommended) Try to set FPS
        // cap_.set(cv::CAP_PROP_FPS, 2);

        RCLCPP_INFO(this->get_logger(),
                    "Camera opened at %.0fx%.0f @ %.0f FPS",
                    cap_.get(cv::CAP_PROP_FRAME_WIDTH),
                    cap_.get(cv::CAP_PROP_FRAME_HEIGHT),
                    cap_.get(cv::CAP_PROP_FPS));
    }

private:
    void timer_callback() {
        cv::Mat frame;
        cap_ >> frame;

        if (frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty frame");
            return;
        }

        const auto msg = cv_bridge::CvImage(
            std_msgs::msg::Header(),
            "bgr8",
            frame
        ).toImageMsg();

        pub_->publish(*msg);
    }

    cv::VideoCapture cap_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(const int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoPublisher>());
    rclcpp::shutdown();
    return 0;
}

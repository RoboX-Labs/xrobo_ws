#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class WebcamNode : public rclcpp::Node
{
public:
    WebcamNode()
        : Node("webcam_node")
    {
        // Create a publisher for the video frames
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);

        // Setting up the webcam
        cap_.set(cv::CAP_PROP_FPS, 30); // Set the desired frame rate
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640); // Set the desired frame width
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480); // Set the desired frame height

        // Open the webcam (default device 0)
        cap_.open(0);
        if (!cap_.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open webcam");
            rclcpp::shutdown();
        }

        // Create a timer to capture and publish frames
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33), std::bind(&WebcamNode::publish_frame, this));
    }

private:
    void publish_frame()
    {
        cv::Mat frame;
        cap_ >> frame; // Capture a frame from the webcam

        if (frame.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Captured empty frame");
            return;
        }

        // Convert the OpenCV frame to a ROS Image message
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        image_publisher_->publish(*msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WebcamNode>());
    rclcpp::shutdown();
    return 0;
}
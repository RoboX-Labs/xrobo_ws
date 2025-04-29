#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include "rs_imu.cpp"

class RealSenseNode : public rclcpp::Node
{
public:
    RealSenseNode() : Node("rs_d455_node")
    {
        // Create publishers for depth and color images
        depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>("depth_image", 10);
        color_pub_ = this->create_publisher<sensor_msgs::msg::Image>("color_image", 10);

        // Start the RealSense pipeline
        pipeline_.start();
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30), std::bind(&RealSenseNode::publish_frames, this));
    }

private:
    void publish_frames()
    {
        // Wait for frames
        rs2::frameset frames = pipeline_.wait_for_frames();

        // Get depth and color frames
        rs2::depth_frame depth_frame = frames.get_depth_frame();
        rs2::video_frame color_frame = frames.get_color_frame();

        // Convert depth frame to OpenCV format
        cv::Mat depth_image(cv::Size(depth_frame.get_width(), depth_frame.get_height()), CV_16UC1, (void *)depth_frame.get_data(), cv::Mat::AUTO_STEP);

        // Convert color frame to OpenCV format
        cv::Mat color_image(cv::Size(color_frame.get_width(), color_frame.get_height()), CV_8UC3, (void *)color_frame.get_data(), cv::Mat::AUTO_STEP);

        // Publish depth image
        auto depth_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono16", depth_image).toImageMsg();
        depth_msg->header.stamp = this->now();
        depth_pub_->publish(*depth_msg);

        // Publish color image
        auto color_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", color_image).toImageMsg();
        color_msg->header.stamp = this->now();
        color_pub_->publish(*color_msg);
    }

    rs2::pipeline pipeline_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RealSenseNode>());
    rclcpp::shutdown();
    return 0;
}
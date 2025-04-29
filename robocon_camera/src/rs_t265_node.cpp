#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <librealsense2/rs.hpp>

class RealSenseNode : public rclcpp::Node
{
public:
    RealSenseNode()
        : Node("rs_pose_node"), pipe_()
    {
        // configure and start RealSense
        rs2::config cfg;
        cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
        pipe_.start(cfg);

        // publisher
        pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("camera/pose", 10);

        // TF broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        // 30 Hz timer
        timer_ = create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&RealSenseNode::timer_callback, this));
    }

private:
    void timer_callback()
    {
        try
        {
            auto frames = pipe_.wait_for_frames();
            auto f = frames.first_or_default(RS2_STREAM_POSE);
            auto pose = f.as<rs2::pose_frame>().get_pose_data();
            
            RCLCPP_WARN(get_logger(), "Mapper confidence: %d", pose.mapper_confidence);
            RCLCPP_WARN(get_logger(), "Pose confidence: %d", pose.tracker_confidence);
            RCLCPP_WARN(get_logger(), "Position: (%f, %f, %f)", pose.translation.x, pose.translation.y, pose.translation.z);

            // 1) publish PoseStamped
            geometry_msgs::msg::PoseStamped msg;
            msg.header.stamp = this->now();
            msg.header.frame_id = "camera_t265_link";
            msg.pose.position.x = pose.translation.x;
            msg.pose.position.y = pose.translation.y;
            msg.pose.position.z = pose.translation.z;
            msg.pose.orientation.x = pose.rotation.x;
            msg.pose.orientation.y = pose.rotation.y;
            msg.pose.orientation.z = pose.rotation.z;
            msg.pose.orientation.w = pose.rotation.w;
            pub_->publish(msg);

            // 2) broadcast TF: world -> camera_t265_link
            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header.stamp = msg.header.stamp;
            tf_msg.header.frame_id = "world";
            tf_msg.child_frame_id = msg.header.frame_id;
            tf_msg.transform.translation.x = msg.pose.position.x;
            tf_msg.transform.translation.y = msg.pose.position.y;
            tf_msg.transform.translation.z = msg.pose.position.z;
            tf_msg.transform.rotation = msg.pose.orientation;
            tf_broadcaster_->sendTransform(tf_msg);
        }
        catch (const rs2::error &e)
        {
            RCLCPP_ERROR(get_logger(), "RealSense error: %s", e.what());
        }
    }

    rs2::pipeline pipe_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RealSenseNode>());
    rclcpp::shutdown();
    return 0;
}
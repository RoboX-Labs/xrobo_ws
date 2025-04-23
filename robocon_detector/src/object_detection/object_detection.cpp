#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

#include "robocon_detector/object_detection/inference.h"
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class ObjectDetection : public rclcpp::Node
{
public:
  ObjectDetection()
      : Node("object_detection")
  {
    publisher_ = this->create_publisher<vision_msgs::msg::Detection2DArray>("detection_result", 10);
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "camera/image_raw", 10, std::bind(&ObjectDetection::image_callback, this, std::placeholders::_1));

    // Assign the inference_ pointer in the constructor
    inference_ = std::make_unique<yolo::Inference>(model_path_, confidence_threshold_, NMS_threshold_);
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // Convert the ROS image message to OpenCV format
    auto cv_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    // Use the Inference class to perform object detection and publish results
    auto results = inference_->RunInference(cv_image->image);

    // Print the number of detections
    std::cout << "Number of detections: " << results.size() << std::endl;

    // Print the detection results
    for (const auto &result : results)
    {
      std::cout << "Class ID: " << result.class_id << ", Confidence: " << result.confidence
                << ", Box: (" << result.box.x << ", " << result.box.y << ", "
                << result.box.width << ", " << result.box.height << ")" << std::endl;
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  std::unique_ptr<yolo::Inference> inference_; // Use a smart pointer for inference_
  const std::string model_path_ = "src/robocon_detector/model/yolov8s_openvino_model/yolov8s.xml";
  const float confidence_threshold_ = 0.5;
  const float NMS_threshold_ = 0.5;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectDetection>());
  rclcpp::shutdown();
  return 0;
}
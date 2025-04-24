#include <chrono>
#include <memory>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "robocon_serial/SerialComHex.h"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses a fancy C++11 lambda
 * function to shorten the callback syntax, at the expense of making the
 * code somewhat more difficult to understand at first glance. */

class SerialComNode : public rclcpp::Node
{
public:
  SerialComNode()
      : Node("serial_com"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("data_raw", 10);
    auto timer_callback =
        [this]() -> void
    {
      auto cmd = Command(0x3E, 0xA2, 0x01, 0x04, std::vector<uint8_t>{this->count_, 0x02, 0x03, 0x04});
      auto message = std_msgs::msg::String();
      message.data = std::to_string(cmd.head) + std::to_string(this->count_++);
      // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());

      serial_->Send(cmd);
      if (serial_->Receive(cmd))
      {
        // std::cout << "Received: " << cmd << std::endl;
      }
      else
      {

        RCLCPP_WARN(this->get_logger(), "Failed to receive data.");
      }

      this->publisher_->publish(message);
    };
    serial_ = std::make_unique<SerialCom>("/dev/ttyACM0", LibSerial::BaudRate::BAUD_115200, 100);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    timer_ = this->create_wall_timer(50ms, timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  std::unique_ptr<SerialCom> serial_;
  size_t count_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialComNode>());
  rclcpp::shutdown();
  return 0;
}
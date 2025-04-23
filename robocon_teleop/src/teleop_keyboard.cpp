#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <string>
#include <algorithm>

#define MECANUM_MAX_LIN_VEL 1.0
#define MECANUM_MAX_ANG_VEL 1.0
#define LIN_VEL_STEP_SIZE 0.1
#define ANG_VEL_STEP_SIZE 0.1

std::string msg = R"(
Control Your Mecanum Robot!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

i/, : increase/decrease linear x velocity
j/l : increase/decrease linear y velocity
u/o : increase/decrease angular z velocity
k   : stop

CTRL-C to quit
)";

double constrain(double input, double low, double high)
{
  return std::min(std::max(input, low), high);
}

void print_vels(double linear_x, double linear_y, double angular_z)
{
  std::cout << "currently:\tlinear_x: " << linear_x
            << "\tlinear_y: " << linear_y
            << "\tangular_z: " << angular_z << std::endl;
}

char get_key()
{
  struct termios oldt, newt;
  char c;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  c = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return c;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("teleop_keyboard");
  auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  double target_linear_x = 0.0;
  double target_linear_y = 0.0;
  double target_angular_z = 0.0;

  std::cout << msg << std::endl;

  try
  {
    while (rclcpp::ok())
    {
      char key = get_key();

      if (key == 'i')
      {
        target_linear_x = constrain(target_linear_x + LIN_VEL_STEP_SIZE, -MECANUM_MAX_LIN_VEL, MECANUM_MAX_LIN_VEL);
      }
      else if (key == ',')
      {
        target_linear_x = constrain(target_linear_x - LIN_VEL_STEP_SIZE, -MECANUM_MAX_LIN_VEL, MECANUM_MAX_LIN_VEL);
      }

      if (key == 'u')
      {
        target_angular_z = constrain(target_angular_z + ANG_VEL_STEP_SIZE, -MECANUM_MAX_ANG_VEL, MECANUM_MAX_ANG_VEL);
      }
      else if (key == 'o')
      {
        target_angular_z = constrain(target_angular_z - ANG_VEL_STEP_SIZE, -MECANUM_MAX_ANG_VEL, MECANUM_MAX_ANG_VEL);
      }

      if (key == 'j')
      {
        target_linear_y = constrain(target_linear_y + LIN_VEL_STEP_SIZE, -MECANUM_MAX_ANG_VEL, MECANUM_MAX_ANG_VEL);
      }
      else if (key == 'l')
      {
        target_linear_y = constrain(target_linear_y - LIN_VEL_STEP_SIZE, -MECANUM_MAX_ANG_VEL, MECANUM_MAX_ANG_VEL);
      }

      if (key == 'k')
      {
        target_linear_x = 0.0;
        target_linear_y = 0.0;
        target_angular_z = 0.0;
      }
      else if (key == '\x03')
      { // CTRL-C
        break;
      }

      print_vels(target_linear_x, target_linear_y, target_angular_z);

      geometry_msgs::msg::Twist twist;
      twist.linear.x = target_linear_x;
      twist.linear.y = target_linear_y;
      twist.angular.z = target_angular_z;

      publisher->publish(twist);
    }
  }
  catch (const std::exception &e)
  {
    std::cerr << e.what() << std::endl;
  }

  rclcpp::shutdown();
  return 0;
}
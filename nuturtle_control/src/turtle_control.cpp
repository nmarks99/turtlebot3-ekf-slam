/// \file
/// \brief turtle_control node: controls the turtlebot with geometry_msgs/Twist messages

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "turtlelib/diff_drive.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

/// \brief nusim turtlebot simulation node
class NuturtleControl : public rclcpp::Node
{

public:
  NuturtleControl() : Node("nuturtle_control")
  {

    /// @brief Subscriber to cmd_vel topic
    _cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        std::bind(&NuturtleControl::cmd_vel_callback, this, _1));

    /// @brief Publisher to wheel_cmd topic
    _wheel_cmd_pub = this->create_publisher<nuturtlebot_msgs::msg::WheelCommands>("wheel_cmd", 10);

    /// @brief Timer
    _timer = this->create_wall_timer(500ms, std::bind(&NuturtleControl::timer_callback, this));
    count = 0;
  }

private:
  uint16_t count;
  rclcpp::TimerBase::SharedPtr _timer;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel_sub;
  rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr _wheel_cmd_pub;

  void cmd_vel_callback(const geometry_msgs::msg::Twist &V)
  {
    // TODO: DiffDrive::inverse_kinematics(V) to get wheel speeds
    RCLCPP_INFO_STREAM(this->get_logger(), "Vx = " << V.linear.x);
    RCLCPP_INFO_STREAM(this->get_logger(), "Vy = " << V.linear.y);
  }

  void timer_callback()
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Hi " << count);
    count++;
  }
};

/// \brief the main function to run the nusim node
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NuturtleControl>());
  rclcpp::shutdown();
  return 0;
}

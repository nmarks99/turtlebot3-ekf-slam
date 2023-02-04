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
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "turtlelib/diff_drive.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

/// @cond
const double DEFAULT_WHEEL_RADIUS = 0.033;
const double DEFAULT_TRACK_WIDTH = 0.160;
/// @endcond

/// \brief nusim turtlebot simulation node
class NuturtleControl : public rclcpp::Node
{

public:
  NuturtleControl() : Node("nuturtle_control")
  {

    this->declare_parameter<double>("wheel_radius", DEFAULT_WHEEL_RADIUS);
    this->declare_parameter<double>("track_width", DEFAULT_TRACK_WIDTH);
    wheel_radius = this->get_parameter("wheel_radius").get_value<double>();
    track_width = this->get_parameter("track_width").get_value<double>();

    /// @brief Subscriber to cmd_vel topic
    cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        std::bind(&NuturtleControl::cmd_vel_callback, this, _1));

    /// @brief Subscriber to sensor_data topic
    sensor_data_sub = this->create_subscription<nuturtlebot_msgs::msg::SensorData>(
        "sensor_data", 10,
        std::bind(&NuturtleControl::sensor_data_callback, this, _1));

    /// @brief Publisher to wheel_cmd topic
    wheel_cmd_pub = this->create_publisher<nuturtlebot_msgs::msg::WheelCommands>("wheel_cmd", 10);

    //// @brief Publisher to joint_states topic
    joint_states_pub = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    /// @brief Timer
    // _timer = this->create_wall_timer(500ms, std::bind(&NuturtleControl::timer_callback, this));

    turtlelib::DiffDrive turtlebot(wheel_radius, track_width);
  }

private:
  rclcpp::TimerBase::SharedPtr _timer;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
  rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_sub;
  rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_pub;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub;
  turtlelib::DiffDrive turtlebot;
  double wheel_radius;
  double track_width;

  void cmd_vel_callback(const geometry_msgs::msg::Twist &Vmsg)
  {
    // Convert the geometry_msg/Twist to a turtlelib/Twist2D
    turtlelib::Twist2D V;
    V.xdot = Vmsg.linear.x;
    V.ydot = Vmsg.linear.y;
    V.thetadot = Vmsg.angular.z;

    // Inverse kinematics to get the required wheel speeds
    turtlelib::WheelState speeds = turtlebot.inverse_kinematics(V);

    // Publish wheel speeds to wheel_cmd topic
    nuturtlebot_msgs::msg::WheelCommands wheel_cmd_msg;
    wheel_cmd_msg.left_velocity = speeds.left;
    wheel_cmd_msg.right_velocity = speeds.right;
    wheel_cmd_pub->publish(wheel_cmd_msg);
  }

  void sensor_data_callback(const nuturtlebot_msgs::msg::SensorData &sensor_msg)
  {
    // TODO: find angle in radians and velocity of the wheels
  }

  // void timer_callback()
  // {
  //   // do nothing
  // }
};

/// \brief the main function to run the nusim node
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NuturtleControl>());
  rclcpp::shutdown();
  return 0;
}

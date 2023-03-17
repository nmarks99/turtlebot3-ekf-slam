/// \file
/// \brief turtle_control node: controls the turtlebot with geometry_msgs/Twist messages
///
/// PARAMETERS:
///     wheel_radius (double): radius of the wheels in meters
///     track_width (double): width of the track (distance between wheels) in meters
///     motor_cmd_max (int): motors are provided commands on the interval [-motor_cmd_max, motor_cmd_max]
///     motor_cmd_per_rad_sec (double): Equivalent speed in rad/s for each motor command "tick"
///     encoder_ticks_per_rad (double): number of encoder ticks per radian
/// PUBLISHES:
///     /wheel_cmd (nuturtlebot_msgs::msg::WheelCommands): commanded value sent to the wheels
///     /joint_states (sensor_msgs::msg::JointState): wheel speeds and angles
/// SUBSCRIBES:
///     /cmd_vel (geometry_msgs::msg::Twist): commanded body twist
///		  /sensor_data (nuturtlebot_msgs::msg::SensorData): sensor data from the real robot sensors or nusim
/// SERVICES:
///     None
/// CLIENTS:
///     None

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

/// \brief nusim turtlebot simulation node
class NuturtleControl : public rclcpp::Node
{

public:
  NuturtleControl()
  : Node("nuturtle_control")
  {

    // Declare parameters to the node and get them
    declare_parameter<double>("wheel_radius", wheel_radius);
    declare_parameter<double>("track_width", track_width);
    declare_parameter<int>("motor_cmd_max", motor_cmd_max);
    declare_parameter<double>("motor_cmd_per_rad_sec", motor_cmd_per_rad_sec);
    declare_parameter<double>("encoder_ticks_per_rad", encoder_ticks_per_rad);
    declare_parameter<int>("rate", RATE);
    RATE = get_parameter("rate").get_value<int>();
    wheel_radius = get_parameter("wheel_radius").get_value<double>();
    track_width = get_parameter("track_width").get_value<double>();
    motor_cmd_max = get_parameter("motor_cmd_max").get_value<int>();
    motor_cmd_per_rad_sec = get_parameter("motor_cmd_per_rad_sec").get_value<double>();
    encoder_ticks_per_rad = get_parameter("encoder_ticks_per_rad").get_value<double>();

    // Throw runtime error if any of the parameters are missing
    if (wheel_radius == 0) {
      RCLCPP_ERROR_STREAM(get_logger(), "wheel_radius parameter not provided");
      throw std::runtime_error("wheel_radius parameter not provided");
    }
    if (track_width == 0) {
      RCLCPP_ERROR_STREAM(get_logger(), "track_width parameter not provided");
      throw std::runtime_error("track_width parameter not provided");
    }
    if (motor_cmd_max == 0) {
      RCLCPP_ERROR_STREAM(get_logger(), "motor_cmd_max parameter not provided");
      throw std::runtime_error("motor_cmd_max parameter not provided");
    }
    if (motor_cmd_per_rad_sec == 0) {
      RCLCPP_ERROR_STREAM(get_logger(), "motor_cmd_per_rad_sec parameter not provided");
      throw std::runtime_error("motor_cmd_per_rad_sec parameter not provided");
    }
    if (encoder_ticks_per_rad == 0) {
      RCLCPP_ERROR_STREAM(get_logger(), "encoder_ticks_per_rad parameter not provided");
      throw std::runtime_error("encoder_ticks_per_rad parameter not provided");
    }

    /// @brief Subscriber to cmd_vel topic
    cmd_vel_sub = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      std::bind(&NuturtleControl::cmd_vel_callback, this, _1));

    /// @brief Subscriber to sensor_data topic
    sensor_data_sub = create_subscription<nuturtlebot_msgs::msg::SensorData>(
      "sensor_data", 10,
      std::bind(&NuturtleControl::sensor_data_callback, this, _1));

    /// @brief Publisher to wheel_cmd topic
    wheel_cmd_pub = create_publisher<nuturtlebot_msgs::msg::WheelCommands>("wheel_cmd", 10);

    //// @brief Publisher to joint_states topic
    joint_states_pub = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    /// \brief Timer (frequency defined by node parameter)
    _timer = create_wall_timer(
      std::chrono::milliseconds((int)(1000 / RATE)),
      std::bind(&NuturtleControl::timer_callback, this));

    // initialize joint states message
    js_msg.name.push_back("wheel_left_joint");
    js_msg.name.push_back("wheel_right_joint");
    js_msg.position.push_back(0.0);
    js_msg.position.push_back(0.0);
    js_msg.velocity.push_back(0.0);
    js_msg.velocity.push_back(0.0);
  }

private:
  // Parameters that can be passed to the node
  // probably from diff_params.yaml in nuturtle_description
  double wheel_radius = 0.0;
  double track_width = 0.0;
  int motor_cmd_max = 0;
  double motor_cmd_per_rad_sec = 0.0;
  double encoder_ticks_per_rad = 0.0;
  int RATE = 200;

  // Pubscriptions
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
  rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_sub;

  // Publishers
  rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_pub;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub;

  // Other
  rclcpp::TimerBase::SharedPtr _timer; // Timer
  turtlelib::DiffDrive turtlebot;      // DiffDrive object for IK and FK for turtlebot
  sensor_msgs::msg::JointState js_msg; // JointStates message

  // Encoder values at the last timestep
  double last_encoder_left = 0.0;
  double last_encoder_right = 0.0;

  /// @brief callback to cmd_vel subscriber which computes the
  /// wheel speeds required to acheive the requested body twist
  /// using inverse kinematics, then published a WheelCommands
  /// message to /wheel_cmd to move the robot with the requested twist
  /// @param Vmsg - (geometry_msgs/msg/Twist)
  void cmd_vel_callback(const geometry_msgs::msg::Twist & Vmsg)
  {
    // Store the geometry_msg/Twist in a turtlelib/Twist2D
    turtlelib::Twist2D V{Vmsg.angular.z, Vmsg.linear.x, 0.0};

    // Inverse kinematics to get the required wheel speeds
    turtlelib::WheelState speeds = turtlebot.inverse_kinematics(V);

    // Convert to speeds to integer wheel command value
    nuturtlebot_msgs::msg::WheelCommands wheel_cmd_msg;
    wheel_cmd_msg.left_velocity = speeds.left / motor_cmd_per_rad_sec;
    wheel_cmd_msg.right_velocity = speeds.right / motor_cmd_per_rad_sec;

    // Ensure wheel_cmd is within the allowable limits
    if (wheel_cmd_msg.left_velocity > motor_cmd_max) {
      RCLCPP_DEBUG_STREAM(get_logger(), "at max left");
      wheel_cmd_msg.left_velocity = motor_cmd_max;
    }
    if (wheel_cmd_msg.left_velocity < -motor_cmd_max) {
      RCLCPP_DEBUG_STREAM(get_logger(), "at min left");
      wheel_cmd_msg.left_velocity = -motor_cmd_max;
    }
    if (wheel_cmd_msg.right_velocity > motor_cmd_max) {
      RCLCPP_DEBUG_STREAM(get_logger(), "at max right");
      wheel_cmd_msg.right_velocity = motor_cmd_max;
    }
    if (wheel_cmd_msg.right_velocity < -motor_cmd_max) {
      RCLCPP_DEBUG_STREAM(get_logger(), "at min right");
      wheel_cmd_msg.right_velocity = -motor_cmd_max;
    }

    // Publish the wheel_cmd_msg on /wheel_cmd topic
    wheel_cmd_pub->publish(wheel_cmd_msg);
  }

  /// @brief  callback function to /sensor_data subscription
  /// which computes joint positions and velcoties from encoder data
  /// @param sensor_data (nuturtlebot_msgs/msg/SensorData)
  void sensor_data_callback(const nuturtlebot_msgs::msg::SensorData & sensor_data)
  {
    // Update wheel angles
    js_msg.position.at(0) = sensor_data.left_encoder / encoder_ticks_per_rad;
    js_msg.position.at(1) = sensor_data.right_encoder / encoder_ticks_per_rad;

    // delta_encoder_ticks / encoder_ticks_per_rad = delta_theta
    // delta_theta / dt = rad/s
    const double dt = (1.0 / RATE);
    js_msg.velocity.at(0) = (sensor_data.left_encoder - last_encoder_left) / encoder_ticks_per_rad /
      dt;
    js_msg.velocity.at(1) = (sensor_data.right_encoder - last_encoder_right) /
      encoder_ticks_per_rad / dt;
    // RCLCPP_INFO_STREAM(get_logger(), "Wheel vel = " << js_msg.velocity.at(0) << "," << js_msg.velocity.at(1));

    // Update last_encode values
    last_encoder_left = sensor_data.left_encoder;
    last_encoder_right = sensor_data.right_encoder;
  }

  /// @brief timer callback to publish joint states
  void timer_callback()
  {
    // stamp and publish joint states
    js_msg.header.stamp = get_clock()->now();
    joint_states_pub->publish(js_msg);
  }
};

/// @brief the main function to run the nuturtle_control node
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NuturtleControl>());
  rclcpp::shutdown();
  return 0;
}

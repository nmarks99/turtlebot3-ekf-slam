/// \file
/// \brief circl node: publishes to cmd_vel to drive the robot in a circle of a desired radius
///
/// PARAMETERS:
///
/// PUBLISHES:
///
/// SUBSCRIBES:
///
/// SERVICES:
///
/// CLIENTS:

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
#include "std_srvs/srv/empty.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

/// \brief nusim turtlebot simulation node
class Circle : public rclcpp::Node
{

public:
    Circle() : Node("circle")
    {

        /// @brief Publisher to cmd_vel topic
        cmd_vel_pub = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // timer callback
        _timer = create_wall_timer(
            10ms,
            std::bind(&Circle::timer_callback, this));

        _reverse_service = this->create_service<std_srvs::srv::Empty>(
            "circle/reverse",
            std::bind(&Circle::reverse_callback, this, _1, _2));

        _stop_service = this->create_service<std_srvs::srv::Empty>(
            "circle/stop",
            std::bind(&Circle::stop_callback, this, _1, _2));
    }

private:
    bool STOPPED = true;

    geometry_msgs::msg::Twist twist_msg;

    // Services
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr _reverse_service;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr _stop_service;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;

    rclcpp::TimerBase::SharedPtr _timer;

    void reverse_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                          std::shared_ptr<std_srvs::srv::Empty::Response>)
    {
        RCLCPP_INFO_STREAM(get_logger(), "reverse service");
    }

    void stop_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                       std::shared_ptr<std_srvs::srv::Empty::Response>)
    {
        RCLCPP_INFO_STREAM(get_logger(), "stop service");
        STOPPED = true;
        twist_msg.linear.x = 0.0;
        twist_msg.linear.y = 0.0;
        twist_msg.linear.z = 0.0;
        twist_msg.angular.x = 0.0;
        twist_msg.angular.y = 0.0;
        twist_msg.angular.z = 0.0;
        cmd_vel_pub->publish(twist_msg);
    }

    void timer_callback()
    {
        if (!STOPPED)
        {
            // do stuff
        }
    }
};

/// @brief the main function to run the nuturtle_control node
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Circle>());
    rclcpp::shutdown();
    return 0;
}

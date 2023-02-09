/// \file
/// \brief circle node: publishes to cmd_vel to drive the robot in a circle of a desired radius
///
/// PARAMETERS:
///
/// PUBLISHES:
///     /cmd_vel (geoemtry_msgs/msg/Twist)
/// SUBSCRIBES:
///     None
/// SERVICES:
///     circle/control (nuturtle_control/srv/Control): defines the velocity and radius of motion
///     circle/stop (std_srvs/srv/Empty): stops the robot
///     circle/reverse (std_srvs/srv/Empty): reverse the direction of the robot
/// CLIENTS:
///     None

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "turtlelib/diff_drive.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "std_srvs/srv/empty.hpp"
#include "nuturtle_control/srv/control.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

/// \brief circle node that defines services to drive the robot in a circle
class Circle : public rclcpp::Node
{

public:
    Circle() : Node("circle")
    {
        declare_parameter("frequency", RATE);
        RATE = get_parameter("frequency").get_value<int>();

        /// @brief Publisher to cmd_vel topic
        cmd_vel_pub = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        /// \brief Timer (frequency defined by node parameter)
        _timer = create_wall_timer(
            std::chrono::milliseconds((int)(1000 / RATE)),
            std::bind(&Circle::timer_callback, this));

        /// @brief reverse service that reverses the direction of the robot
        _reverse_service = this->create_service<std_srvs::srv::Empty>(
            "circle/reverse",
            std::bind(&Circle::reverse_callback, this, _1, _2));

        /// @brief stop service that stops the robot
        _stop_service = this->create_service<std_srvs::srv::Empty>(
            "circle/stop",
            std::bind(&Circle::stop_callback, this, _1, _2));

        /// @brief control service that sets the angular velocity and
        /// radius of the circle for the robot to follow
        _control_service = create_service<nuturtle_control::srv::Control>(
            "circle/control",
            std::bind(&Circle::control_callback, this, _1, _2));
    }

private:
    bool STOPPED = true;

    geometry_msgs::msg::Twist twist_msg;
    rclcpp::TimerBase::SharedPtr _timer;
    int RATE = 100;

    // Services
    rclcpp::Service<nuturtle_control::srv::Control>::SharedPtr _control_service;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr _reverse_service;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr _stop_service;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;

    /// @brief control callback fills in a twist message with the values from the request
    /// @param request - nuturtle_control/srv/Control service request
    void control_callback(const std::shared_ptr<nuturtle_control::srv::Control::Request> request,
                          std::shared_ptr<nuturtle_control::srv::Control::Response>)
    {
        RCLCPP_INFO_STREAM(get_logger(), "velocity = " << request->velocity);
        RCLCPP_INFO_STREAM(get_logger(), "radius = " << request->radius);
        RCLCPP_INFO_STREAM(get_logger(), "angular_vel = " << request->velocity / request->radius);

        // Set flag to enable publishing of cmd_vel messsages
        STOPPED = false;

        // set linear x velocity to v
        twist_msg.linear.x = request->velocity;

        // Set angular z to v/r so it moves in a circle
        twist_msg.angular.z = request->velocity / request->radius;

        // all else zero
        twist_msg.linear.y = 0.0;
        twist_msg.linear.z = 0.0;
        twist_msg.angular.x = 0.0;
        twist_msg.angular.y = 0.0;
    }

    /// @brief reverses the direction of the robot's motion
    void reverse_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                          std::shared_ptr<std_srvs::srv::Empty::Response>)
    {
        if (!STOPPED)
        {
            twist_msg.linear.x = -twist_msg.linear.x;
            twist_msg.angular.z = -twist_msg.angular.z;
        }
        RCLCPP_INFO_STREAM(get_logger(), "Reversing");
    }

    /// @brief stops the robot
    void stop_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                       std::shared_ptr<std_srvs::srv::Empty::Response>)
    {
        RCLCPP_INFO_STREAM(get_logger(), "Stopping robot");
        STOPPED = true;
        twist_msg.linear.x = 0.0;
        twist_msg.linear.y = 0.0;
        twist_msg.linear.z = 0.0;
        twist_msg.angular.x = 0.0;
        twist_msg.angular.y = 0.0;
        twist_msg.angular.z = 0.0;
        cmd_vel_pub->publish(twist_msg);
    }

    /// @brief timer callback to publish the Twist messages on /cmd_vel
    void timer_callback()
    {
        if (!STOPPED)
        {
            cmd_vel_pub->publish(twist_msg);
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

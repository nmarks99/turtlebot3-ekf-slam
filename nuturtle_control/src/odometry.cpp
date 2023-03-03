/// @file
/// @brief odometry node
///
/// PARAMETERS:
///     body_id: The name of the body frame of the robot (e.g. "base_footprint")
///     odom_id: The name of the odometry frame. Defaults to odom if not specified
///     wheel_left: The name of the left wheel joint
///     wheel_right: The name of the right wheel joint
/// PUBLISHES:
///     /odom (nav_msgs::msg::Odometry): odom information
/// SUBSCRIBES:
///		/joint_states (sensor_msgs::msg::JointStat): joint (wheel) states information
/// SERVICES:
///		/initial_pose
/// CLIENTS:
///

#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "turtlelib/diff_drive.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "nuturtle_control/srv/initial_pose.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

/// @brief odometry node class
class Odometry : public rclcpp::Node
{
public:
  Odometry()
      : Node("odometry")
  {
    // pose_now.x = 0;
    // pose_now.y = 0;
    // pose_now.theta = 0;
    // wheel_angles_now.left = 0;
    // wheel_angles_now.right = 0;
    // wheel_speeds_now.left = 0;
    // wheel_speeds_now.right = 0;
    // Vb_now.thetadot = 0;
    // Vb_now.xdot = 0;
    // Vb_now.ydot = 0;

    // declare parameters to the node
    declare_parameter("body_id", body_id);
    declare_parameter("odom_id", odom_id);
    declare_parameter("wheel_left", wheel_left);
    declare_parameter("wheel_right", wheel_right);
    body_id = get_parameter("body_id").get_value<std::string>();
    odom_id = get_parameter("odom_id").get_value<std::string>();
    wheel_left = get_parameter("wheel_left").get_value<std::string>();
    wheel_right = get_parameter("wheel_right").get_value<std::string>();

    // throw runtime error if parameters are undefined
    if (body_id.empty())
    {
      RCLCPP_ERROR_STREAM(get_logger(), "body_id parameter not specified");
      throw std::runtime_error("body_id parameter not specified");
    }
    if (wheel_right.empty())
    {
      RCLCPP_ERROR_STREAM(get_logger(), "wheel_right parameter not specified");
      throw std::runtime_error("wheel_right parameter not specified");
    }
    if (wheel_left.empty())
    {
      RCLCPP_ERROR_STREAM(get_logger(), "left_right parameter not specified");
      throw std::runtime_error("left_right parameter not specified");
    }

    /// @brief Publisher to the odom topic
    odom_pub = create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    /// @brief publishes the path of the odometry (nav_msgs/Path)
    path_pub = create_publisher<nav_msgs::msg::Path>(
        "/odom/path", 10);

    /// @brief Subscriber to joint_states topic
    joint_states_sub = create_subscription<sensor_msgs::msg::JointState>(
        "/blue/joint_states", 10,
        std::bind(&Odometry::joint_states_callback, this, _1));

    /// @brief initial pose service that sets the initial pose of the robot
    _init_pose_service = this->create_service<nuturtle_control::srv::InitialPose>(
        "odometry/initial_pose",
        std::bind(&Odometry::init_pose_callback, this, _1, _2));

    /// @brief transform broadcaster used to publish transforms on the /tf topic
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    /// \brief Timer (frequency defined by node parameter)
    _timer = create_wall_timer(
        std::chrono::milliseconds((int)(1000 / RATE)),
        std::bind(&Odometry::timer_callback, this));
  }

private:
  int RATE = 200;

  // Parameters that can be passed to the node
  std::string body_id;
  std::string wheel_left;
  std::string wheel_right;
  std::string odom_id = "odom";
  int count = 0;

  // DiffDrive object
  turtlelib::DiffDrive ddrive;

  // Current robot state
  turtlelib::Pose2D pose_now{0.0, 0.0, 0.0};
  turtlelib::WheelState wheel_angles_now{0.0, 0.0};
  turtlelib::WheelState wheel_speeds_now{0.0, 0.0};
  turtlelib::Twist2D Vb_now{0.0, 0.0, 0.0};
  tf2::Quaternion q;

  // Declare messages
  nav_msgs::msg::Odometry odom_msg;
  geometry_msgs::msg::TransformStamped odom_body_tf;
  nav_msgs::msg::Path path_msg;

  // Declare timer
  rclcpp::TimerBase::SharedPtr _timer;

  // Declare transform broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

  // Declare subscriptions
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub;

  // Declare publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;

  // Services
  rclcpp::Service<nuturtle_control::srv::InitialPose>::SharedPtr _init_pose_service;

  /// @brief Callback to odometry/initial_pose service which sets the starting
  /// pose of the robot to begin odometry calculations at
  /// @param request
  /// @param
  void init_pose_callback(
      const std::shared_ptr<nuturtle_control::srv::InitialPose::Request> request,
      std::shared_ptr<nuturtle_control::srv::InitialPose::Response>)
  {
    pose_now.x = request->x;
    pose_now.y = request->y;
    pose_now.theta = request->theta;
  }

  void joint_states_callback(sensor_msgs::msg::JointState js_data)
  {
    // Update velocities and positions of the wheels
    wheel_angles_now.left = js_data.position.at(0);
    wheel_angles_now.right = js_data.position.at(1);
    wheel_speeds_now.left = js_data.velocity.at(0);
    wheel_speeds_now.right = js_data.velocity.at(1);

    // Compute current body twist from given wheel velocities
    Vb_now = ddrive.body_twist(wheel_speeds_now);

    // Update current pose of the robot with forward kinematics
    pose_now = ddrive.forward_kinematics(pose_now, wheel_angles_now);
  }

  void timer_callback()
  {
    // Define quaternion for current rotation
    q.setRPY(0.0, 0.0, pose_now.theta);

    // Fill in Odometry message
    odom_msg.header.stamp = get_clock()->now();
    odom_msg.header.frame_id = odom_id;
    odom_msg.child_frame_id = body_id;
    odom_msg.pose.pose.position.x = pose_now.x;
    odom_msg.pose.pose.position.y = pose_now.y;
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();
    odom_msg.twist.twist.linear.x = Vb_now.xdot;
    odom_msg.twist.twist.linear.y = Vb_now.ydot;
    odom_msg.twist.twist.angular.z = Vb_now.thetadot;

    // Fill in TransformStamped message between odom_id frame and body_id frame
    odom_body_tf.header.stamp = get_clock()->now();
    odom_body_tf.header.frame_id = odom_id;
    odom_body_tf.child_frame_id = body_id;
    odom_body_tf.transform.translation.x = pose_now.x;
    odom_body_tf.transform.translation.y = pose_now.y;
    odom_body_tf.transform.rotation.x = q.x();
    odom_body_tf.transform.rotation.y = q.y();
    odom_body_tf.transform.rotation.z = q.z();
    odom_body_tf.transform.rotation.w = q.w();

    // publish odometry on /odom and transform on /tf
    tf_broadcaster->sendTransform(odom_body_tf);
    odom_pub->publish(odom_msg);

    constexpr int PATH_PUB_RATE = 100;
    if (count >= PATH_PUB_RATE)
    {
      count = 0;
      geometry_msgs::msg::PoseStamped temp_pose;
      temp_pose.header.stamp = get_clock()->now();
      temp_pose.pose.position.x = pose_now.x;
      temp_pose.pose.position.y = pose_now.y;
      temp_pose.pose.position.z = 0.0;
      temp_pose.pose.orientation.x = q.x();
      temp_pose.pose.orientation.y = q.y();
      temp_pose.pose.orientation.z = q.z();
      temp_pose.pose.orientation.w = q.w();

      path_msg.header.frame_id = "odom";
      path_msg.header.stamp = get_clock()->now();
      path_msg.poses.push_back(temp_pose);
      path_pub->publish(path_msg);
    }
    else
    {
      count++;
    }
  }
};

/// @brief the main function to run the odometry node
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odometry>());
  rclcpp::shutdown();
  return 0;
}

/// \file
/// \brief nusim node: a turtlebot simulation program
///
/// PARAMETERS:
///     x0 (double): starting x location of the turtlebot in the simulator
///     y0 (double): starting y location of the turtlebot in the simulator
///     theta0 (double): starting yaw angle of the turtlebot in the simulator
///     obstacles/x (std::vector<double>): Array of x locations of obstacles
///     obstacles/y (std::vector<double>): Array of y locations of obstacles
///     obstacles/r (double): Radius of the obtacles
/// PUBLISHES:
///     ~/timestep (std_msgs/msg/UInt64): simulation timestep
///     ~/obstacles (visualization_msgs/msg/MarkerArray): array of Marker messages
/// SUBSCRIBES:
///     None
/// SERVERS:
///     ~/reset (std_srvs/srv/Empty): resets the simulation timestep and the robot to its initial pose
///     ~/teleport (nusim/srv/Teleport): teleports the robot to a specified pose
/// CLIENTS:
///     None

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <rclcpp/logging.hpp>

#include "std_msgs/msg/u_int64.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/empty.hpp"
#include "nusim/srv/teleport.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

/// \cond
#define UNUSED(x) (void)(x) // used to suppress "unused-variable" warnings

const double DEFAULT_X0 = 0.0;
const double DEFAULT_Y0 = 0.0;
const double DEFAULT_THETA0 = 0.0;
const int DEFAULT_TIMER_FREQ = 200;
const std::vector<double> DEFAULT_OBSTACLES_X;
const std::vector<double> DEFAULT_OBSTACLES_Y;
const double DEFAULT_OBSTACLES_R = 0.038;
const double OBSTACLE_HEIGHT = 0.25;
/// \endcond

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

/// \brief nusim turtlebot simulation node
class Nusim : public rclcpp::Node
{

public:
  Nusim()
  : Node("nusim"), step(0)
  {

    // Declare parameters
    this->declare_parameter<int>("rate", DEFAULT_TIMER_FREQ);
    this->declare_parameter<double>("x0", DEFAULT_X0);
    this->declare_parameter<double>("y0", DEFAULT_Y0);
    this->declare_parameter<double>("theta0", DEFAULT_THETA0);
    this->declare_parameter<std::vector<double>>("obstacles/x", DEFAULT_OBSTACLES_X);
    this->declare_parameter<std::vector<double>>("obstacles/y", DEFAULT_OBSTACLES_Y);
    this->declare_parameter<double>("obstacles/r", DEFAULT_OBSTACLES_R);

    /// \brief timestep publisher (std_msgs/msg/UInt64)
    timestep_pub = this->create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);

    /// \brief timestep publisher (visualization_msgs/msg/MarkerArray)
    marker_arr_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/obstacles",
      10);

    /// \brief ~/reset service (std_srvs/srv/Empty)
    /// resets the timestep variable to 0 and resets the turtlebot
    /// pose to its initial location
    _reset_service = this->create_service<std_srvs::srv::Empty>(
      "~/reset",
      std::bind(&Nusim::reset_callback, this, _1, _2));

    /// \brief ~/teleport service (nusim/srv/Teleport)
    /// teleports the robot in the simulation to the specified pose
    _teleport_service = this->create_service<nusim::srv::Teleport>(
      "~/teleport",
      std::bind(&Nusim::teleport_callback, this, _1, _2));

    /// \brief transform broadcaster:
    /// used to publish transform on the /tf topic
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    rate = this->get_parameter("rate").get_value<int>();
    int period_ms = (int)(1000 / rate);

    /// \brief Timer (frequency defined by node parameter)
    _timer = this->create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&Nusim::timer_callback, this));

    // Ground truth pose of the robot known only to the simulator
    // Initial values are passed as parameters to the node
    x0 = this->get_parameter("x0").get_value<double>();
    y0 = this->get_parameter("y0").get_value<double>();
    theta0 = this->get_parameter("theta0").get_value<double>();
    true_pose.x = x0;
    true_pose.y = y0;
    true_pose.theta = theta0;

    // Get the requested obstacle locations and size
    obstacles_r = this->get_parameter("obstacles/r").get_value<double>();
    obstacles_x = this->get_parameter("obstacles/x").get_value<std::vector<double>>();
    obstacles_y = this->get_parameter("obstacles/y").get_value<std::vector<double>>();

    // Debugging info on parameters
    RCLCPP_DEBUG(this->get_logger(), "x0 = %lf", true_pose.x);
    RCLCPP_DEBUG(this->get_logger(), "y0 = %lf", true_pose.y);
    RCLCPP_DEBUG(this->get_logger(), "theta0 = %lf", true_pose.theta);
    RCLCPP_DEBUG(this->get_logger(), "obstacles/x length = %ld", obstacles_x.size());
    RCLCPP_DEBUG(this->get_logger(), "obstacles/y length = %ld", obstacles_y.size());
    RCLCPP_DEBUG(this->get_logger(), "obstacles/r = %lf", obstacles_r);

    // crashes if unequal number of x and y since they are ordered pairs
    assert(obstacles_x.size() == obstacles_y.size());

    // creates a marker at each specified location
    for (size_t i = 0; i < obstacles_x.size(); i++) {
      obstacle_marker.header.frame_id = "nusim/world";
      obstacle_marker.id = i;
      obstacle_marker.type = visualization_msgs::msg::Marker::CYLINDER;
      obstacle_marker.action = visualization_msgs::msg::Marker::ADD;
      obstacle_marker.scale.x = obstacles_r;
      obstacle_marker.scale.y = obstacles_r;
      obstacle_marker.scale.z = OBSTACLE_HEIGHT;
      obstacle_marker.pose.position.x = obstacles_x.at(i);
      obstacle_marker.pose.position.y = obstacles_y.at(i);
      obstacle_marker.pose.position.z = OBSTACLE_HEIGHT / 2;
      obstacle_marker.color.r = 0.0;
      obstacle_marker.color.g = 1.0;
      obstacle_marker.color.b = 0.0;
      obstacle_marker.color.a = 1.0;
      obstacle_marker_arr.markers.push_back(obstacle_marker);       // pack Marker into MarkerArray
    }
  }

private:
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_arr_pub;
  rclcpp::TimerBase::SharedPtr _timer;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr _reset_service;
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr _teleport_service;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

  visualization_msgs::msg::MarkerArray obstacle_marker_arr;
  visualization_msgs::msg::Marker obstacle_marker;

  std::vector<double> obstacles_x;
  std::vector<double> obstacles_y;
  double obstacles_r;
  double x0, y0, theta0;
  int rate;

  uint64_t step;

  /// \brief a 2D pose
  struct Pose2D
  {
    double x;
    double y;
    double theta;
  } true_pose;

  /// \brief ~/reset service callback function:
  /// resets the timestep variable to 0 and resets the
  /// turtlebot pose to its initial location
  /// \param request - std_srvs/srv/Empty request (unused)
  /// \param response - std_srvs/srv/Emptry response (unused)
  void reset_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response)
  {
    UNUSED(request);
    UNUSED(response);

    step = 0;
    true_pose.x = x0;
    true_pose.y = x0;
    true_pose.theta = theta0;
  }

  /// \brief ~/teleport service callback function:
  /// teleports the robot to the desired pose by setting
  /// true_pose equal to the input x,y,theta
  /// \param request - nusim/srv/Teleport request which has x,y,theta fields (UInt64)
  /// \param response - nusim/srv/Teleport response which is empty (unused)
  void teleport_callback(
    const std::shared_ptr<nusim::srv::Teleport::Request> request,
    std::shared_ptr<nusim::srv::Teleport::Response> response)
  {
    UNUSED(response);

    true_pose.x = request->x;
    true_pose.y = request->y;
    true_pose.theta = request->theta;
  }

  /// \brief timer callback function:
  /// publises the simulation timestep, updates the transform between
  /// the nusim/world and red/base_footprint frames, and published obstacle MarkerArray
  void timer_callback()
  {

    // publish timestep
    auto timestep_message = std_msgs::msg::UInt64();
    timestep_message.data = step++;
    timestep_pub->publish(timestep_message);

    // publish transform between nusim/world and red/base_footprint frames
    geometry_msgs::msg::TransformStamped world_red_tf;
    world_red_tf.header.stamp = this->get_clock()->now();
    world_red_tf.header.frame_id = "nusim/world";
    world_red_tf.child_frame_id = "red/base_footprint";
    world_red_tf.transform.translation.x = true_pose.x;
    world_red_tf.transform.translation.y = true_pose.y;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, true_pose.theta);
    world_red_tf.transform.rotation.x = q.x();
    world_red_tf.transform.rotation.y = q.y();
    world_red_tf.transform.rotation.z = q.z();
    world_red_tf.transform.rotation.w = q.w();
    tf_broadcaster->sendTransform(world_red_tf);

    // publish MarkerArray of obstacles
    marker_arr_pub->publish(obstacle_marker_arr);
  }
};

/// \brief the main function to run the nusim node
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nusim>());
  rclcpp::shutdown();
  return 0;
}

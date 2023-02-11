/// \file
/// \brief nusim node: a turtlebot3 simulation program
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
///		/red/sensor_data (nuturtlebot_msgs/msg/SensorData): wheel encoder values
/// SUBSCRIBES:
///     /red/wheel_cmd (nuturtlebot_msgs/msg/WheelCommands): integer valued wheel command speeds
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
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "turtlelib/diff_drive.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

static constexpr double OBSTACLE_HEIGHT = 0.25;
static constexpr double WALL_HEIGHT = 0.25;
static constexpr double WALL_WIDTH = 0.15;

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
    declare_parameter<int>("rate", RATE);
    declare_parameter<double>("x0", X0);
    declare_parameter<double>("y0", Y0);
    declare_parameter<double>("theta0", THETA0);
    declare_parameter<std::vector<double>>("obstacles/x", obstacles_x);
    declare_parameter<std::vector<double>>("obstacles/y", obstacles_y);
    declare_parameter<double>("obstacles/r", obstacles_r);
    declare_parameter<double>("motor_cmd_per_rad_sec", MOTOR_CMD_PER_RAD_SEC);
    declare_parameter<int>("motor_cmd_max", MOTOR_CMD_MAX);
    declare_parameter<double>("encoder_ticks_per_rad", ENCODER_TICKS_PER_RAD);
    declare_parameter<double>("wall_x_length", X_LENGTH);
    declare_parameter<double>("wall_y_length", Y_LENGTH);

    // Get parameters
    obstacles_r = get_parameter("obstacles/r").get_value<double>();
    obstacles_x = get_parameter("obstacles/x").get_value<std::vector<double>>();
    obstacles_y = get_parameter("obstacles/y").get_value<std::vector<double>>();
    assert(obstacles_x.size() == obstacles_y.size());
    MOTOR_CMD_PER_RAD_SEC = get_parameter("motor_cmd_per_rad_sec").get_value<double>();
    MOTOR_CMD_MAX = get_parameter("motor_cmd_max").get_value<int>();
    ENCODER_TICKS_PER_RAD = get_parameter("encoder_ticks_per_rad").get_value<double>();
    X0 = get_parameter("x0").get_value<double>();
    Y0 = get_parameter("y0").get_value<double>();
    THETA0 = get_parameter("theta0").get_value<double>();
    RATE = get_parameter("rate").get_value<int>();

    // Check for required parameters
    if (turtlelib::almost_equal(MOTOR_CMD_PER_RAD_SEC, 0.0))
    {
      RCLCPP_ERROR_STREAM(get_logger(), "motor_cmd_per_rad_sec parameter missing");
      throw std::runtime_error("motor_cmd_per_rad_sec parameter missing");
    }

    if (MOTOR_CMD_MAX == 0)
    {
      RCLCPP_ERROR_STREAM(get_logger(), "motor_cmd_max parameter missing");
      throw std::runtime_error("motor_cmd_max parameter missing");
    }

    if (turtlelib::almost_equal(ENCODER_TICKS_PER_RAD, 0.0))
    {
      RCLCPP_ERROR_STREAM(get_logger(), "encoder_ticks_per_rad parameter missing");
      throw std::runtime_error("encoder_ticks_per_rad parameter missing");
    }

    /// @brief timestep publisher (std_msgs/msg/UInt64)
    timestep_pub = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);

    /// @brief marker publisher (visualization_msgs/msg/MarkerArray)
    marker_arr_pub = create_publisher<visualization_msgs::msg::MarkerArray>(
        "~/obstacles", 10);

    /// @brief red/sensor data publisher which publishes the encoder ticks of the wheels
    sensor_data_pub = create_publisher<nuturtlebot_msgs::msg::SensorData>(
        "red/sensor_data", 10);

    /// @brief subscription ot wheel_cmd to get the commanded
    /// integer values which detemines the wheel velocities
    wheel_cmd_sub = create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
        "red/wheel_cmd",
        10,
        std::bind(&Nusim::wheel_cmd_callback, this, _1));

    /// \brief ~/reset service (std_srvs/srv/Empty)
    /// resets the timestep variable to 0 and resets the turtlebot
    /// pose to its initial location
    _reset_service = create_service<std_srvs::srv::Empty>(
        "~/reset",
        std::bind(&Nusim::reset_callback, this, _1, _2));

    /// \brief ~/teleport service (nusim/srv/Teleport)
    /// teleports the robot in the simulation to the specified pose
    _teleport_service = create_service<nusim::srv::Teleport>(
        "~/teleport",
        std::bind(&Nusim::teleport_callback, this, _1, _2));

    /// \brief transform broadcaster:
    /// used to publish transform on the /tf topic
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    /// \brief Timer (frequency defined by node parameter)
    _timer = create_wall_timer(
        std::chrono::milliseconds((int)(1000 / RATE)),
        std::bind(&Nusim::timer_callback, this));

    // Ground truth pose of the robot known only to the simulator
    // Initial values are passed as parameters to the node
    true_pose.x = X0;
    true_pose.y = Y0;
    true_pose.theta = THETA0;

    // Creates a marker obstacle at each specified location
    size_t i = 0;
    for (i = 0; i < obstacles_x.size(); i++)
    {
      marker_msg.header.frame_id = "nusim/world";
      marker_msg.id = i;
      marker_msg.type = visualization_msgs::msg::Marker::CYLINDER;
      marker_msg.action = visualization_msgs::msg::Marker::ADD;
      marker_msg.scale.x = obstacles_r;
      marker_msg.scale.y = obstacles_r;
      marker_msg.scale.z = OBSTACLE_HEIGHT;
      marker_msg.pose.position.x = obstacles_x.at(i);
      marker_msg.pose.position.y = obstacles_y.at(i);
      marker_msg.pose.position.z = OBSTACLE_HEIGHT / 2;
      marker_msg.color.r = 0.0;
      marker_msg.color.g = 1.0;
      marker_msg.color.b = 0.0;
      marker_msg.color.a = 1.0;
      obstacle_marker_arr.markers.push_back(marker_msg); // pack Marker into MarkerArray
    }

    // Creates the four walls
    size_t last_id = i;
    for (size_t i = 0; i < 4; i++)
    {
      marker_msg.header.frame_id = "nusim/world";
      marker_msg.id = last_id + i;
      marker_msg.type = visualization_msgs::msg::Marker::CUBE;
      marker_msg.action = visualization_msgs::msg::Marker::ADD;
      marker_msg.color.r = 1.0;
      marker_msg.color.g = 1.0;
      marker_msg.color.b = 1.0;
      marker_msg.color.a = 1.0;
      marker_msg.scale.x = WALL_WIDTH;
      marker_msg.scale.z = WALL_HEIGHT;
      marker_msg.pose.position.z = WALL_HEIGHT / 2.0;

      if (i < 2)
      {
        marker_msg.scale.y = Y_LENGTH;
        marker_msg.pose.position.y = 0.0;
        if (i == 0)
        {
          marker_msg.pose.position.x = X_LENGTH / 2.0 + WALL_WIDTH / 2.0;
        }
        else
        {
          marker_msg.pose.position.x = -X_LENGTH / 2.0 - WALL_WIDTH / 2.0;
        }
      }
      else
      {
        q.setRPY(0.0, 0.0, 1.57);
        marker_msg.pose.orientation.x = q.x();
        marker_msg.pose.orientation.y = q.y();
        marker_msg.pose.orientation.z = q.z();
        marker_msg.pose.orientation.w = q.w();
        marker_msg.pose.position.x = 0.0;
        marker_msg.scale.y = X_LENGTH;
        if (i == 2)
        {
          marker_msg.pose.position.y = Y_LENGTH / 2.0 + WALL_WIDTH / 2.0;
        }
        else
        {
          marker_msg.pose.position.y = -Y_LENGTH / 2.0 - WALL_WIDTH / 2.0;
        }
      }

      // Pack Markers into MarkerArray
      obstacle_marker_arr.markers.push_back(marker_msg);
    }

    // Define parent and child frame id's
    world_red_tf.header.frame_id = "nusim/world";
    world_red_tf.child_frame_id = "red/base_footprint";
  }

private:
  std::vector<double> obstacles_x;
  std::vector<double> obstacles_y;
  double obstacles_r = 0.0;
  double X0 = 0.0;
  double Y0 = 0.0;
  double THETA0 = 0.0;
  int RATE = 200;
  double MOTOR_CMD_PER_RAD_SEC = 0.0;
  double ENCODER_TICKS_PER_RAD = 0.0;
  int MOTOR_CMD_MAX = 0;
  double X_LENGTH = 5.0;
  double Y_LENGTH = 5.0;
  uint64_t step;

  turtlelib::WheelState wheel_speeds{0.0, 0.0};
  turtlelib::WheelState wheel_angles{0.0, 0.0};
  turtlelib::Pose2D true_pose{0.0, 0.0, 0.0};
  turtlelib::DiffDrive ddrive;
  tf2::Quaternion q;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_arr_pub;
  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_pub;

  // Subscribers
  rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_sub;

  // Timer
  rclcpp::TimerBase::SharedPtr _timer;

  // Services
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr _reset_service;
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr _teleport_service;

  // tf broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

  // Declare messages
  geometry_msgs::msg::TransformStamped world_red_tf;
  nuturtlebot_msgs::msg::SensorData sensor_data;
  visualization_msgs::msg::MarkerArray obstacle_marker_arr;
  visualization_msgs::msg::Marker marker_msg;

  /// @brief /wheel_cmd topic callback function that reads the integer value
  /// WheelCommands, converts them to speeds in rad/s, computes the angles
  /// at the next step, sensor encoder values, and finally the new pose of the
  /// robot using forward kinematics.
  void wheel_cmd_callback(const nuturtlebot_msgs::msg::WheelCommands &wheel_cmd)
  {
    // Compute wheel speeds (rad/s) from wheel command message
    wheel_speeds.left = wheel_cmd.left_velocity * MOTOR_CMD_PER_RAD_SEC;
    wheel_speeds.right = wheel_cmd.right_velocity * MOTOR_CMD_PER_RAD_SEC;

    // Compute new wheel angles (rad)
    wheel_angles.left = wheel_angles.left + wheel_speeds.left * 0.005;
    wheel_angles.right = wheel_angles.right + wheel_speeds.right * 0.005;

    // Convert angle to encoder ticks to fill in sensor_data message
    sensor_data.left_encoder = (int)(wheel_angles.left * ENCODER_TICKS_PER_RAD);
    sensor_data.right_encoder = (int)(wheel_angles.right * ENCODER_TICKS_PER_RAD);

    // Use new wheel angles with forward kinematics to update transform
    true_pose = ddrive.forward_kinematics(true_pose, wheel_angles);
  }

  /// \brief ~/reset service callback function:
  /// resets the timestep variable to 0 and resets the
  /// turtlebot pose to its initial location
  void reset_callback(
      const std::shared_ptr<std_srvs::srv::Empty::Request>,
      std::shared_ptr<std_srvs::srv::Empty::Response>)
  {

    true_pose.x = X0;
    true_pose.y = Y0;
    true_pose.theta = THETA0;
  }

  /// \brief ~/teleport service callback function:
  /// teleports the robot to the desired pose by setting
  /// true_pose equal to the input x,y,theta
  /// \param request - nusim/srv/Teleport request which has x,y,theta fields (UInt64)
  void teleport_callback(
      const std::shared_ptr<nusim::srv::Teleport::Request> request,
      std::shared_ptr<nusim::srv::Teleport::Response>)
  {
    // Teleports the robot by redefining the current pose
    true_pose.x = request->x;
    true_pose.y = request->y;
    true_pose.theta = request->theta;
  }

  /// \brief timer callback function:
  /// publises the simulation timestep, updates the transform between
  /// the nusim/world and red/base_footprint frames, and published obstacle MarkerArray
  void timer_callback()
  {
    // Publish timestep
    auto timestep_message = std_msgs::msg::UInt64();
    timestep_message.data = step++;
    timestep_pub->publish(timestep_message);

    // Set the translation
    world_red_tf.transform.translation.x = true_pose.x;
    world_red_tf.transform.translation.y = true_pose.y;

    // Set the rotation
    q.setRPY(0.0, 0.0, true_pose.theta);
    world_red_tf.transform.rotation.x = q.x();
    world_red_tf.transform.rotation.y = q.y();
    world_red_tf.transform.rotation.z = q.z();
    world_red_tf.transform.rotation.w = q.w();

    // Stamp and broadcast the transform
    world_red_tf.header.stamp = get_clock()->now();
    tf_broadcaster->sendTransform(world_red_tf);

    // Publish MarkerArray of obstacles
    marker_arr_pub->publish(obstacle_marker_arr);

    // Publish sensor data
    sensor_data_pub->publish(sensor_data);
  }
};

/// \brief the main function to run the nusim node
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nusim>());
  rclcpp::shutdown();
  return 0;
}

/// @file
/// @brief SLAM and Odometry node
///
/// PARAMETERS:
///     Q :  Extended Kalman Filter process noise gain
///     R :  Extended Kalman Filter sensor noise gain
///     odom_id: The name of the odometry frame. Defaults to odom if not specified
///     wheel_left: The name of the left wheel joint
///     wheel_right: The name of the right wheel joint
/// PUBLISHES:
///     /odom (nav_msgs/Odometry): odom information
///     /odom/path (nav_msgs/Path): path taken by robot from odometry estimate
///     /slam/path (nav_msgs/Path): path taken by roobt from SLAM estimate
///     /slam/landmarks (visualization_msgs/MarkerArray): Estimated landmark locations from SLAM
/// SUBSCRIBES:
///		/joint_states (sensor_msgs/JointState): joint (wheel) states information
///		/detected_landmarks (nuslam/PointArray): landmark locations from circle fitting algorithm
///		/fake_sensor (visualization_msgs/MarkerArray): Markers representing fake sensed obstacles
/// SERVICES:
///		/initial_pose: Can be used to set the initial pose for the odometry estimate
/// CLIENTS:
///

#include <chrono>
#include <functional>
#include <memory>
#include <fstream>
#include <iostream>

#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"

#include "nuslam/srv/initial_pose.hpp"

#include "turtlelib/diff_drive.hpp"
#include "turtlelib/kalman.hpp"

#include "armadillo"
#include "nuslam/msg/point_array.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

/// @cond
// if true, saves slam data (pose predictions etc.) to a csv file
static constexpr bool SAVE_TO_CSV = true;

// for logging slam data to a csv file
std::ofstream slam_log_file;
std::ofstream odom_log_file;
auto t0 = std::chrono::system_clock::now();

// throttle the rate at which path messages are published
constexpr unsigned int PATH_PUB_RATE = 100;
/// @endcond

/// @brief SLAM + Odometry node
class Slam : public rclcpp::Node
{
public:
  Slam()
  : Node("slam")
  {

    // Declare parameters to the node
    declare_parameter("body_id", body_id);
    declare_parameter("odom_id", odom_id);
    declare_parameter("wheel_left", wheel_left);
    declare_parameter("wheel_right", wheel_right);
    declare_parameter("Q", Q);
    declare_parameter("R", R);
    declare_parameter("known_association", KNOWN_ASSOCIATION);

    // Get parameters
    Q = get_parameter("Q").get_value<double>();
    R = get_parameter("R").get_value<double>();
    KNOWN_ASSOCIATION = get_parameter("known_association").get_value<bool>();
    body_id = get_parameter("body_id").get_value<std::string>();
    odom_id = get_parameter("odom_id").get_value<std::string>();
    wheel_left = get_parameter("wheel_left").get_value<std::string>();
    wheel_right = get_parameter("wheel_right").get_value<std::string>();

    // throw runtime error if required parameters are undefined
    if (body_id.empty()) {
      RCLCPP_ERROR_STREAM(get_logger(), "body_id parameter not specified");
      throw std::runtime_error("body_id parameter not specified");
    }
    if (wheel_right.empty()) {
      RCLCPP_ERROR_STREAM(get_logger(), "wheel_right parameter not specified");
      throw std::runtime_error("wheel_right parameter not specified");
    }
    if (wheel_left.empty()) {
      RCLCPP_ERROR_STREAM(get_logger(), "left_right parameter not specified");
      throw std::runtime_error("left_right parameter not specified");
    }

    /// @brief Publisher to the odom topic
    odom_pub = create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    /// @brief Publishes the path of the slam estimate
    slam_path_pub = create_publisher<nav_msgs::msg::Path>("/slam/path", 10);

    /// @brief Publishes the path of the slam estimate
    odom_path_pub = create_publisher<nav_msgs::msg::Path>("/odom/path", 10);

    /// @brief Publishses the SLAM landmarks
    slam_marker_arr_pub = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/slam/landmarks", 10);

    /// @brief Subscriber to joint_states topic
    joint_states_sub = create_subscription<sensor_msgs::msg::JointState>(
      "/blue/joint_states", 10,
      std::bind(&Slam::joint_states_callback, this, _1));
    
    /// @brief subscription to the detected landmarks from circle fitting
    /// which is based on data from the lidar scanner (or fake lidar scanner)
    if (not KNOWN_ASSOCIATION)
    {
      detected_landmarks_sub = create_subscription<nuslam::msg::PointArray>(
          "/detected_landmarks", 10, std::bind(&Slam::detected_landmarks_callback, this, _1)
          );
    }

    /// @brief subscription to fake sensor for use with SLAM 
    /// with known data association
    if (KNOWN_ASSOCIATION)
    {
      RCLCPP_INFO_STREAM(get_logger(), "Known data association, using fake sensor");
      fake_sensor_sub = create_subscription<visualization_msgs::msg::MarkerArray>(
        "/fake_sensor", 10, std::bind(&Slam::fake_sensor_callback, this, _1));
    }

    /// @brief initial pose service that sets the initial pose of the robot
    _init_pose_service = this->create_service<nuslam::srv::InitialPose>(
      "odometry/initial_pose",
      std::bind(&Slam::init_pose_callback, this, _1, _2));

    /// @brief transform broadcaster used to publish transforms on the /tf topic
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    /// @brief static transform broadcaster used to publish a tf between world and map frames
    static_tf_broadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

    /// \brief Timer (frequency defined by node parameter)
    _timer = create_wall_timer(
      std::chrono::milliseconds((int)(1000 / RATE)),
      std::bind(&Slam::timer_callback, this));

    // world -> map (static)
    world_map_tf.header.stamp = get_clock()->now();
    world_map_tf.header.frame_id = "nusim/world";
    world_map_tf.child_frame_id = "map";
    static_tf_broadcaster->sendTransform(world_map_tf);

    // odom -> green robot
    // this is the same as odom -> blue/base_footprint
    odom_green_tf.header.frame_id = "odom_slam";
    odom_green_tf.child_frame_id = "green/base_footprint";

    // map -> odom_slam
    // this comes from the state estimate from the EKF
    map_odom_tf.header.frame_id = "map";
    map_odom_tf.child_frame_id = "odom_slam";

    // slam path message
    slam_path_msg.header.frame_id = "map";

    // odom path message
    odom_path_msg.header.frame_id = "odom";
  }

private:
  // Main timer callback frequency in Hz
  int RATE = 100;

  unsigned int count = 0;
  bool path_flag = true;
  bool fake_sensor_flag = false;

  // Parameters that can be passed to the node
  bool KNOWN_ASSOCIATION = true;
  double Q = 100.0;
  double R = 100.0;
  std::string body_id;
  std::string wheel_left;
  std::string wheel_right;
  std::string odom_id = "odom";

  // KalmanFilter object
  turtlelib::KalmanFilter ekf{Q, R};
  arma::mat slam_pose_estimate = arma::mat(3, 1, arma::fill::zeros);
  arma::mat slam_map_estimate = arma::mat(3, 1, arma::fill::zeros);
  arma::mat slam_state_estimate = arma::mat(5, 1, arma::fill::zeros);
  std::vector<turtlelib::LandmarkMeasurement> landmarks;

  // DiffDrive object
  turtlelib::DiffDrive ddrive;

  // Current robot state
  turtlelib::Pose2D pose_now{0.0, 0.0, 0.0};
  turtlelib::WheelState wheel_angles_now{0.0, 0.0};
  turtlelib::WheelState wheel_speeds_now{0.0, 0.0};
  turtlelib::Twist2D Vb_now{0.0, 0.0, 0.0};

  // Declare messages
  nav_msgs::msg::Odometry odom_msg;
  nav_msgs::msg::Path slam_path_msg;
  nav_msgs::msg::Path odom_path_msg;
  visualization_msgs::msg::MarkerArray slam_marker_arr;
  geometry_msgs::msg::TransformStamped odom_blue_tf;
  geometry_msgs::msg::TransformStamped world_map_tf;
  geometry_msgs::msg::TransformStamped map_odom_tf;
  geometry_msgs::msg::TransformStamped odom_green_tf;

  // Declare timer
  rclcpp::TimerBase::SharedPtr _timer;

  // Declare transform broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster;

  // Declare subscriptions
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub;
  rclcpp::Subscription<nuslam::msg::PointArray>::SharedPtr detected_landmarks_sub;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr fake_sensor_sub;

  // Declare publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr slam_path_pub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr odom_path_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr slam_marker_arr_pub;

  // Services
  rclcpp::Service<nuslam::srv::InitialPose>::SharedPtr _init_pose_service;

  /// @brief Callback to odometry/initial_pose service which sets the starting
  /// pose of the robot to begin odometry calculations at
  /// @param request
  /// @param
  void init_pose_callback(
    const std::shared_ptr<nuslam::srv::InitialPose::Request> request,
    std::shared_ptr<nuslam::srv::InitialPose::Response>)
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
  
  /// @brief callback function for detected landmark centers
  /// from circle fitting/classification published by landmarks node
  void detected_landmarks_callback(const nuslam::msg::PointArray &point_arr)
  {
    // by not providing an id for the landmark, the EKF assumes an unknown data association
    std::vector<turtlelib::LandmarkMeasurement> measurements;
    for (const auto &landmark_point : point_arr.points)
    {
      auto m = turtlelib::LandmarkMeasurement::from_cartesian(
          landmark_point.x,landmark_point.y);
      measurements.push_back(m); // vector of 1 measurement...should be reworked
    }
    
    // Run the extended Kalman filter for these measurements
    ekf.run(pose_now,Vb_now,measurements);
    
    // Store state prediction from SLAM
    slam_pose_estimate = ekf.pose_prediction();
    slam_map_estimate = ekf.map_prediction();
    slam_state_estimate = ekf.state_prediction();

    // fill and publish SLAM landmark markers
    fill_slam_marker_arr();

    measurements.clear();
  }

  /// @brief callback for fake sensors for SLAM with known data association
  void fake_sensor_callback(const visualization_msgs::msg::MarkerArray & marker_arr)
  {   
    
    fake_sensor_flag = true;

    // store markers in a vector of turtlelib::LandmarkMeasurement's
    // passing a marker_id signifies to the EKF that the data association is known
    for (size_t i = 0; i < marker_arr.markers.size(); i++) {
      const double x = marker_arr.markers.at(i).pose.position.x;
      const double y = marker_arr.markers.at(i).pose.position.y;
      const unsigned int marker_id = marker_arr.markers.at(i).id;
      landmarks.push_back(turtlelib::LandmarkMeasurement::from_cartesian(x, y, marker_id));
    }

    // Run extended Kalman filter and update get the updated state estimate
    ekf.run(pose_now, Vb_now, landmarks);

    // Store state estimate from SLAM
    slam_pose_estimate = ekf.pose_prediction();
    slam_map_estimate = ekf.map_prediction();
    slam_state_estimate = ekf.state_prediction();

    // Saves the state estimate from the EKF to a csv file
    if (SAVE_TO_CSV) {
      auto t1 = std::chrono::system_clock::now();
      std::chrono::duration<double> diff = t1 - t0;
      double timestamp = diff.count();
      slam_log_file << timestamp << ",";

      for (size_t i = 0; i <= slam_state_estimate.n_rows - 1; i++) {
        if (i != slam_state_estimate.n_rows - 1) {
          slam_log_file << slam_state_estimate(i, 0) << ",";
        } else {
          slam_log_file << slam_state_estimate(i, 0) << "\n";
        }
      }

      odom_log_file << timestamp << ",";
      odom_log_file << pose_now.theta << ",";
      odom_log_file << pose_now.x << ",";
      odom_log_file << pose_now.y << "\n";
    }
  }

  /// @brief fills in MarkerArray of landmarks based on SLAM estimation
  void fill_slam_marker_arr()
  {
    slam_marker_arr.markers.clear();

    visualization_msgs::msg::Marker slam_marker_msg;
    for (size_t i = 0; i <= slam_map_estimate.n_rows - 2; i += 2) {
      // Landmarks in the map frame
      const double x_m = slam_map_estimate(i, 0);
      const double y_m = slam_map_estimate(i + 1, 0);

      slam_marker_msg.header.frame_id = "map";
      slam_marker_msg.header.stamp = get_clock()->now();
      slam_marker_msg.id = i;
      slam_marker_msg.scale.x = 0.038;
      slam_marker_msg.scale.y = 0.038;
      slam_marker_msg.scale.z = 0.25;
      slam_marker_msg.pose.position.x = x_m;
      slam_marker_msg.pose.position.y = y_m;
      slam_marker_msg.pose.position.z = 0.125;
      slam_marker_msg.pose.orientation.x = 0.0;
      slam_marker_msg.pose.orientation.y = 0.0;
      slam_marker_msg.pose.orientation.z = 0.0;
      slam_marker_msg.pose.orientation.w = 1.0;
      slam_marker_msg.color.a = 0.7;
      slam_marker_msg.color.r = 0.0;
      slam_marker_msg.color.b = 0.0;
      slam_marker_msg.color.g = 1.0;
      slam_marker_msg.type = visualization_msgs::msg::Marker::CYLINDER;

      slam_marker_arr.markers.push_back(slam_marker_msg);
    }

    slam_marker_arr_pub->publish(slam_marker_arr);
  }

  /// @brief Publishes transforms for the odometry (blue) robot
  /// as well as odometry information on the odom topic
  /// @param path_flag flag for whether or not to actually publish the path
  /// which is useful for decreasing path publishing frequency for performance
  void odom_to_blue(const bool & path_flag)
  {
    // Define quaternion for current rotation
    tf2::Quaternion q_ob;
    q_ob.setRPY(0.0, 0.0, pose_now.theta);

    // Fill in Odometry message
    odom_msg.header.stamp = get_clock()->now();
    odom_msg.header.frame_id = odom_id;
    odom_msg.child_frame_id = body_id;
    odom_msg.pose.pose.position.x = pose_now.x;
    odom_msg.pose.pose.position.y = pose_now.y;
    odom_msg.pose.pose.orientation.x = q_ob.x();
    odom_msg.pose.pose.orientation.y = q_ob.y();
    odom_msg.pose.pose.orientation.z = q_ob.z();
    odom_msg.pose.pose.orientation.w = q_ob.w();
    odom_msg.twist.twist.linear.x = Vb_now.xdot;
    odom_msg.twist.twist.linear.y = Vb_now.ydot;
    odom_msg.twist.twist.angular.z = Vb_now.thetadot;

    // Fill in TransformStamped message between odom_id frame and body_id frame
    odom_blue_tf.header.stamp = get_clock()->now();
    odom_blue_tf.header.frame_id = odom_id;
    odom_blue_tf.child_frame_id = body_id;
    odom_blue_tf.transform.translation.x = pose_now.x;
    odom_blue_tf.transform.translation.y = pose_now.y;
    odom_blue_tf.transform.rotation.x = q_ob.x();
    odom_blue_tf.transform.rotation.y = q_ob.y();
    odom_blue_tf.transform.rotation.z = q_ob.z();
    odom_blue_tf.transform.rotation.w = q_ob.w();

    if (path_flag) {
      count = 0;
      geometry_msgs::msg::PoseStamped temp_pose;
      temp_pose.header.stamp = get_clock()->now();
      temp_pose.pose.position.x = pose_now.x;
      temp_pose.pose.position.y = pose_now.y;
      temp_pose.pose.position.z = 0.0;
      temp_pose.pose.orientation.x = q_ob.x();
      temp_pose.pose.orientation.y = q_ob.y();
      temp_pose.pose.orientation.z = q_ob.z();
      temp_pose.pose.orientation.w = q_ob.w();

      odom_path_msg.header.frame_id = "odom";
      odom_path_msg.header.stamp = get_clock()->now();
      odom_path_msg.poses.push_back(temp_pose);
      odom_path_pub->publish(odom_path_msg);
    }
  }

  /// @brief Fills in odom_slam to green/base_footprint tf message
  /// with the same transform info as the odom -> blue tf
  void slam_odom_to_green()
  {
    // odom_slam -> green/base_footprint
    odom_green_tf.header.stamp = get_clock()->now();
    odom_green_tf.transform = odom_blue_tf.transform;
  }

  /// @brief Publishes the map -> odom_slam tf and the path of the SLAM (green) robot
  /// @param path_flag flag for whether or not to actually publish the path
  /// which is useful for decreasing path publishing frequency for performance
  void map_to_slam_odom(const bool & path_flag)
  {
    // Compute transforms between frames
    const turtlelib::Vector2D vec_mb{slam_pose_estimate(1, 0), slam_pose_estimate(2, 0)};
    const double angle_mb = slam_pose_estimate(0, 0);
    const turtlelib::Transform2D T_MB(vec_mb, angle_mb);

    const turtlelib::Vector2D vec_ob{pose_now.x, pose_now.y};
    const turtlelib::Transform2D T_OB(vec_ob, pose_now.theta);

    const turtlelib::Transform2D T_MO = T_MB * T_OB.inv();

    // map -> odom_slam
    tf2::Quaternion q_mo;
    q_mo.setRPY(0.0, 0.0, T_MO.rotation());
    map_odom_tf.header.stamp = get_clock()->now();
    map_odom_tf.transform.translation.x = T_MO.translation().x;
    map_odom_tf.transform.translation.y = T_MO.translation().y;
    map_odom_tf.transform.rotation.x = q_mo.x();
    map_odom_tf.transform.rotation.y = q_mo.y();
    map_odom_tf.transform.rotation.z = q_mo.z();
    map_odom_tf.transform.rotation.w = q_mo.w();

    if (path_flag) {
      tf2::Quaternion q_mb;
      q_mb.setRPY(0.0, 0.0, T_MB.rotation());

      geometry_msgs::msg::PoseStamped path_pose;
      path_pose.header.frame_id = "green/base_footprint";
      path_pose.header.stamp = get_clock()->now();
      path_pose.pose.position.x = vec_mb.x;
      path_pose.pose.position.y = vec_mb.y;
      path_pose.pose.orientation.x = q_mb.x();
      path_pose.pose.orientation.y = q_mb.y();
      path_pose.pose.orientation.z = q_mb.z();
      path_pose.pose.orientation.w = q_mb.w();

      slam_path_msg.header.stamp = get_clock()->now();
      slam_path_msg.poses.push_back(path_pose);
      slam_path_pub->publish(slam_path_msg);
    }
  }

  void timer_callback()
  {
    // throttle the path publishing for performance
    if (count > PATH_PUB_RATE) {
      path_flag = true;
      count = 0;
    } else {
      path_flag = false;
      count++;
    }

    // fill in tf messages
    slam_odom_to_green();
    odom_to_blue(path_flag);
    map_to_slam_odom(path_flag);

    // send transforms
    tf_broadcaster->sendTransform(odom_blue_tf);
    tf_broadcaster->sendTransform(odom_green_tf);
    tf_broadcaster->sendTransform(map_odom_tf);

    // publish odometry msg
    odom_pub->publish(odom_msg);

    if (fake_sensor_flag) {
      fake_sensor_flag = false;

      // publish marker messages for map landmarks based on map_estimate
      fill_slam_marker_arr();

      landmarks.clear();
    }
  }
};

/// @brief the main function to run the odometry node
int main(int argc, char * argv[])
{
  if (SAVE_TO_CSV) {
    t0 = std::chrono::system_clock::now();
    slam_log_file.open("slam_log.csv");
    odom_log_file.open("odom_log.csv");
  }

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Slam>());
  rclcpp::shutdown();

  slam_log_file.close();   // may be problamatic in the event that the node crashes and we don't get here?
  odom_log_file.close();

  return 0;
}

#ifndef NUSIM_UTILS_INCLUDE_GUARD_HPP
#define NUSIM_UTILS_INCLUDE_GUARD_HPP

#include <random>
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/logging.hpp>
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/rigid2d.hpp"

static constexpr double OBSTACLE_HEIGHT = 0.25;
static constexpr double WALL_HEIGHT = 0.25;
static constexpr double WALL_WIDTH = 0.15;

/// @brief fills in the MarkerArray with cylindrical obstacles at the requested locations
/// @param marker_arr - a MarkerArray which can be empty or already containing other markers
/// @param obstacles_x - std::vector<double> of x positions of the obstacles
/// @param obstacles_y - std::vector<double> of y positions of the obstacles
/// @param obstacles_y - radius of the obstacles
void fill_obstacles(
  visualization_msgs::msg::MarkerArray & marker_arr,
  const std::vector<double> & obstacles_x, const std::vector<double> & obstacles_y,
  double obstacles_r);

/// @brief fills in the MarkerArray msg with walls to surround the "arena"
/// @param X_LENGTH - x length of the walls
/// @param Y_LENGTH - y length of the walls
void fill_walls(
  visualization_msgs::msg::MarkerArray & marker_arr, double X_LENGTH,
  double Y_LENGTH);

void fill_basic_sensor_obstacles(
  visualization_msgs::msg::MarkerArray & marker_arr,
  const std::vector<double> & obstacles_x, const std::vector<double> & obstacles_y,
  double obstacles_r, const turtlelib::Pose2D & true_pose,
  double max_range, double basic_sensor_variance);

/// @brief gets a random number, ensuring you are only seeding the
/// random number generator once
/// Credit: Matt Elwin https://nu-msr.github.io/navigation_site/lectures/gaussian.html
std::mt19937 & get_random();

#endif

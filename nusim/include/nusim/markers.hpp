#ifndef NUSIM_MARKERS_INCLUDE_GUARD_HPP
#define NUSIM_MARKERS_INCLUDE_GUARD_HPP

#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/logging.hpp>

static constexpr double OBSTACLE_HEIGHT = 0.25;
static constexpr double WALL_HEIGHT = 0.25;
static constexpr double WALL_WIDTH = 0.15;

/// @brief fills in the MarkerArray with cylindrical obstacles at the requested locations
/// @param marker_arr - a MarkerArray which can be empty or already containing other markers
/// @param obstacles_x - std::vector<double> of x positions of the obstacles
/// @param obstacles_y - std::vector<double> of y positions of the obstacles
/// @param obstacles_y - radius of the obstacles
void make_obstacles(visualization_msgs::msg::MarkerArray &marker_arr,
                    std::vector<double> obstacles_x, std::vector<double> obstacles_y, double obstacles_r);

/// @brief fills in the MarkerArray msg with walls to surround the "arena"
/// @param X_LENGTH - x length of the walls
/// @param Y_LENGTH - y length of the walls
void make_walls(visualization_msgs::msg::MarkerArray &marker_arr, double X_LENGTH, double Y_LENGTH);

#endif
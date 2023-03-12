/// @file
/// @brief Landmarks detection node
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
///

#include <chrono>
#include <cstddef>
#include <functional>
#include <memory>
#include <fstream>
#include <iostream>

#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "turtlelib/diff_drive.hpp"
#include "turtlelib/kalman.hpp"
#include "turtlelib/rigid2d.hpp"

#include "armadillo"

using namespace std::chrono_literals;
using std::placeholders::_1;

/// @brief Landmarks detection node
class Landmarks : public rclcpp::Node
{
public:
  Landmarks()
  : Node("landmarks")
  {

    lidar_sub = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&Landmarks::lidar_callback, this, _1));

  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub;

  void lidar_callback(const sensor_msgs::msg::LaserScan & lidar_data)
  {
    constexpr double THRESHOLD = 0.1;
    // Find clusters
    for (size_t i = 0; i < lidar_data.ranges.size()-1; i++)
    {
      const double r1 = lidar_data.ranges.at(i);
      const double phi1 = turtlelib::normalize_angle(turtlelib::deg2rad(i));
      const double r2 = lidar_data.ranges.at(i+1);
      const double phi2 = turtlelib::normalize_angle(turtlelib::deg2rad(i+1));
      
      const turtlelib::Vector2D v1 = turtlelib::Vector2D::from_polar(r1, phi1);
      const turtlelib::Vector2D v2 = turtlelib::Vector2D::from_polar(r2, phi2);
      const double d = turtlelib::distance(v1, v2);


      if (d > 0.0 and d <= THRESHOLD)
      {
        RCLCPP_INFO_STREAM(get_logger(),"(r1,phi1) = " << r1 << "," << turtlelib::rad2deg(phi1));
        RCLCPP_INFO_STREAM(get_logger(),"(r2,phi2) = " << r2 << "," << turtlelib::rad2deg(phi2));
        RCLCPP_INFO_STREAM(get_logger(),"v1 = " << v1);
        RCLCPP_INFO_STREAM(get_logger(),"v2 = " << v2);
        RCLCPP_INFO_STREAM(get_logger(),"d = " << d);
        RCLCPP_INFO_STREAM(get_logger(),"-------------------");
      }

    }
  }


};

/// @brief the main function to run the odometry node
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Landmarks>());
  rclcpp::shutdown();
  return 0;
}

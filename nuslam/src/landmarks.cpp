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

  void lidar_callback(const sensor_msgs::msg::LaserScan &lidar_data)
  {
    RCLCPP_INFO_STREAM(get_logger(),"lidar timestamp = " << lidar_data.header.stamp.sec);
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

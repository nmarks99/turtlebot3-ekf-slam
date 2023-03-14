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
#include <rclcpp/publisher.hpp>
#include <visualization_msgs/msg/detail/marker__struct.hpp>
#include <visualization_msgs/msg/detail/marker_array__struct.hpp>

#include "nuslam/circle_fitting.hpp"
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

/// @brief minimum amount of points to be considered a cluster
constexpr unsigned int MIN_CLUSTER_SIZE = 3;


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

    cluster_pub = create_publisher<visualization_msgs::msg::MarkerArray>
      ("/clusters", 10);
  }

private:
  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub;
  
  // Publishers
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cluster_pub;

  // Messages 
  visualization_msgs::msg::MarkerArray cluster_marker_arr;
 
  // Vector of Cluster objects representing all the clusters of points
  std::vector<Cluster> all_clusters;

  /// @brief fills in a marker array containing a spherical
  /// marker at the average (x,y) location of each cluster
  void fill_cluster_markers()
  {
    // Clear the markers from the last scan
    cluster_marker_arr.markers.clear();

    for (size_t i = 0; i < all_clusters.size(); i++)
    {
      
      auto p_avg = all_clusters.at(i).centroid();

      // RCLCPP_INFO_STREAM(get_logger(), "Cluster " << i << ":");
      // for (auto &v : all_clusters.at(i).as_vector())
      // {
        // RCLCPP_INFO_STREAM(get_logger(), v);
      // }
      // RCLCPP_INFO_STREAM(get_logger(),"avg (x,y) = " << p_avg);
      // RCLCPP_INFO_STREAM(get_logger(), "----------------------");

      visualization_msgs::msg::Marker cluster_marker;

      cluster_marker.header.stamp = get_clock()->now();
      cluster_marker.header.frame_id = "red/base_footprint";
      cluster_marker.id = i;
      cluster_marker.action = visualization_msgs::msg::Marker::ADD;
      cluster_marker.type = visualization_msgs::msg::Marker::SPHERE;
      cluster_marker.scale.x = 0.05; 
      cluster_marker.scale.y = 0.05; 
      cluster_marker.scale.z = 0.05;
      cluster_marker.pose.position.x = p_avg.x;
      cluster_marker.pose.position.y = p_avg.y;
      cluster_marker.pose.position.z = 0.05;
      cluster_marker.color.a = 1.0;
      cluster_marker.color.r = 0.4;
      cluster_marker.color.g = 0.4;
      cluster_marker.color.b = 0.2;
      cluster_marker_arr.markers.push_back(cluster_marker);
    }

    cluster_pub->publish(cluster_marker_arr);

  }

  void lidar_callback(const sensor_msgs::msg::LaserScan & lidar_data)
  {

    all_clusters.clear(); // clears clusters from the previous scan

    // check each point to see if it fits in an existing cluster,
    // if not, add it to a new cluster
    for (size_t i = 0; i < lidar_data.ranges.size(); i++)
    {
      // for each point
      const double r = lidar_data.ranges.at(i);
      if (turtlelib::almost_equal(r,0.0))
      {
        continue;
      }
      const double phi = turtlelib::normalize_angle(turtlelib::deg2rad(i));
      const turtlelib::Vector2D v = turtlelib::Vector2D::from_polar(r, phi);
      
      // for each cluster we have
      bool added = false;
      for (auto &cluster : all_clusters)
      {
        added = cluster.belongs(v);
        if (added)
        {
          break;
        }
      }
      if (not added)
      {
        // add the point as a new Cluster
        all_clusters.push_back(Cluster(v));
      }
    }

    // Remove clusters with fewer than 3 points
    for (size_t i = 0; i < all_clusters.size(); i++)
    {
      if (all_clusters.at(i).count() < MIN_CLUSTER_SIZE)
      {
        all_clusters.erase(all_clusters.begin()+i);
      }
    }
    
    // fill and publish cluster marker array for testing
    fill_cluster_markers();

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

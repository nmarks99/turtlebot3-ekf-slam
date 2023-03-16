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
#include <geometry_msgs/msg/detail/point__struct.hpp>
#include <rclcpp/duration.hpp>
#include <tuple>
#include <memory>
#include <fstream>
#include <iostream>
#include <rclcpp/publisher.hpp>

#include "nuslam/circle_fitting.hpp"
#include "nuslam/msg/detail/point_array__struct.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nuslam/msg/point_array.hpp"

#include "turtlelib/diff_drive.hpp"
#include "turtlelib/kalman.hpp"
#include "turtlelib/rigid2d.hpp"

#include "nuslam/circle_fitting.hpp"

#include "armadillo"

using namespace std::chrono_literals;
using std::placeholders::_1;

/// @brief minimum amount of points to be considered a cluster
constexpr unsigned int MIN_CLUSTER_SIZE = 4;

/// @brief true radius of the landmarks
constexpr double TRUE_RADIUS = 0.038;

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

    detected_landmarks_pub = create_publisher<nuslam::msg::PointArray>
      ("/detected_landmarks", 10);
  }

private:
  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub;
  
  // Publishers
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cluster_pub;
  rclcpp::Publisher<nuslam::msg::PointArray>::SharedPtr detected_landmarks_pub;

  // Messages 
  // visualization_msgs::msg::MarkerArray cluster_marker_arr;
 
  // Vector of Cluster objects representing all the clusters of points
  std::vector<Cluster> all_clusters;
  const std::tuple<double,double> mean_threshold{0.0,130.0}; // TODO: tune
  const std::tuple<double,double> true_threshold{TRUE_RADIUS,0.1};
  const double std_threshold = 0.15;


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

    // Remove clusters with too few points
    std::vector<Cluster> temp_cluster_vec;
    for (const auto &cluster : all_clusters)
    {
      if (cluster.count() >= MIN_CLUSTER_SIZE)
      {
        temp_cluster_vec.push_back(cluster);
      }
    }
    all_clusters = temp_cluster_vec;
    temp_cluster_vec.clear();
    
    nuslam::msg::PointArray point_arr;
    RCLCPP_INFO_STREAM(get_logger(),"----------------------------------");
    int count = 1;
    if (not all_clusters.empty())
    {
      for (const auto &cluster: all_clusters)
      {
        auto hkr = fit_circle(cluster);
        RCLCPP_INFO_STREAM(get_logger(),"Cluster " << count);
        RCLCPP_INFO_STREAM(get_logger(),"Center = " << std::get<0>(hkr));
        RCLCPP_INFO_STREAM(get_logger(),"Radius = " << std::get<1>(hkr));
        RCLCPP_INFO_STREAM(get_logger(),"----------------------------------");
        count ++; 
        if(is_circle(cluster,mean_threshold,std_threshold,true_threshold))
        {
          // publishe the landmarks (x,y) which is the center of the circle
          geometry_msgs::msg::Point center;
          center.x = std::get<0>(hkr).x;
          center.y = std::get<0>(hkr).y;
          center.z = 0.0;
          point_arr.points.push_back(center);
        }
      }
      detected_landmarks_pub->publish(point_arr);
    
  
      // Publisher cluster markers
      visualization_msgs::msg::MarkerArray cluster_marker_arr;

      for (size_t i = 0; i < all_clusters.size(); i++)
      {
        
        auto p_avg = all_clusters.at(i).centroid();

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
        cluster_marker.lifetime = rclcpp::Duration(210ms);
        cluster_marker_arr.markers.push_back(cluster_marker);
      }

      cluster_pub->publish(cluster_marker_arr);
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

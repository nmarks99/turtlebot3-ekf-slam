#include "nusim/utils.hpp"

void fill_obstacles(
  visualization_msgs::msg::MarkerArray & marker_arr,
  const std::vector<double> & obstacles_x, const std::vector<double> & obstacles_y,
  double obstacles_r)
{

  visualization_msgs::msg::Marker marker_msg;

  // if marker_arr already had markers in it, be sure
  // keep track of the marker id's
  int last_id = 0;
  if (marker_arr.markers.empty()) {
    last_id = -1;
  } else {
    last_id = marker_arr.markers.back().id;
  }

  // Creates a marker obstacle at each specified location
  size_t i = 0;
  for (i = 0; i < obstacles_x.size(); i++) {
    marker_msg.header.frame_id = "nusim/world";
    marker_msg.header.stamp = rclcpp::Clock{}.now();
    marker_msg.id = last_id + (i + 1);
    marker_msg.type = visualization_msgs::msg::Marker::CYLINDER;
    marker_msg.action = visualization_msgs::msg::Marker::ADD;
    marker_msg.scale.x = obstacles_r;
    marker_msg.scale.y = obstacles_r;
    marker_msg.scale.z = OBSTACLE_HEIGHT;
    marker_msg.pose.position.x = obstacles_x.at(i);
    marker_msg.pose.position.y = obstacles_y.at(i);
    marker_msg.pose.position.z = OBSTACLE_HEIGHT / 2.0;
    marker_msg.color.r = 1.0;
    marker_msg.color.g = 0.0;
    marker_msg.color.b = 0.0;
    marker_msg.color.a = 1.0;
    marker_arr.markers.push_back(marker_msg);     // pack Marker into MarkerArray
  }
}

void fill_walls(visualization_msgs::msg::MarkerArray & marker_arr, double X_LENGTH, double Y_LENGTH)
{

  visualization_msgs::msg::Marker marker_msg;
  tf2::Quaternion q;

  // if marker_arr already had markers in it, be sure
  // keep track of the marker id's
  int last_id = 0;
  if (marker_arr.markers.empty()) {
    last_id = -1;
  } else {
    last_id = marker_arr.markers.back().id;
  }

  for (size_t i = 0; i < 4; i++) {
    marker_msg.header.frame_id = "nusim/world";
    marker_msg.header.stamp = rclcpp::Clock{}.now();
    marker_msg.id = last_id + (i + 1);
    marker_msg.type = visualization_msgs::msg::Marker::CUBE;
    marker_msg.action = visualization_msgs::msg::Marker::ADD;
    marker_msg.color.r = 1.0;
    marker_msg.color.g = 0.0;
    marker_msg.color.b = 0.0;
    marker_msg.color.a = 1.0;
    marker_msg.scale.x = WALL_WIDTH;
    marker_msg.scale.z = WALL_HEIGHT;
    marker_msg.pose.position.z = WALL_HEIGHT / 2.0;

    if (i < 2) {
      marker_msg.scale.y = Y_LENGTH;
      marker_msg.pose.position.y = 0.0;
      if (i == 0) {
        marker_msg.pose.position.x = X_LENGTH / 2.0 + WALL_WIDTH / 2.0;
      } else {
        marker_msg.pose.position.x = -X_LENGTH / 2.0 - WALL_WIDTH / 2.0;
      }
    } else {
      q.setRPY(0.0, 0.0, 1.57);
      marker_msg.pose.orientation.x = q.x();
      marker_msg.pose.orientation.y = q.y();
      marker_msg.pose.orientation.z = q.z();
      marker_msg.pose.orientation.w = q.w();
      marker_msg.pose.position.x = 0.0;
      marker_msg.scale.y = X_LENGTH;
      if (i == 2) {
        marker_msg.pose.position.y = Y_LENGTH / 2.0 + WALL_WIDTH / 2.0;
      } else {
        marker_msg.pose.position.y = -Y_LENGTH / 2.0 - WALL_WIDTH / 2.0;
      }
    }

    // Pack Markers into MarkerArray
    marker_arr.markers.push_back(marker_msg);
  }
}

std::mt19937 & get_random()
{
  // Credit Matt Elwin: https://nu-msr.github.io/navigation_site/lectures/gaussian.html

  // static variables inside a function are created once and persist for the remainder of the program
  static std::random_device rd{};
  static std::mt19937 mt{rd()};
  // we return a reference to the pseudo-random number genrator object. This is always the
  // same object every time get_random is called
  return mt;
}

void fill_basic_sensor_obstacles(
  visualization_msgs::msg::MarkerArray & marker_arr,
  const std::vector<double> & obstacles_x, const std::vector<double> & obstacles_y,
  double obstacles_r, const turtlelib::Pose2D & true_pose,
  double max_range, double basic_sensor_variance)
{

  visualization_msgs::msg::Marker marker_msg;

  // if marker_arr already had markers in it, be sure
  // keep track of the marker id's
  int last_id = 0;
  if (marker_arr.markers.empty()) {
    last_id = -1;
  } else {
    last_id = marker_arr.markers.back().id;
  }

  // Create normal distribution for random numbers
  std::normal_distribution<> d(0.0, basic_sensor_variance);

  // Creates a marker obstacle at each specified location
  size_t i = 0;
  for (i = 0; i < obstacles_x.size(); i++) {
    // Create a Vector2D for the current obstacle (x,y)
    turtlelib::Vector2D _v{obstacles_x.at(i), obstacles_y.at(i)};

    // Get the transforms so we can publish the obstacles in the body frame
    turtlelib::Transform2D T_WO(_v);
    turtlelib::Transform2D T_WB(turtlelib::Vector2D{true_pose.x, true_pose.y}, true_pose.theta);
    auto T_BO = T_WB.inv() * T_WO;
    double ob_x_in_body = T_BO.translation().x;
    double ob_y_in_body = T_BO.translation().y;

    marker_msg.header.frame_id = "red/base_footprint";
    marker_msg.header.stamp = rclcpp::Clock{}.now();
    marker_msg.id = last_id + (i + 1);
    marker_msg.type = visualization_msgs::msg::Marker::CYLINDER;

    // DELETE if out of range, ADD if in range
    auto dist = turtlelib::distance(
      turtlelib::Vector2D{_v.x, _v.y},
      turtlelib::Vector2D{true_pose.x, true_pose.y});
    if (dist > max_range) {
      // RCLCPP_INFO_STREAM(rclcpp::get_logger("nusim/utils"), "obstacle " << _v.x << "," << _v.y << " OUT OF RANGE: distance = " << dist);
      marker_msg.action = visualization_msgs::msg::Marker::DELETE;
    } else {
      // RCLCPP_INFO_STREAM(rclcpp::get_logger("nusim/utils"), "obstacle " << _v.x << "," << _v.y << " IN RANGE: distance = " << dist);
      marker_msg.action = visualization_msgs::msg::Marker::ADD;
    }
    marker_msg.scale.x = obstacles_r;
    marker_msg.scale.y = obstacles_r;
    marker_msg.scale.z = OBSTACLE_HEIGHT;
    auto noisex = d(get_random());
    auto noisey = d(get_random());
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("nusim/utils"), "noisex = " << noisex);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("nusim/utils"), "noisey = " << noisey);
    marker_msg.pose.position.x = ob_x_in_body + noisex;
    marker_msg.pose.position.y = ob_y_in_body + noisey;
    marker_msg.pose.position.z = OBSTACLE_HEIGHT / 2.0;
    marker_msg.color.r = 1.0;
    marker_msg.color.g = 1.0;
    marker_msg.color.b = 0.0;
    marker_msg.color.a = 1.0;
    marker_arr.markers.push_back(marker_msg);     // pack Marker into MarkerArray
  }
}

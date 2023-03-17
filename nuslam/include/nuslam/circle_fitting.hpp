#ifndef CIRCLE_FITTING_INCLUDE_GUARD_HPP
#define CIRCLE_FITTING_INCLUDE_GUARD_HPP

#include <iostream>
#include <vector>
#include <numeric>
#include <armadillo>
#include <cassert>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include "turtlelib/rigid2d.hpp"

using turtlelib::Vector2D;

/// @brief Cluster defines a set of 2D points which are within
/// a threshold linear distance of each other.
/// Simply put, a Cluster is a type comprised of a
/// std::vector<Vector2D> and various useful methods
struct Cluster
{
private:
  double THRESHOLD = 0.1;
  std::vector<Vector2D> cluster_vec;

public:
  /// @brief default constructor creates an empty Cluster
  Cluster();

  /// @brief creates an empty Cluster
  /// with the specified distance threshold
  Cluster(double threshold);

  /// @brief constructor that takes a Vector2D and adds it to the cluster
  /// @param p a Vector2D to be added to this new cluster
  /// @param threshold a double which defines the threshold distance for
  /// points to be considered part of the cluster
  Cluster(Vector2D p_new);

  /// @brief constructor that takes a Vector2D and adds it to the cluster
  /// as well as a distance threshold
  /// @param p a Vector2D to be added to this new cluster
  /// @param threshold a double which defines the threshold distance for
  /// points to be considered part of the cluster
  Cluster(Vector2D p_new, double threshold);

  /// @brief returns true if the Cluster contains the point
  bool contains(Vector2D p_new);

  /// @brief checks if the point belongs in the cluster, if it does, adds it
  /// @param p a Vector2D
  /// @return boolean true if point belongs in cluster, else false
  bool belongs(Vector2D p_new);

  /// @brief adds a point to the cluster without checking if it belongs.
  /// The point is not added if it is already in the cluster
  /// @param p a Vector2D
  /// @return boolean true if add was successful, else false
  bool blind_add(Vector2D p_new);

  /// @brief computes the centroid (mean x,y) in the cluster
  /// @return Vector2D of the centroid (x,y)
  Vector2D centroid() const;

  /// @brief returns the cluster as a std::vector<Vector2D>
  std::vector<Vector2D> as_vector() const;

  /// @brief returns the number of points in the cluster
  size_t count() const;

};

/// @brief Attempts to fit a circle to the points in
/// the given cluster, returning the center and radius
/// @param cluster a Cluster object defining a cluster of 2D points
/// @returns std::tuple<Vector2D,double> (center,radius)
std::tuple<Vector2D, double> fit_circle(const Cluster & cluster);

/// @brief classifies whether a cluster is a circle or not
/// Here you can specify the expected radius and threshold.
/// if the fit radius is outside the threshold percentage,
/// this function returns false
/// @param cluster a Cluster object
/// @param mean_threshold the lower and upper mean threshold
/// @param std_threshold the standard deviation threshold
/// @param rad_thresh the true radius and a percentage the
/// predicted radius must be within to be considered
bool is_circle(
  const Cluster & cluster,
  std::tuple<double, double> mean_threshold,
  double std_threshold,
  std::tuple<double, double> rad_thresh);

/// @brief classifies whether a cluster is a circle or not
bool is_circle(
  const Cluster & cluster,
  std::tuple<double, double> mean_threshold,
  double std_threshold);

namespace vec
{
/// @brief computes the mean of the elements in a std::vector<double>
double mean(const std::vector<double> & v);

double standard_deviation(const std::vector<double> & v);
}


#endif

#include <catch2/catch_test_macros.hpp>
#include <future>
#include <turtlelib/rigid2d.hpp>
#include <vector>
#include "nuslam/circle_fitting.hpp"

using turtlelib::almost_equal;
using turtlelib::Vector2D;

TEST_CASE("Cluster()", "[Cluster]")
{
  // Create a cluster with a point at 1,1 and
  // a distance threshold of 0.01
  Cluster cluster(Vector2D{1.0, 1.0}, 0.01);

  // Add a new point if it belongs (it does)
  turtlelib::Vector2D p{1.001, 1.0};
  REQUIRE(cluster.belongs(p));

  // Size should be two since the new point should be added
  REQUIRE(cluster.as_vector().size() == 2);

  // Add a new point if it belongs (this time it doesn't)
  Vector2D p2{2.0, 2.0};
  REQUIRE_FALSE(cluster.belongs(p2));

  // should still be the same size
  REQUIRE(cluster.as_vector().size() == 2);

}

TEST_CASE("contains()", "[Cluster]")
{
  Cluster cluster(Vector2D{1.0, 1.0}, 0.01);
  REQUIRE(cluster.contains(Vector2D{1.0, 1.0}));
  REQUIRE_FALSE(cluster.contains(Vector2D{2.0, 2.0}));
}

TEST_CASE("centroid()", "[Cluster]")
{
  Cluster cluster;

  cluster.blind_add(Vector2D{0.0, 0.0});
  cluster.blind_add(Vector2D{1.0, 1.0});
  cluster.blind_add(Vector2D{1.0, 0.0});
  cluster.blind_add(Vector2D{0.0, 1.0});

  auto avg_xy = cluster.centroid();

  REQUIRE(almost_equal(avg_xy.x, 0.5));
  REQUIRE(almost_equal(avg_xy.y, 0.5));
}

TEST_CASE("vec::mean()")
{
  std::vector<double> v{10.0, 20.0};
  double avg = vec::mean(v);
  REQUIRE(almost_equal(avg, 15.0));
}

TEST_CASE("vec::standard_deviation()")
{
  std::vector<double> v1{10.0, 12.0, 23.0, 9.0};
  double std_dev = vec::standard_deviation(v1);
  REQUIRE(almost_equal(std_dev, 5.59017, 1e-4));
}

TEST_CASE("fit_circle()")
{
  SECTION("Case 1: from circle fitting notes")
  {
    Cluster cluster;
    cluster.blind_add(Vector2D{1.0, 7.0});
    cluster.blind_add(Vector2D{2.0, 6.0});
    cluster.blind_add(Vector2D{5.0, 8.0});
    cluster.blind_add(Vector2D{7.0, 7.0});
    cluster.blind_add(Vector2D{9.0, 5.0});
    cluster.blind_add(Vector2D{3.0, 7.0});

    auto hkr = fit_circle(cluster);
    Vector2D center = std::get<0>(hkr);
    double R = std::get<1>(hkr);

    REQUIRE(almost_equal(center.x, 4.615482, 10e-4));
    REQUIRE(almost_equal(center.y, 2.807354, 10e-4));
    REQUIRE(almost_equal(R, 4.8275, 10e-4));
  }

  SECTION("Case 2: from circle fitting notes")
  {
    Cluster cluster;
    cluster.blind_add(Vector2D{-1.0, 0.0});
    cluster.blind_add(Vector2D{-0.3, -0.06});
    cluster.blind_add(Vector2D{0.3, 0.1});
    cluster.blind_add(Vector2D{1.0, 0.0});

    auto hkr = fit_circle(cluster);
    Vector2D center = std::get<0>(hkr);
    double R = std::get<1>(hkr);

    REQUIRE(almost_equal(center.x, 0.4908357, 10e-4));
    REQUIRE(almost_equal(center.y, -22.15212, 10e-4));
    REQUIRE(almost_equal(R, 22.17979, 10e-4));
  }


}


//
//
// NOTE: Previously passed these tests before making these
// functions private to the circle_fitting.cpp file. Not
// sure of a good way to include them here while still
// retaining their static nature

// TEST_CASE("compute_zi()")
// {
// Vector2D v{2.0,2.0};
// Vector2D com{1.0,1.0};
// double zi = compute_zi(v, com);
// REQUIRE(almost_equal(zi, 2.0));
// }
//
//
// TEST_CASE("construct_Z()")
// {
// Cluster cluster;
// cluster.blind_add(Vector2D{1.01,1.02});
// cluster.blind_add(Vector2D{1.03,1.04});
// cluster.blind_add(Vector2D{1.04,1.03});
//
// Vector2D centroid = cluster.centroid();
// std::vector<double> z_vec;
// for (auto &p : cluster.as_vector())
// {
// z_vec.push_back(compute_zi(p,centroid));
// }
//
// arma::mat Z = compute_Z(cluster,z_vec,centroid);
//
// REQUIRE(almost_equal(Z(0,1), 1.01-centroid.x));
// REQUIRE(almost_equal(Z(0,2), 1.02-centroid.y));
// REQUIRE(almost_equal(Z(1,1), 1.03-centroid.x));
// REQUIRE(almost_equal(Z(1,2), 1.04-centroid.y));
// }
//
// TEST_CASE("compute_M()")
// {
// Cluster cluster;
// cluster.blind_add(Vector2D{1.01,1.02});
// cluster.blind_add(Vector2D{1.03,1.04});
// cluster.blind_add(Vector2D{1.04,1.03});
//
// Vector2D centroid = cluster.centroid();
// std::vector<double> z_vec;
// for (auto &p : cluster.as_vector())
// {
// z_vec.push_back(compute_zi(p,centroid));
// }
//
// arma::mat Z = compute_Z(cluster,z_vec,centroid);
// arma::mat M = compute_M(Z);
// }
//
// TEST_CASE("compute_H()")
// {
// Cluster cluster;
// cluster.blind_add(Vector2D{1.01,1.02});
// cluster.blind_add(Vector2D{1.03,1.04});
// cluster.blind_add(Vector2D{1.04,1.03});
//
// Vector2D centroid = cluster.centroid();
// std::vector<double> z_vec;
// for (auto &p : cluster.as_vector())
// {
// z_vec.push_back(compute_zi(p,centroid));
// }
//
// double z_bar = vec::mean(z_vec);
// arma::mat H = compute_H(z_bar);
//
// }
//
//
// TEST_CASE("compute_Hinv()")
// {
// Cluster cluster;
// cluster.blind_add(Vector2D{1.01,1.02});
// cluster.blind_add(Vector2D{1.03,1.04});
// cluster.blind_add(Vector2D{1.04,1.03});
//
// Vector2D centroid = cluster.centroid();
// std::vector<double> z_vec;
// for (auto &p : cluster.as_vector())
// {
// z_vec.push_back(compute_zi(p,centroid));
// }
//
// double z_bar = vec::mean(z_vec);
// arma::mat Hinv = compute_Hinv(z_bar);
//
// }
//

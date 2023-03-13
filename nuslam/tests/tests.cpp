#include <catch2/catch_test_macros.hpp>
#include <turtlelib/rigid2d.hpp>
#include <vector>
#include "nuslam/circle_fitting.hpp"

using turtlelib::almost_equal;
using turtlelib::Vector2D;

TEST_CASE("Cluster()", "[Cluster]")
{
    // Create a cluster with a point at 1,1 and 
    // a distance threshold of 0.01
    Cluster cluster(Vector2D{1.0,1.0}, 0.01);
    
    // Add a new point if it belongs (it does)
    turtlelib::Vector2D p{1.001,1.0};
    cluster.belongs(p);

    // Size should be two since the new point should be added
    REQUIRE(cluster.get_vector().size() == 2); 

    // Add a new point if it belongs (this time it doesn't)
    Vector2D p2{2.0,2.0};
    cluster.belongs(p2);
    
    // should still be the same size
    REQUIRE(cluster.get_vector().size() == 2); 

}

TEST_CASE("contains()", "[Cluster]")
{
    Cluster cluster(Vector2D{1.0,1.0}, 0.01);
    REQUIRE(cluster.contains(Vector2D{1.0,1.0}));
    REQUIRE_FALSE(cluster.contains(Vector2D{2.0,2.0}));
}

TEST_CASE("mean_point()","[Cluster]")
{
    Cluster cluster;

    cluster.blind_add(Vector2D{0.0,0.0});
    cluster.blind_add(Vector2D{1.0,1.0});
    cluster.blind_add(Vector2D{1.0,0.0});
    cluster.blind_add(Vector2D{0.0,1.0});

    auto avg_xy = cluster.mean_point();
    
    REQUIRE(almost_equal(avg_xy.x, 0.5));
    REQUIRE(almost_equal(avg_xy.y, 0.5));
}

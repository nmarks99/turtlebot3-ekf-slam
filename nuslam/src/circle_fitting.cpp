#include "nuslam/circle_fitting.hpp"
#include <turtlelib/rigid2d.hpp>

using turtlelib::almost_equal;

Cluster::Cluster() {}


Cluster::Cluster(double threshold) : THRESHOLD(threshold) {}


Cluster::Cluster(turtlelib::Vector2D p_new, double threshold) : THRESHOLD(threshold)
{
    cluster_vec.push_back(p_new);
}


Cluster::Cluster(turtlelib::Vector2D p_new)
{
    cluster_vec.push_back(p_new);
}

bool Cluster::contains(turtlelib::Vector2D p_new)
{
    for (auto &p : cluster_vec)
    {
        if (almost_equal(p.x, p_new.x) and almost_equal(p.y, p_new.y))
        {
            return true;
        }
    }
    return false;
}


bool Cluster::belongs(turtlelib::Vector2D p_new)
{
    for (auto &p : cluster_vec)
    {
        if (not contains(p_new))
        {
            // if the new point is within the threshold
            // distance of any of the other points, add it
            // and return true
            const double d = turtlelib::distance(p, p_new);
            if (d > 0.0 and d < THRESHOLD)
            {
                cluster_vec.push_back(p_new);
                return true;
            }
        }
    }
    return false;
}

bool Cluster::blind_add(turtlelib::Vector2D p_new)
{
    if (contains(p_new))
    {
        return false;
    }
    else
    {
        cluster_vec.push_back(p_new);
        return true;
    }

}


turtlelib::Vector2D Cluster::mean_point() const
{
    double sum_x = 0.0;
    double sum_y = 0.0;
    auto n = cluster_vec.size();
    for (auto &c : cluster_vec)
    {
        sum_x += c.x;
        sum_y += c.y;
    }
    
    return turtlelib::Vector2D{sum_x/n, sum_y/n};

}


std::vector<turtlelib::Vector2D> Cluster::get_vector() const
{
    return cluster_vec;
}

size_t Cluster::count() const
{
    return cluster_vec.size();
}



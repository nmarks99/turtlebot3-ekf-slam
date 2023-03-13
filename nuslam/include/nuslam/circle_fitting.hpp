#ifndef CIRCLE_FITTING_INCLUDE_GUARD_HPP
#define CIRCLE_FITTING_INCLUDE_GUARD_HPP

#include <iostream>
#include <vector>
#include "turtlelib/rigid2d.hpp"

/// @brief Cluster defines a set of 2D points which are within
/// a threshold linear distance of each other
struct Cluster
{
    private:
        double THRESHOLD = 0.1;
        std::vector<turtlelib::Vector2D> cluster_vec;
        
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
        Cluster(turtlelib::Vector2D p_new);

        /// @brief constructor that takes a Vector2D and adds it to the cluster
        /// as well as a distance threshold
        /// @param p a Vector2D to be added to this new cluster
        /// @param threshold a double which defines the threshold distance for
        /// points to be considered part of the cluster
        Cluster(turtlelib::Vector2D p_new, double threshold);

        /// @brief returns true if the Cluster contains the point
        bool contains(turtlelib::Vector2D p_new);

        /// @brief checks if the point belongs in the cluster, if it does, adds it
        /// @param p a Vector2D
        /// @return boolean true if point belongs in cluster, else false
        bool belongs(turtlelib::Vector2D p_new);

        /// @brief adds a point to the cluster without checking if it belongs.
        /// The point is not added if it is already in the cluster
        /// @param p a Vector2D
        /// @return boolean true if add was successful, else false
        bool blind_add(turtlelib::Vector2D p_new);
        
        /// @brief computes the average (x,y) in the cluster
        /// @return Vector2D of the average (x,y)
        turtlelib::Vector2D mean_point() const;

        /// @brief returns the cluster as a vector of Vector2D points
        std::vector<turtlelib::Vector2D> get_vector() const;
        
        /// @brief returns the number of points in the cluster
        size_t count() const;
    
};


#endif

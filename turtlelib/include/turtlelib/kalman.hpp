#ifndef KALMAN_INCLUDE_GUARD_HPP
#define KALMAN_INCLUDE_GUARD_HPP
/// @file
/// @brief Implemenation of the Exteneded Kalman Filter algorithm

#include <iosfwd>
#include <cstdlib>
#include <vector>
#include <cmath>
#include <iostream>
#include <armadillo>
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"

namespace turtlelib
{

    // /// @brief Each landmark measurement contains a distance r,
    // /// bearing (angle) phi, and marker_id
    // struct LandmarkMeasurement
    // {
    //     double r;
    //     double phi;
    //     int marker_id;

    //     /// @brief Default constructor that initializes variables to zero
    //     LandmarkMeasurement();

    //     /// @brief Constructor which defines the r, phi, and marker_id
    //     /// @param _r range
    //     /// @param _phi bearing (angle)
    //     /// @param _marker_id integer id of the marker in the MarkerArray of landmarks
    //     LandmarkMeasurement(double _r, double _phi, int _marker_id);

    //     /// @brief Constructor which defines the r, phi, and marker_id
    //     /// given cartesian coordinates x and y, as well as the marker_id
    //     /// @param _x x coordinate
    //     /// @param _y y coordinate
    //     /// @param _marker_id integer id of the marker in the MarkerArray of landmarks
    //     static LandmarkMeasurement from_cartesian(double _x, double _y, int _marker_id);
    // };

    class KalmanFilter
    {

    private:
        Pose2D qt_hat{0.0, 0.0, 0.0};          // predicted robot state vector
        arma::mat sigma_hat = arma::mat(3, 3); // covariance
        arma::mat Q_mat = arma::mat(3, 3);     // process noise matrix
        // std::vector<LandmarkMeasurement> landmarks;

    public:
        /// @brief class constructor
        KalmanFilter();

        /// @brief extented Kalman filter prediction step which predicts
        /// the new robot state qt_hat at time t. The process noise is zero here.
        /// @param V a Twist2D at time t
        void predict(const Twist2D &V);
    };

}
#endif
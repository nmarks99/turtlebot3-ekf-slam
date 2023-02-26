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

    class KalmanFilter
    {

    private:
        Pose2D qt_hat;       // predicted robot state vector
        arma::mat sigma_hat; // covariance
        arma::mat Q_mat;

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
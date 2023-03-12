#ifndef KALMAN_INCLUDE_GUARD_HPP
#define KALMAN_INCLUDE_GUARD_HPP
/// @file
/// @brief Implemenation of the Exteneded Kalman Filter algorithm

#include <iosfwd>
#include <cstdlib>
#include <fstream>
#include <vector>
#include <cmath>
#include <iostream>
#include <armadillo>
#include <map>
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"

namespace turtlelib
{

    /// @brief Each landmark measurement contains a distance r,
    /// bearing (angle) phi, and marker_id (from ROS Marker msg).
    /// Angles are normalized to be in the range (-pi, pi]
    struct LandmarkMeasurement
    {
        /// @brief distance in meters to the landmark
        double r = 0.0;

        /// @brief  bearing(angle) to the landmark in radians
        double phi = 0.0;

        /// @brief integer id, typically associated with a ROS Marker message
        unsigned int marker_id = 0;

        /// @brief Default constructor that initializes variables to zero
        LandmarkMeasurement();

        /// @brief Constructor which defines the r, and phi
        /// @param _r range
        /// @param _phi bearing (angle)
        LandmarkMeasurement(double _r, double _phi);

        /// @brief Constructor which defines the r, phi, and marker_id
        /// @param _r range
        /// @param _phi bearing (angle)
        /// @param _marker_id integer id of the marker in the MarkerArray of landmarks
        LandmarkMeasurement(double _r, double _phi, int _marker_id);

        /// @brief Constructor which defines the r, phi, and marker_id
        /// given cartesian coordinates x and y, as well as the marker_id
        /// @param _x x coordinate
        /// @param _y y coordinate
        /// @param _marker_id integer id of the marker in the MarkerArray of landmarks
        static LandmarkMeasurement from_cartesian(double _x, double _y, int _marker_id);

        /// @brief Stores the r and phi values in a 2x1 arma::mat and returns it
        /// @return 2x1 arma::mat [r phi]
        arma::mat to_mat() const;

    };

    /// @brief Extended Kalman Filter for use with EKF-SLAM
    /// This was written with the Turtlebot3 in mind but could
    /// probably be used for a wide variety of applications
    class KalmanFilter
    {

    private:
        arma::mat Xi_hat;    // Full state prediction [pose, map state]
        arma::mat sigma_hat; // Covariance matrix
        arma::mat Q_bar;     // Process noise: Measure of how accurate the model is
        arma::mat R_bar;     // Sensor noise: Measure of how accurate the sensors are
        uint64_t n = 0;      // Number of landmarks

        /// @brief map (dictionary) of id:index key value pairs
        // the index is the index of the x_j component of the map_j so index+1 is y_j
        std::map<unsigned int, unsigned int> landmarks_dict;

        /// @brief computes the theoretical measurement given the current state estimate
        /// @param j index of x component of mt_j in Xi_hat
        arma::mat compute_h(unsigned int ind_in_Xi) const;

        /// @brief computes the derivative of h with respect to the state Xi
        /// @param j index of x component of mt_j in Xi_hat
        arma::mat compute_H(unsigned int ind_in_Xi) const;

        /// @brief takes a measurement and if it hasn't been seen before, initializes it,
        /// and adds it to the set of known landmark measurments.
        void update_measurements(const LandmarkMeasurement &measurement);

        /// @brief extented Kalman filter prediction step which predicts
        /// the new robot state qt_hat at time t using the pose estimate
        /// from the odometry calculation done elsewhere. The process noise is zero here.
        /// @param pose a Pose2D at time t
        /// @param V a Twist2D at time t
        void predict_from_odometry(const Pose2D &pose, const Twist2D &V);

        /// @brief extented Kalman filter update step
        /// @param measurements a vector of LandmarkMeasurements
        void update(const std::vector<LandmarkMeasurement> &measurements);

    public:
        /// @brief class constructor
        KalmanFilter();

        /// @brief class constructor that accepts Q and R gains
        /// @param Q process noise gain
        /// @param R measurment noise gain
        KalmanFilter(double Q, double R);

        /// @brief Runs one iteration of the extended Kalman filtera
        /// with the given twist and landmark measurements. The pose
        /// estimate from the prediction step comes from wheel odometry
        /// which is computed elsewhere and passed to this function
        /// @param pose a Pose2D of the current pose from the odometry
        /// @param V a Twist2D
        /// @param measurements a vector of LandmarkMeasurements
        void run(const Pose2D &pose, const Twist2D &V, const std::vector<LandmarkMeasurement> &measurements);

        /// @brief returns the current pose prediction
        /// @return an arma::mat of the prediction of the robot's current pose
        arma::mat pose_prediction() const;

        /// @brief returns the current map prediction
        /// @return an arma::mat of the prediction of the state of the map
        arma::mat map_prediction() const;

        /// @brief returns the current full state prediction
        /// @return an arma::mat of the prediction of the full state (robot+map)
        arma::mat state_prediction() const;
    };

}
#endif

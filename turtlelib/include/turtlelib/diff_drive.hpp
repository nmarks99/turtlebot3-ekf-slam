#ifndef DIFF_DRIVE_INCLUDE_GUARD_HPP
#define DIFF_DRIVE_INCLUDE_GUARD_HPP
/// @file
/// @brief models the kinematics of a differential drive robot

#include <iosfwd>
#include <cstdlib>
#include <vector>
#include <cmath>
#include <iostream>
#include <cassert>
#include <armadillo>
#include "turtlelib/rigid2d.hpp"

namespace turtlelib
{

    /// @brief a 2D robot pose (configuration)
    struct Pose2D
    {
        /// @brief the x position
        double x = 0.0;

        /// @brief the y position
        double y = 0.0;

        /// @brief the theta position
        double theta = 0.0;

        /// @brief converts the Pose2D into an armadillo Matrix ordered (theta, x, y)
        /// @return arma::mat object
        arma::mat to_mat();
    };

    /// @brief generic state of the left and right wheels.
    /// Could be position, velocity, or whatever
    struct WheelState
    {
        /// @brief state of the left wheel
        double left = 0.0;

        /// @brief state of the right wheel
        double right = 0.0;
    };

    /// @brief models the kinematics of a differential drive robot
    class DiffDrive
    {

    private:
        Pose2D _pose{0.0, 0.0, 0.0};  // x,y,theta position in meters, radians
        WheelState _phi{0.0, 0.0};    // wheel angles in radians
        WheelState _phidot{0.0, 0.0}; // wheel velocities in m/s

        // values default to turtlebot3 burger values
        // r and 2*D as shown in docs/Kinematics.pdf
        double WHEEL_RADIUS = 0.033; // r
        double TRACK_WIDTH = 0.160;  // 2*D

    public:
        /// @brief create a DiffDrive object with a default wheel radius
        /// and wheel separation. All other parameters are zero.
        DiffDrive();

        /// @brief create a DiffDrive object with given wheel radius and
        /// wheel separation. All other parameters are zero.
        /// @param wheel_radius - radius of the wheels on the robot
        /// @param wheel_separation - center to center distance between the wheels
        DiffDrive(double wheel_radius, double wheel_separation);

        /// @brief create a DiffDrive object with the provided pose
        /// @param pose - a Pose2D representing the current configuration of the robot
        DiffDrive(const Pose2D &pose);

        /// @brief create a DiffDrive object with provided pose and wheel angles
        /// @param pose - a Pose2D representing the current configuration of the robot
        /// @param phi - a WheelState of the current wheel angles in radians
        DiffDrive(const Pose2D &pose, const WheelState &phi);

        /// @brief create a DiffDrive object with provided pose, wheel angles, and velocities
        /// @param pose - a Pose2D representing the current configuration of the robot
        /// @param phi - a WheelState of the current wheel angles in radians
        /// @param phidot - a WheelState of the current wheel speeds in rad/s
        DiffDrive(const Pose2D &pose, const WheelState &phi, const WheelState &phidot);

        /// @brief computes the forward kinematics to find
        /// the new pose of robot given new wheel angles
        /// @param phi_new - the new wheel angles
        Pose2D forward_kinematics(WheelState phi_new);

        /// @brief computes the forward kinematics to find
        /// the new pose of robot given new wheel angles
        /// @param pose - the current Pose2D of the robot
        /// @param phi_new - the new wheel angles
        Pose2D forward_kinematics(const Pose2D &pose, const WheelState &phi_new);

        /// @brief computes the current body twist given wheel velocities
        /// @param phi_dot - WheelState of current wheel velocities
        /// @return the body twist of the robot as a Twist2D
        Twist2D body_twist(WheelState phi_dot);

        /// @brief computes the inverse kinematics to find the wheel
        /// speeds required to achieve the desired body twist
        /// @param V - the desired twist
        WheelState inverse_kinematics(Twist2D V);

        /// @brief returns the current robot configuration
        /// @return a Pose2D representing the current robot configuration
        Pose2D pose() const;

        /// @brief returns the current wheel angles as a WheelState
        /// @return the current wheel angles as a WheelState object
        WheelState wheel_angles() const;

        /// @brief returns the current wheel speeds
        /// @return the current wheel speeds as a WheelState object
        WheelState wheel_speeds() const;
    };

}

#endif
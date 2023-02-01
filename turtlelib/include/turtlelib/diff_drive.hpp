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
#include "turtlelib/rigid2d.hpp"

namespace turtlelib
{

    /// @brief a 2D robot pose (configuration)
    struct Pose2D
    {
        double x;
        double y;
        double theta;
    };

    /// @brief state of the left and right wheels.
    /// Could be position, velocity, or whatever is desired
    struct WheelState
    {
        double right;
        double left;
    };

    /// @brief models the kinematics of a differential drive robot
    class DiffDrive
    {

    private:
        Pose2D _pose{0, 0, 0};    // x,y,theta position in meters, radians
        WheelState _phi{0, 0};    // wheel angles in radians
        WheelState _phidot{0, 0}; // wheel velocities in m/s
        double WHEEL_RADIUS;
        double WHEEL_SEPARATION;

    public:
        /// @brief returns the current robot configuration
        /// @return a Pose2D representing the current robot configuration
        // Pose2D pose() const;

        /// @brief create a DiffDrive object with Pose and phi all zero
        /// @param wheel_radius - radius of the wheels on the robot
        /// @param wheel_separation - center to center distance between the wheels
        DiffDrive(double wheel_radius, double wheel_separation);

        /// @brief computes the forward kinematics to find
        /// the new pose of robot given new wheel angles
        /// @param phi_new - the new wheel angles
        Pose2D forward_kinematics(WheelState phi_new);

        /// @brief computes the inverse kinematics to find the wheel
        /// speeds required to achieve the desired body twist
        /// @param V - the desired twist
        WheelState inverse_kinematics(Twist2D V);
    };

}

#endif
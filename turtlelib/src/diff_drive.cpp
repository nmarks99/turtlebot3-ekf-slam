#include "turtlelib/diff_drive.hpp"

turtlelib::DiffDrive::DiffDrive() {}

turtlelib::DiffDrive::DiffDrive(double wheel_radius, double wheel_separation)
{
    WHEEL_RADIUS = wheel_radius;
    WHEEL_SEPARATION = wheel_separation;
}

turtlelib::WheelState turtlelib::DiffDrive::inverse_kinematics(Twist2D V)
{
    if (!almost_equal(V.ydot, 0.0))
    {
        throw std::logic_error("Non-zero y component of body twist is not possible");
    }

    // Compute the wheel speeds needed to achieve the desired body twist
    // Derivations for these equations can found in docs/Kinematics.pdf
    _phidot.left = (1 / WHEEL_RADIUS) * (-WHEEL_SEPARATION * V.thetadot + V.xdot);
    _phidot.right = (1 / WHEEL_RADIUS) * (WHEEL_SEPARATION * V.thetadot + V.xdot + V.ydot);
    return _phidot;
}

turtlelib::Pose2D turtlelib::DiffDrive::forward_kinematics(WheelState phi_new)
{
    // Define D and r so equations are shorter
    auto D = WHEEL_SEPARATION / 2;
    auto r = WHEEL_RADIUS;

    // Compute the new wheel speeds, which for t=1 are just
    // equal to the change in angle
    _phidot.left = (phi_new.left - _phi.left);
    _phidot.right = (phi_new.right - _phi.right);

    // Compute the body twist
    // Derivations for these equations can be found in docs/Kinematics.pdf
    turtlelib::Twist2D body_twist;
    body_twist.thetadot = (r / 2 * D) * (_phidot.right - _phidot.left);
    body_twist.xdot = (r * _phidot.right) - (D * body_twist.thetadot);
    body_twist.ydot = 0.0;

    // Define transform between world and B frame
    // B is the body frame before achieving the new wheel angles phi_new
    turtlelib::Vector2D xy_B{_pose.x, _pose.y};
    turtlelib::Transform2D Twb(xy_B, _pose.theta);

    // The transform between the B frame and the B' frame can be obtained
    // by integrating the twist. B' is the body frame once the robot has
    // achieved phi_new
    auto Tb_bprime = Twb.integrate_twist(body_twist);

    // Get the B' frame in the world frame by composing the transforms
    auto Tw_bprime = Twb * Tb_bprime;

    // Get the new pose from the Tw_bprime transform and return it
    _pose.theta = Tw_bprime.rotation();
    _pose.x = Tw_bprime.translation().x;
    _pose.y = Tw_bprime.translation().y;

    return _pose;
}

turtlelib::Twist2D turtlelib::DiffDrive::body_twist(turtlelib::WheelState phi_dot)
{
    // See Equation 6 in docs/Kinematics.pdf for where these equations come from
    turtlelib::Twist2D Vb;
    Vb.thetadot = (WHEEL_RADIUS / 2 * WHEEL_SEPARATION) * (phi_dot.right - phi_dot.left);
    Vb.xdot = (WHEEL_RADIUS / 2) * (phi_dot.left + phi_dot.right);
    Vb.ydot = 0.0; // no slipping
    return Vb;
}

turtlelib::Pose2D turtlelib::DiffDrive::pose() const
{
    return _pose;
}

turtlelib::WheelState turtlelib::DiffDrive::wheel_angles() const
{
    return _phi;
}

turtlelib::WheelState turtlelib::DiffDrive::wheel_speeds() const
{
    return _phidot;
}

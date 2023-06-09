#include "turtlelib/diff_drive.hpp"

namespace turtlelib
{

    arma::mat Pose2D::to_mat()
    {
        return arma::mat{theta, x, y};
    }

    DiffDrive::DiffDrive() {}

    DiffDrive::DiffDrive(double wheel_radius, double wheel_separation)
        : WHEEL_RADIUS(wheel_radius), TRACK_WIDTH(wheel_separation) {}

    DiffDrive::DiffDrive(const Pose2D &pose) : _pose(pose) {}

    DiffDrive::DiffDrive(const Pose2D &pose, const WheelState &phi)
        : _pose(pose), _phi(phi) {}

    DiffDrive::DiffDrive(const Pose2D &pose, const WheelState &phi, const WheelState &phidot)
        : _pose(pose), _phi(phi), _phidot(phidot) {}

    WheelState DiffDrive::inverse_kinematics(Twist2D V)
    {
        if (!almost_equal(V.ydot, 0.0))
        {
            throw std::logic_error("Non-zero y component of body twist is not possible");
        }

        // Define D and r so equations are shorter
        // These are the same as in docs/Kinematics.pdf
        static const auto D = TRACK_WIDTH / 2.0; // body radius
        static const auto r = WHEEL_RADIUS;

        // Compute the wheel speeds needed to achieve the desired body twist
        // Derivations for these equations can found in docs/Kinematics.pdf
        _phidot.left = (1 / r) * (-D * V.thetadot + V.xdot);
        _phidot.right = (1 / r) * (D * V.thetadot + V.xdot); // + V.ydot is ommitted
        return _phidot;
    }

    Pose2D DiffDrive::forward_kinematics(WheelState phi_new)
    {

        // Compute the new wheel speeds, which for t=1 are just
        // equal to the change in angle
        _phidot.left = (phi_new.left - _phi.left);
        _phidot.right = (phi_new.right - _phi.right);

        // update angles to be the new ones
        _phi = phi_new;

        // Compute the body twist
        // Derivations for these equations can be found in docs/Kinematics.pdf
        auto body_twist = DiffDrive::body_twist(_phidot);

        // Define transform between world and B frame
        // B is the body frame before achieving the new wheel angles phi_new
        Vector2D xy_B{_pose.x, _pose.y};
        Transform2D Twb(xy_B, _pose.theta);

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

    Pose2D DiffDrive::forward_kinematics(const Pose2D &pose, const WheelState &phi_new)
    {
        // define _pose as the pose passed to the function
        _pose = pose;

        // Compute the new wheel speeds, which for t=1 are just
        // equal to the change in angle
        _phidot.left = (phi_new.left - _phi.left);
        _phidot.right = (phi_new.right - _phi.right);

        // update angles to be the new ones
        _phi = phi_new;

        // Compute the body twist
        // Derivations for these equations can be found in docs/Kinematics.pdf
        auto body_twist = DiffDrive::body_twist(_phidot);

        // Define transform between world and B frame
        // B is the body frame before achieving the new wheel angles phi_new
        Vector2D xy_B{_pose.x, _pose.y};
        Transform2D Twb(xy_B, _pose.theta);

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

    Twist2D DiffDrive::body_twist(WheelState phi_dot)
    {
        // See Equation 6 in docs/Kinematics.pdf for where these equations come from
        Twist2D Vb;
        Vb.thetadot = (WHEEL_RADIUS / TRACK_WIDTH) * (phi_dot.right - phi_dot.left);
        Vb.xdot = (WHEEL_RADIUS / 2.0) * (phi_dot.left + phi_dot.right);
        Vb.ydot = 0.0; // no slipping
        return Vb;
    }

    Pose2D DiffDrive::pose() const
    {
        return _pose;
    }

    WheelState DiffDrive::wheel_angles() const
    {
        return _phi;
    }

    WheelState DiffDrive::wheel_speeds() const
    {
        return _phidot;
    }
}
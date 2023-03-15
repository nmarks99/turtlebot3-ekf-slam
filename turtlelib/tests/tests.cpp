#include <catch2/catch_test_macros.hpp>
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/kalman.hpp"
#include <iostream>
#include <cmath>
#include <sstream>

namespace turtlelib
{

    // =================
    //      rigid2D
    // =================

    TEST_CASE("check_quadrant")
    { // Nick, Marks
        REQUIRE(check_quadrant(1.0, 1.0, deg2rad(45.0)) == true);
        REQUIRE(check_quadrant(-1.0, 1.0, deg2rad(135.0)) == true);
        REQUIRE(check_quadrant(1.0, -1.0, deg2rad(315.0)) == true);
        REQUIRE(check_quadrant(-1.0, -1.0, deg2rad(225.0)) == true);
        REQUIRE(check_quadrant(-1.0, 1.0, deg2rad(315.0)) == false);
    }

    TEST_CASE("distance()", "[rigid2D]")
    { // Nick, Marks
        Vector2D p1{0.0, 0.0};
        Vector2D p2{1.0, 1.0};
        REQUIRE(almost_equal(distance(p1, p2), std::sqrt(2.0)));

        Vector2D p3{1.0,1.0};
        Vector2D p4{1.01,1.0};
        REQUIRE(almost_equal(distance(p3, p4), 0.01));
    }

    TEST_CASE("normalize_angle()", "[rigid2D]")
    { // Nick, Marks
        REQUIRE(almost_equal(normalize_angle(M_PI), M_PI));
        REQUIRE(almost_equal(normalize_angle(0), 0));
        REQUIRE(almost_equal(normalize_angle(-M_PI / 4), -M_PI / 4));
        REQUIRE(almost_equal(normalize_angle(3 * M_PI / 2), -M_PI / 2));
        REQUIRE(almost_equal(normalize_angle(-5 * M_PI / 2), -M_PI / 2));
    }

    TEST_CASE("()operator", "[Transform2D]")
    { // Nick, Marks
        Transform2D tf;
        Vector2D vec{1, 2};
        REQUIRE(tf(vec).x == vec.x);
        REQUIRE(tf(vec).y == vec.y);
    }

    TEST_CASE("inv()", "[Transform2D]")
    { // Nick, Marks
        Transform2D tf;
        REQUIRE(tf.inv().rotation() == tf.rotation());
        REQUIRE(tf.inv().translation().x == tf.translation().x);
        REQUIRE(tf.inv().translation().y == tf.translation().y);

        Vector2D vec{1, 1};
        double angle = M_PI / 2;
        Transform2D tf1(vec, angle);
        REQUIRE(almost_equal(tf1.inv().rotation(), -tf1.rotation()));
        REQUIRE(almost_equal(tf1.inv().translation().x, -1.0));
        REQUIRE(almost_equal(tf1.inv().translation().y, 1.0));
    }

    TEST_CASE("operator*=", "[Transform2D]")
    { // Nick, Marks
        Vector2D vec{1, 1};
        double angle = M_PI / 2;
        Transform2D tf1(vec, angle);
        Transform2D tf2(vec, angle);
        tf1 *= tf2;
        REQUIRE(almost_equal(tf1.rotation(), M_PI));

        Vector2D v1{4, 7};
        double a1 = M_PI / 3;
        Transform2D tf3(v1, a1);

        Vector2D v2{9, 3};
        double a2 = M_PI / 2;
        Transform2D tf4(v2, a2);
        tf3 *= tf4;
        REQUIRE(almost_equal(tf3.rotation(), (a1 + a2)));
        REQUIRE(almost_equal(tf3.translation().x, 5.90192, 1e-4));
        REQUIRE(almost_equal(tf3.translation().y, 16.29422, 1e-4));
    }

    TEST_CASE("map_twist()", "[Transform2D]")
    { // Nick, Marks
        double a1 = M_PI / 4;
        Vector2D vec1{8, 3};
        Transform2D tf(vec1, a1);
        Twist2D V{1.0, 2.0, 3.0};
        auto V_new = tf.map_twist(V);
        REQUIRE(almost_equal(V_new.thetadot, 1.0, 1e-4));
        REQUIRE(almost_equal(V_new.xdot, 2.29289, 1e-4));
        REQUIRE(almost_equal(V_new.ydot, -4.46447, 1e-4));
    }

    TEST_CASE("integrate_twist()", "[Transform2D]")
    {
        SECTION("Pure translation")
        {
            Transform2D tf;
            Twist2D V{0, 1, 0};
            auto tf_new = tf.integrate_twist(V);
            REQUIRE(almost_equal(tf_new.translation().x, 1.0));
            REQUIRE(almost_equal(tf_new.translation().y, 0.0));
            REQUIRE(almost_equal(tf_new.rotation(), 0.0));
        }
        SECTION("Pure rotation")
        {
            Transform2D tf;
            Twist2D V{M_PI, 0, 0};
            auto tf_new = tf.integrate_twist(V);
            REQUIRE(almost_equal(tf_new.translation().x, 0.0));
            REQUIRE(almost_equal(tf_new.translation().y, 0.0));
            REQUIRE(almost_equal(tf_new.rotation(), M_PI));
        }
        SECTION("Rotation and translation")
        {
            Transform2D tf;
            Twist2D V{2 * M_PI, M_PI, M_PI};
            auto tf_new = tf.integrate_twist(V);
            REQUIRE(almost_equal(tf_new.translation().x, 0.0));
            REQUIRE(almost_equal(tf_new.translation().y, 0.0));
            REQUIRE(almost_equal(tf_new.rotation(), 2 * M_PI));
        }
    }

    TEST_CASE("translation()", "[Transform2D]")
    { // Nick, Marks
        Transform2D tf;
        REQUIRE(almost_equal(tf.translation().x, 0));
        REQUIRE(almost_equal(tf.translation().y, 0));
    }

    TEST_CASE("rotation()"
              "[Transform2D]")
    { // Nick, Marks
        Transform2D tf1;
        REQUIRE(almost_equal(tf1.rotation(), 0.0));

        double angle = 1.57;
        Transform2D tf2(angle);
        REQUIRE(almost_equal(tf2.rotation(), 1.57));
    }

    TEST_CASE("operator<<", "[Transform2D]")
    { // Nick, Marks
        double a1 = M_PI / 4;
        Vector2D vec1{8, 3};
        Transform2D tf(vec1, a1);
        std::ostringstream out;
        out << tf;
        REQUIRE(out.str() == "deg:45 x:8 y:3");
    }

    TEST_CASE("operator>>", "[Transform2D]")
    { // Nick, Marks
        Transform2D tf;

        std::istringstream in;
        in.str("90 2 4");
        in >> tf;
        REQUIRE(almost_equal(tf.rotation(), M_PI / 2));
        REQUIRE(almost_equal(tf.translation().x, 2.0));
        REQUIRE(almost_equal(tf.translation().y, 4.0));
    }

    TEST_CASE("operator*", "[Transform2D]")
    { // Nick, Marks
        Vector2D v1{5, 3};
        Vector2D v2{2, 7};
        Transform2D tf1(v1);
        Transform2D tf2(v2);
        auto tf3 = tf1 * tf2;
        REQUIRE(almost_equal(tf3.rotation(), 0.0));
        REQUIRE(almost_equal(tf3.translation().x, 7.0));
        REQUIRE(almost_equal(tf3.translation().y, 10.0));
    }

    TEST_CASE("operator<<", "[Vector2D]")
    { // Nick, Marks
        Vector2D vec1{8, 3};
        std::ostringstream out;
        out << vec1;
        REQUIRE(out.str() == "[8 3]");
    }

    TEST_CASE("operator>>", "[Vector2D]")
    { // Nick, Marks
        Vector2D vec1;

        std::istringstream in;
        in.str("9 2");
        in >> vec1;
        REQUIRE(almost_equal(vec1.x, 9.0));
        REQUIRE(almost_equal(vec1.y, 2.0));
    }

    TEST_CASE("normalize()", "[Vector2D]")
    { // Nick, Marks
        Vector2D v{3, 2};
        auto norm_v = v.normalize();
        REQUIRE(almost_equal(norm_v.x, 0.83205029, 1e-4));
        REQUIRE(almost_equal(norm_v.y, 0.5547002, 1e-4));
    }

    TEST_CASE("dot()", "[Vector2D]")
    { // Nick, Marks
        Vector2D v1{2, 2};
        Vector2D v2{5, 5};
        REQUIRE(almost_equal(v1.dot(v2), 20));
    }

    TEST_CASE("magnitude()", "[Vector2D]")
    { // Nick, Marks
        Vector2D v1{2, 2};
        REQUIRE(almost_equal(v1.magnitude(), sqrt(8)));
    }

    TEST_CASE("angle()", "[Vector2D]")
    { // Nick, Marks
        Vector2D v1{1, 0};
        Vector2D v2{0, 1};
        REQUIRE(almost_equal(v1.angle(v2), M_PI / 2));
    }

    TEST_CASE("operator+=", "[Vector2D]")
    { // Nick, Marks
        Vector2D v1{1.0, 2.0};
        Vector2D v2{1.0, 2.0};
        v1 += v2;
        REQUIRE(almost_equal(v1.x, 2.0));
        REQUIRE(almost_equal(v1.y, 4.0));
    }

    TEST_CASE("operator+", "[Vector2D]")
    { // Nick, Marks
        Vector2D v1{1.0, 2.0};
        Vector2D v2{1.0, 2.0};
        REQUIRE(almost_equal((v1 + v2).x, 2.0));
        REQUIRE(almost_equal((v1 + v2).y, 4.0));
    }

    TEST_CASE("operator-=", "[Vector2D]")
    { // Nick, Marks
        Vector2D v1{3.0, 5.0};
        Vector2D v2{5.0, 2.0};
        v1 -= v2;
        REQUIRE(almost_equal(v1.x, -2.0));
        REQUIRE(almost_equal(v1.y, 3.0));
    }

    TEST_CASE("operator-", "[Vector2D]")
    { // Nick, Marks
        Vector2D v1{1.0, 2.0};
        Vector2D v2{2.0, 2.0};
        REQUIRE(almost_equal((v1 - v2).x, -1.0));
        REQUIRE(almost_equal((v1 - v2).y, 0.0));
    }

    TEST_CASE("operator*=", "[Vector2D]")
    { // Nick, Marks
        Vector2D v1{2.0, 3.0};
        v1 *= 2.0;
        REQUIRE(almost_equal(v1.x, 4.0));
        REQUIRE(almost_equal(v1.y, 6.0));
    }

    TEST_CASE("operator*", "[Vector2D]")
    { // Nick, Marks
        Vector2D v1{1.0, 2.0};
        auto result = v1 * 3.0;
        REQUIRE(almost_equal(result.x, 3.0));
        REQUIRE(almost_equal(result.y, 6.0));
    }

    TEST_CASE("from_polar()","[Twist2D]")
    { // Nick, Marks
        auto v = Vector2D::from_polar(1.0,M_PI/2.0);
        REQUIRE(almost_equal(v.x, 0.0));
        REQUIRE(almost_equal(v.y, 1.0));

        auto v1 = Vector2D::from_polar(std::sqrt(2.0),M_PI/4.0);
        REQUIRE(almost_equal(v1.x, 1.0));
        REQUIRE(almost_equal(v1.y, 1.0));

        auto v2 = Vector2D::from_polar(std::sqrt(2.0),-M_PI/4.0);
        REQUIRE(almost_equal(v2.x, 1.0));
        REQUIRE(almost_equal(v2.y, -1.0));
    }

    // =================
    //      Twist2D
    // =================

    TEST_CASE("operator<<", "[Twist2D]")
    { // Nick, Marks
        Twist2D V{4, 8, 3};
        std::ostringstream out;
        out << V;
        REQUIRE(out.str() == "[4 8 3]");
    }

    TEST_CASE("operator>>", "[Twist2D]")
    { // Nick, Marks
        Twist2D V;
        std::istringstream in;
        in.str("3 3 2");
        in >> V;
        REQUIRE(almost_equal(V.thetadot, 3.0));
        REQUIRE(almost_equal(V.xdot, 3.0));
        REQUIRE(almost_equal(V.ydot, 2.0));
    }

    // =================
    //     DiffDrive
    // =================

    // These values are just for the tests
    static const double D_TEST = 1.0;
    static const double r_TEST = 1.0;

    TEST_CASE("DiffDrive()", "[DiffDrive]")
    { // Nick, Marks
        SECTION("DiffDrive(pose)")
        {
            Pose2D pose{1.0, 2.0, 3.0};
            DiffDrive ddrive(pose);
            REQUIRE(almost_equal(ddrive.pose().x, 1.0));
            REQUIRE(almost_equal(ddrive.pose().y, 2.0));
            REQUIRE(almost_equal(ddrive.pose().theta, 3.0));
        }
        SECTION("DiffDrive(pose,phi)")
        {
            Pose2D pose{1.0, 2.0, 3.0};
            WheelState phi{5.0, 7.0};
            DiffDrive ddrive(pose, phi);
            REQUIRE(almost_equal(ddrive.pose().x, 1.0));
            REQUIRE(almost_equal(ddrive.pose().y, 2.0));
            REQUIRE(almost_equal(ddrive.pose().theta, 3.0));
            REQUIRE(almost_equal(ddrive.wheel_angles().left, 5.0));
            REQUIRE(almost_equal(ddrive.wheel_angles().right, 7.0));
        }

        SECTION("DiffDrive(pose, phi, phidot)")
        {
            Pose2D pose{1.0, 2.0, 3.0};
            WheelState phi{5.0, 7.0};
            WheelState phidot{8.0, 9.0};
            DiffDrive ddrive(pose, phi, phidot);
            REQUIRE(almost_equal(ddrive.pose().x, 1.0));
            REQUIRE(almost_equal(ddrive.pose().y, 2.0));
            REQUIRE(almost_equal(ddrive.pose().theta, 3.0));
            REQUIRE(almost_equal(ddrive.wheel_angles().left, 5.0));
            REQUIRE(almost_equal(ddrive.wheel_angles().right, 7.0));
            REQUIRE(almost_equal(ddrive.wheel_speeds().left, 8.0));
            REQUIRE(almost_equal(ddrive.wheel_speeds().right, 9.0));
        }
    }

    TEST_CASE("inverse_kinematics()", "[DiffDrive]")
    { // Nick, Marks
        SECTION("Robot drives forward")
        {
            DiffDrive turtlebot(r_TEST, D_TEST * 2);
            Twist2D V{0.0, 1.0, 0.0};
            WheelState speeds = turtlebot.inverse_kinematics(V);
            REQUIRE(almost_equal(speeds.left, 1.0));
            REQUIRE(almost_equal(speeds.right, 1.0));
        }

        SECTION("Robot drive backwards")
        {
            DiffDrive turtlebot(r_TEST, D_TEST * 2);
            Twist2D V{0, -1, 0};
            WheelState speeds = turtlebot.inverse_kinematics(V);
            REQUIRE(almost_equal(speeds.left, -1.0));
            REQUIRE(almost_equal(speeds.right, -1.0));
        }

        SECTION("Robot drives forwards and turns")
        {
            DiffDrive turtlebot(r_TEST, D_TEST * 2);
            Twist2D V{1.57, 1.57 * 2, 0};
            WheelState speeds = turtlebot.inverse_kinematics(V);
            REQUIRE(almost_equal(speeds.left, 1.57));
            REQUIRE(almost_equal(speeds.right, 4.71));
        }

        SECTION("Invalid requested body twist")
        {
            DiffDrive turtlebot(r_TEST, D_TEST * 2);
            Twist2D V{0, 0, 1};
            REQUIRE_THROWS(turtlebot.inverse_kinematics(V));
        }
    }

    TEST_CASE("forward_kinematics()", "[DiffDrive]")
    { // Nick, Marks
        SECTION("Robot drives forwards")
        {
            DiffDrive bot(r_TEST, D_TEST * 2);
            WheelState phi_new{1.57, 1.57};
            auto new_pose = bot.forward_kinematics(phi_new);
            REQUIRE(almost_equal(new_pose.x, 1.57));
            REQUIRE(almost_equal(new_pose.y, 0));
            REQUIRE(almost_equal(new_pose.theta, 0));
        }

        SECTION("Robot drives backwards")
        {
            DiffDrive bot(r_TEST, D_TEST * 2);
            WheelState phi_new{-1.57, -1.57};
            auto new_pose = bot.forward_kinematics(phi_new);
            REQUIRE(almost_equal(new_pose.x, -1.57));
            REQUIRE(almost_equal(new_pose.y, 0));
            REQUIRE(almost_equal(new_pose.theta, 0));
        }

        SECTION("Robot drives forwards and turns")
        {
            DiffDrive bot(r_TEST, D_TEST * 2);
            WheelState phi_new{M_PI, 0};
            auto new_pose = bot.forward_kinematics(phi_new);
            REQUIRE(almost_equal(new_pose.x, 1.0));
            REQUIRE(almost_equal(new_pose.y, -1.0));
            REQUIRE(almost_equal(new_pose.theta, -M_PI / 2));
        }

        SECTION("Robot spins in place")
        {
            DiffDrive bot;
            WheelState phi_new{-M_PI, M_PI};
            Pose2D pose{0.0, 0.0};
            auto new_pose = bot.forward_kinematics(phi_new);
            REQUIRE(almost_equal(new_pose.x, pose.x));
            REQUIRE(almost_equal(new_pose.y, pose.y));
        }
    }

    TEST_CASE("forward_kinematics(config,new_phi)", "[DiffDrive]")
    { // Nick, Marks
        SECTION("Robot spins in place")
        {
            Pose2D p{1.0, 1.0, 0.0};
            WheelState phi{0.0, 0.0};
            DiffDrive bot(p, phi);
            WheelState phi_new{1.57, -1.57};
            auto new_pose = bot.forward_kinematics(p, phi_new);
            REQUIRE(almost_equal(new_pose.x, p.x));
        }
    }

    TEST_CASE("pose()", "[DiffDrive]")
    { // Nick, Marks
        DiffDrive bot(0.1, 0.2);
        Pose2D q = bot.pose();
        REQUIRE(almost_equal(q.x, 0));
        REQUIRE(almost_equal(q.y, 0));
        REQUIRE(almost_equal(q.theta, 0));
    }

    TEST_CASE("wheel_angles()", "[DiffDrive]")
    { // Nick, Marks
        DiffDrive bot(0.1, 0.2);
        WheelState angles = bot.wheel_angles();
        REQUIRE(almost_equal(angles.left, 0.0));
        REQUIRE(almost_equal(angles.right, 0.0));
    }

    TEST_CASE("wheel_speeds()", "[DiffDrive]")
    { // Nick, Marks
        DiffDrive bot(0.1, 0.2);
        WheelState speeds = bot.wheel_speeds();
        REQUIRE(almost_equal(speeds.left, 0.0));
        REQUIRE(almost_equal(speeds.right, 0.0));
    }

    TEST_CASE("body_twist()", "[DiffDrive]")
    { // Nick, Marks
        DiffDrive bot;
        WheelState phi_dot{1.0, 1.0};
        Twist2D V = bot.body_twist(phi_dot);
        REQUIRE(almost_equal(V.xdot, 0.033));
    }

    // ====================
    //      KalmanFilter
    // ====================

    TEST_CASE("from_cartesian()", "[LandmarkMeasurement]")
    { // Nick, Marks
        auto ms = LandmarkMeasurement::from_cartesian(1.0, 1.0, 1);
        REQUIRE(almost_equal(ms.r, std::sqrt(2.0)));
        REQUIRE(almost_equal(ms.phi, M_PI / 4));
        REQUIRE(ms.marker_id == 1);
    }

    TEST_CASE("associate_measurements", "[KalmanFilter]")
    {
        auto m1 = LandmarkMeasurement::from_cartesian(1.0, 1.0, 0);
        auto m2 = LandmarkMeasurement::from_cartesian(3.0, 2.0, 0);
        auto m3 = LandmarkMeasurement::from_cartesian(1.0, 2.0, 0);
        std::vector<LandmarkMeasurement> measurments;
        measurments.push_back(m1);
        measurments.push_back(m2);
        measurments.push_back(m3);
        
    }

}

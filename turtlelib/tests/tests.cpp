#include <catch2/catch_test_macros.hpp>
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <iostream>
#include <cmath>
#include <sstream>

using turtlelib::almost_equal;

/*
=======
rigid2D
=======
*/

TEST_CASE("normalize_angle()", "[rigid2D]")
{ // Nick, Marks
    REQUIRE(almost_equal(turtlelib::normalize_angle(M_PI), M_PI));
    REQUIRE(almost_equal(turtlelib::normalize_angle(0), 0));
    REQUIRE(almost_equal(turtlelib::normalize_angle(-M_PI / 4), -M_PI / 4));
    REQUIRE(almost_equal(turtlelib::normalize_angle(3 * M_PI / 2), -M_PI / 2));
    REQUIRE(almost_equal(turtlelib::normalize_angle(-5 * M_PI / 2), -M_PI / 2));
}

TEST_CASE("()operator", "[Transform2D]")
{ // Nick, Marks
    turtlelib::Transform2D tf;
    turtlelib::Vector2D vec{1, 2};
    REQUIRE(tf(vec).x == vec.x);
    REQUIRE(tf(vec).y == vec.y);
}

TEST_CASE("inv()", "[Transform2D]")
{ // Nick, Marks
    turtlelib::Transform2D tf;
    REQUIRE(tf.inv().rotation() == tf.rotation());
    REQUIRE(tf.inv().translation().x == tf.translation().x);
    REQUIRE(tf.inv().translation().y == tf.translation().y);

    turtlelib::Vector2D vec{1, 1};
    double angle = M_PI / 2;
    turtlelib::Transform2D tf1(vec, angle);
    REQUIRE(almost_equal(tf1.inv().rotation(), -tf1.rotation()));
    REQUIRE(almost_equal(tf1.inv().translation().x, -1.0));
    REQUIRE(almost_equal(tf1.inv().translation().y, 1.0));
}

TEST_CASE("operator*=", "[Transform2D]")
{ // Nick, Marks
    turtlelib::Vector2D vec{1, 1};
    double angle = M_PI / 2;
    turtlelib::Transform2D tf1(vec, angle);
    turtlelib::Transform2D tf2(vec, angle);
    tf1 *= tf2;
    REQUIRE(almost_equal(tf1.rotation(), M_PI));

    turtlelib::Vector2D v1{4, 7};
    double a1 = M_PI / 3;
    turtlelib::Transform2D tf3(v1, a1);

    turtlelib::Vector2D v2{9, 3};
    double a2 = M_PI / 2;
    turtlelib::Transform2D tf4(v2, a2);
    tf3 *= tf4;
    REQUIRE(almost_equal(tf3.rotation(), (a1 + a2)));
    REQUIRE(almost_equal(tf3.translation().x, 5.90192, 1e-4));
    REQUIRE(almost_equal(tf3.translation().y, 16.29422, 1e-4));
}

TEST_CASE("map_twist()", "[Transform2D]")
{ // Nick, Marks
    double a1 = M_PI / 4;
    turtlelib::Vector2D vec1{8, 3};
    turtlelib::Transform2D tf(vec1, a1);
    turtlelib::Twist2D V{1.0, 2.0, 3.0};
    auto V_new = tf.map_twist(V);
    REQUIRE(almost_equal(V_new.thetadot, 1.0, 1e-4));
    REQUIRE(almost_equal(V_new.xdot, 2.29289, 1e-4));
    REQUIRE(almost_equal(V_new.ydot, -4.46447, 1e-4));
}

TEST_CASE("integrate_twist()", "[Transform2D]")
{
    SECTION("Pure translation")
    {
        turtlelib::Transform2D tf;
        turtlelib::Twist2D V{0, 1, 0};
        auto tf_new = tf.integrate_twist(V);
        REQUIRE(almost_equal(tf_new.translation().x, 1.0));
        REQUIRE(almost_equal(tf_new.translation().y, 0.0));
        REQUIRE(almost_equal(tf_new.rotation(), 0.0));
    }
    SECTION("Pure rotation")
    {
        turtlelib::Transform2D tf;
        turtlelib::Twist2D V{M_PI, 0, 0};
        auto tf_new = tf.integrate_twist(V);
        REQUIRE(almost_equal(tf_new.translation().x, 0.0));
        REQUIRE(almost_equal(tf_new.translation().y, 0.0));
        REQUIRE(almost_equal(tf_new.rotation(), M_PI));
    }
    SECTION("Rotation and translation")
    {
        turtlelib::Transform2D tf;
        turtlelib::Twist2D V{2 * M_PI, M_PI, M_PI};
        auto tf_new = tf.integrate_twist(V);
        REQUIRE(almost_equal(tf_new.translation().x, 0.0));
        REQUIRE(almost_equal(tf_new.translation().y, 0.0));
        REQUIRE(almost_equal(tf_new.rotation(), 2 * M_PI));
    }
}

TEST_CASE("translation()", "[Transform2D]")
{ // Nick, Marks
    turtlelib::Transform2D tf;
    REQUIRE(almost_equal(tf.translation().x, 0));
    REQUIRE(almost_equal(tf.translation().y, 0));
}

TEST_CASE("rotation()"
          "[Transform2D]")
{ // Nick, Marks
    turtlelib::Transform2D tf1;
    REQUIRE(almost_equal(tf1.rotation(), 0.0));

    double angle = 1.57;
    turtlelib::Transform2D tf2(angle);
    REQUIRE(almost_equal(tf2.rotation(), 1.57));
}

TEST_CASE("operator<<", "[Transform2D]")
{ // Nick, Marks
    double a1 = M_PI / 4;
    turtlelib::Vector2D vec1{8, 3};
    turtlelib::Transform2D tf(vec1, a1);
    std::ostringstream out;
    out << tf;
    REQUIRE(out.str() == "deg:45 x:8 y:3");
}

TEST_CASE("operator>>", "[Transform2D]")
{ // Nick, Marks
    turtlelib::Transform2D tf;

    std::istringstream in;
    in.str("90 2 4");
    in >> tf;
    REQUIRE(almost_equal(tf.rotation(), M_PI / 2));
    REQUIRE(almost_equal(tf.translation().x, 2.0));
    REQUIRE(almost_equal(tf.translation().y, 4.0));
}

TEST_CASE("operator*", "[Transform2D]")
{ // Nick, Marks
    turtlelib::Vector2D v1{5, 3};
    turtlelib::Vector2D v2{2, 7};
    turtlelib::Transform2D tf1(v1);
    turtlelib::Transform2D tf2(v2);
    auto tf3 = tf1 * tf2;
    REQUIRE(almost_equal(tf3.rotation(), 0.0));
    REQUIRE(almost_equal(tf3.translation().x, 7.0));
    REQUIRE(almost_equal(tf3.translation().y, 10.0));
}

TEST_CASE("operator<<", "[Vector2D]")
{ // Nick, Marks
    turtlelib::Vector2D vec1{8, 3};
    std::ostringstream out;
    out << vec1;
    REQUIRE(out.str() == "[8 3]");
}

TEST_CASE("operator>>", "[Vector2D]")
{ // Nick, Marks
    turtlelib::Vector2D vec1;

    std::istringstream in;
    in.str("9 2");
    in >> vec1;
    REQUIRE(almost_equal(vec1.x, 9.0));
    REQUIRE(almost_equal(vec1.y, 2.0));
}

TEST_CASE("normalize()", "[Vector2D]")
{ // Nick, Marks
    turtlelib::Vector2D v{3, 2};
    auto norm_v = v.normalize();
    REQUIRE(almost_equal(norm_v.x, 0.83205029, 1e-4));
    REQUIRE(almost_equal(norm_v.y, 0.5547002, 1e-4));
}

TEST_CASE("dot()", "[Vector2D]")
{ // Nick, Marks
    turtlelib::Vector2D v1{2, 2};
    turtlelib::Vector2D v2{5, 5};
    REQUIRE(almost_equal(v1.dot(v2), 20));
}

TEST_CASE("magnitude()", "[Vector2D]")
{ // Nick, Marks
    turtlelib::Vector2D v1{2, 2};
    REQUIRE(almost_equal(v1.magnitude(), sqrt(8)));
}

TEST_CASE("angle()", "[Vector2D]")
{ // Nick, Marks
    turtlelib::Vector2D v1{1, 0};
    turtlelib::Vector2D v2{0, 1};
    REQUIRE(almost_equal(v1.angle(v2), M_PI / 2));
}

TEST_CASE("operator+=", "[Vector2D]")
{ // Nick, Marks
    turtlelib::Vector2D v1{1.0, 2.0};
    turtlelib::Vector2D v2{1.0, 2.0};
    v1 += v2;
    REQUIRE(turtlelib::almost_equal(v1.x, 2.0));
    REQUIRE(turtlelib::almost_equal(v1.y, 4.0));
}

TEST_CASE("operator+", "[Vector2D]")
{ // Nick, Marks
    turtlelib::Vector2D v1{1.0, 2.0};
    turtlelib::Vector2D v2{1.0, 2.0};
    REQUIRE(turtlelib::almost_equal((v1 + v2).x, 2.0));
    REQUIRE(turtlelib::almost_equal((v1 + v2).y, 4.0));
}

TEST_CASE("operator-=", "[Vector2D]")
{ // Nick, Marks
    turtlelib::Vector2D v1{3.0, 5.0};
    turtlelib::Vector2D v2{5.0, 2.0};
    v1 -= v2;
    REQUIRE(turtlelib::almost_equal(v1.x, -2.0));
    REQUIRE(turtlelib::almost_equal(v1.y, 3.0));
}

TEST_CASE("operator-", "[Vector2D]")
{ // Nick, Marks
    turtlelib::Vector2D v1{1.0, 2.0};
    turtlelib::Vector2D v2{2.0, 2.0};
    REQUIRE(turtlelib::almost_equal((v1 - v2).x, -1.0));
    REQUIRE(turtlelib::almost_equal((v1 - v2).y, 0.0));
}

TEST_CASE("operator*=", "[Vector2D]")
{ // Nick, Marks
    turtlelib::Vector2D v1{2.0, 3.0};
    v1 *= 2.0;
    REQUIRE(almost_equal(v1.x, 4.0));
    REQUIRE(almost_equal(v1.y, 6.0));
}

TEST_CASE("operator*", "[Vector2D]")
{ // Nick, Marks
    turtlelib::Vector2D v1{1.0, 2.0};
    auto result = v1 * 3.0;
    REQUIRE(almost_equal(result.x, 3.0));
    REQUIRE(almost_equal(result.y, 6.0));
}

TEST_CASE("operator<<", "[Twist2D]")
{ // Nick, Marks
    turtlelib::Twist2D V{4, 8, 3};
    std::ostringstream out;
    out << V;
    REQUIRE(out.str() == "[4 8 3]");
}

TEST_CASE("operator>>", "[Twist2D]")
{ // Nick, Marks
    turtlelib::Twist2D V;
    std::istringstream in;
    in.str("3 3 2");
    in >> V;
    REQUIRE(almost_equal(V.thetadot, 3.0));
    REQUIRE(almost_equal(V.xdot, 3.0));
    REQUIRE(almost_equal(V.ydot, 2.0));
}

/*
==========
DiffDrive
==========
*/
TEST_CASE("inverse_kinematics()", "[DiffDrive]")
{
    turtlelib::DiffDrive turtlebot(0.1, 0.2);
    turtlelib::Twist2D V{0, 1, 0};
    turtlelib::WheelState speeds = turtlebot.inverse_kinematics(V);
}

TEST_CASE("forward_kinematics()", "[DiffDrive]")
{
    turtlelib::DiffDrive bot(0.1, 0.2);
    turtlelib::WheelState phi_new{1.57, 1.57};
    auto new_pose = bot.forward_kinematics(phi_new);
    std::cout << new_pose.x << std::endl;
    std::cout << new_pose.y << std::endl;
    std::cout << new_pose.theta << std::endl;
}

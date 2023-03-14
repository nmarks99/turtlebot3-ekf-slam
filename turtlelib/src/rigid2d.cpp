#include "turtlelib/rigid2d.hpp"
#include <iostream>

/*
============
Transform2D
============
*/
namespace turtlelib
{

    Transform2D::Transform2D()
        : angle(0.0)
    {
    }

    Transform2D::Transform2D(Vector2D trans)
        : p_vec(trans)
    {
    }

    Transform2D::Transform2D(double radians)
        : angle(radians)
    {
    }

    Transform2D::Transform2D(Vector2D trans, double radians)
        : angle(radians), p_vec(trans)
    {
    }

    Vector2D Transform2D::operator()(Vector2D v) const
    {

        return {
            v.x * std::cos(angle) - v.y * std::sin(angle) + p_vec.x,
            v.x * std::sin(angle) + v.y * std::cos(angle) + p_vec.y};
    }

    Transform2D Transform2D::inv() const
    {

        Transform2D tf_out;

        // Set new rotation angle
        tf_out.angle = -angle;

        // Set new translation vector
        tf_out.p_vec.x = -p_vec.x * cos(angle) - p_vec.y * sin(angle);
        tf_out.p_vec.y = p_vec.x * sin(angle) - p_vec.y * cos(angle);

        // Return new Transform2D object
        return tf_out;
    }

    Transform2D &Transform2D::operator*=(const Transform2D &rhs)
    {

        // Set new translation vector
        p_vec.x = p_vec.x + rhs.p_vec.x * cos(angle) - rhs.p_vec.y * sin(angle);
        p_vec.y = p_vec.y + rhs.p_vec.x * sin(angle) + rhs.p_vec.y * cos(angle);

        // Set new rotation angle
        angle = angle + rhs.angle;

        // Return modified Transform2D object
        return *this;
    }

    Twist2D Transform2D::map_twist(const Twist2D &V) const
    {
        Twist2D V_new;

        // Result obtained from V_i = Adjoint_ij*V_j and plugging in
        // values for given V and current Transform2D
        V_new.thetadot = V.thetadot;
        V_new.xdot = p_vec.y * V.thetadot + V.xdot * cos(angle) - V.ydot * sin(angle);
        V_new.ydot = -p_vec.x * V.thetadot + V.xdot * sin(angle) + V.ydot * cos(angle);

        return V_new;
    }

    Transform2D Transform2D::integrate_twist(const Twist2D &V) const
    {

        if (almost_equal(V.thetadot, 0.0))
        {
            return Transform2D(Vector2D{V.xdot, V.ydot}, 0.0);
        }
        else
        {
            double _angle = V.thetadot;
            Vector2D vec;
            vec.x = (1 / _angle) * (-V.ydot + V.xdot * sin(_angle) + V.ydot * cos(_angle));
            vec.y = (1 / _angle) * (V.xdot - V.xdot * cos(_angle) + V.ydot * sin(_angle));
            return Transform2D(vec, _angle);
        }
    }

    Vector2D Transform2D::translation() const
    {
        return p_vec;
    }

    double Transform2D::rotation() const
    {
        return angle;
    }

    std::ostream &operator<<(std::ostream &os, const Transform2D &tf)
    {
        os << "deg:" << rad2deg(tf.angle) << " x:" << tf.p_vec.x << " y:" << tf.p_vec.y;
        return os;
    }

    std::istream &operator>>(std::istream &is, Transform2D &tf)
    {

        double a;
        double px, py;
        is >> a >> px >> py;

        tf.angle = deg2rad(a);
        tf.p_vec.x = px;
        tf.p_vec.y = py;

        return is;
    }

    Transform2D operator*(Transform2D lhs, const Transform2D &rhs)
    {
        return lhs *= rhs;
    }

    /*
    =========
    Vector2D
    =========
    */

    std::ostream &operator<<(std::ostream &os, const Vector2D &v)
    {
        os << "[" << v.x << " " << v.y << "]";
        return os;
    }

    std::istream &operator>>(std::istream &is, Vector2D &v)
    {

        char ch = is.peek();
        if (ch == '[')
        {
            is.get(); // pop [
        }

        // First two chars should be v.x and v.y
        is >> v.x;
        is >> v.y;

        return is;
    }

    Vector2D Vector2D::normalize() const
    {
        Vector2D v_norm;

        if (x != 0 && y != 0)
        {
            auto mag = sqrt(pow(x, 2) + pow(y, 2));
            v_norm.x = x / mag;
            v_norm.y = y / mag;
        }
        else
        {
            v_norm.x = 0.0;
            v_norm.y = 0.0;
        }

        return v_norm;
    }

    double Vector2D::dot(const Vector2D &rhs) const
    {
        double result = (x * rhs.x) + (y * rhs.y);
        return result;
    }

    double Vector2D::magnitude() const
    {
        return sqrt(x * x + y * y);
    }

    double Vector2D::angle(const Vector2D &rhs) const
    {
        auto dot_prod = dot(rhs);
        auto prod_of_mag = magnitude() * rhs.magnitude();
        return acos(dot_prod / prod_of_mag);
    }

    Vector2D &Vector2D::operator+=(const Vector2D &rhs)
    {
        x = x + rhs.x;
        y = y + rhs.y;
        return *this;
    }

    Vector2D operator+(Vector2D lhs, const Vector2D &rhs)
    {
        return lhs += rhs;
    }

    Vector2D &Vector2D::operator-=(const Vector2D &rhs)
    {
        x = x - rhs.x;
        y = y - rhs.y;
        return *this;
    }

    Vector2D operator-(Vector2D lhs, const Vector2D &rhs)
    {
        return lhs -= rhs;
    }

    Vector2D &Vector2D::operator*=(double scalar)
    {
        x = x * scalar;
        y = y * scalar;
        return *this;
    }

    Vector2D operator*(Vector2D v, double scalar)
    {
        return v *= scalar;
    }

    Vector2D Vector2D::from_polar(double r, double phi)
    {
        return Vector2D{r*std::cos(phi), r*std::sin(phi)};
    }

    /*
    ========
    Twist2D
    ========
    */

    std::ostream &operator<<(std::ostream &os, const Twist2D &v)
    {
        os << "[" << v.thetadot << " " << v.xdot << " " << v.ydot << "]";
        return os;
    }

    std::istream &operator>>(std::istream &is, Twist2D &v)
    {
        char ch = is.peek();
        if (ch == '[')
        {
            is.get();
        }

        // First three chars should be thetadot, xdot, ydot
        is >> v.thetadot;
        is >> v.xdot;
        is >> v.ydot;

        return is;
    }
}

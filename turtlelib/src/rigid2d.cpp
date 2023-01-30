#include "turtlelib/rigid2d.hpp"

/*
============
Transform2D
============
*/

turtlelib::Transform2D::Transform2D()
{
    angle = 0.0;
}

turtlelib::Transform2D::Transform2D(turtlelib::Vector2D trans)
{
    p_vec = trans;
}

turtlelib::Transform2D::Transform2D(double radians)
{
    angle = radians;
}

turtlelib::Transform2D::Transform2D(Vector2D trans, double radians)
{
    angle = radians;
    p_vec = trans;
}

turtlelib::Vector2D turtlelib::Transform2D::operator()(turtlelib::Vector2D v) const
{

    turtlelib::Vector2D res;

    res.x = v.x * cos(angle) - v.y * sin(angle) + p_vec.x;
    res.y = v.x * sin(angle) + v.y * cos(angle) + p_vec.y;

    return res;
}

turtlelib::Transform2D turtlelib::Transform2D::inv() const
{

    turtlelib::Transform2D tf_out;

    // Set new rotation angle
    tf_out.angle = -angle;

    // Set new translation vector
    tf_out.p_vec.x = -p_vec.x * cos(angle) - p_vec.y * sin(angle);
    tf_out.p_vec.y = p_vec.x * sin(angle) - p_vec.y * cos(angle);

    // Return new Transform2D object
    return tf_out;
}

turtlelib::Transform2D &turtlelib::Transform2D::operator*=(const turtlelib::Transform2D &rhs)
{

    // Set new translation vector
    p_vec.x = p_vec.x + rhs.p_vec.x * cos(angle) - rhs.p_vec.y * sin(angle);
    p_vec.y = p_vec.y + rhs.p_vec.x * sin(angle) + rhs.p_vec.y * cos(angle);

    // Set new rotation angle
    angle = angle + rhs.angle;

    // Return modified Transform2D object
    return *this;
}

turtlelib::Twist2D turtlelib::Transform2D::map_twist(const turtlelib::Twist2D &V) const
{
    turtlelib::Twist2D V_new;

    // Result obtained from V_i = Adjoint_ij*V_j and plugging in
    // values for given V and current Transform2D
    V_new.thetadot = V.thetadot;
    V_new.xdot = p_vec.y * V.thetadot + V.xdot * cos(angle) - V.ydot * sin(angle);
    V_new.ydot = -p_vec.x * V.thetadot + V.xdot * sin(angle) + V.ydot * cos(angle);

    return V_new;
}

turtlelib::Transform2D turtlelib::Transform2D::integrate_twist(const turtlelib::Twist2D &V) const
{

    turtlelib::Vector2D vec;

    if (almost_equal(V.thetadot, 0.0))
    {
        vec.x = V.xdot;
        vec.y = V.ydot;
        turtlelib::Transform2D tf(vec, 0.0);
        return tf;
    }
    else
    {
        double _angle = V.thetadot;
        vec.x = (1 / _angle) * (-V.ydot + V.xdot * sin(_angle) + V.ydot * cos(_angle));
        vec.y = (1 / _angle) * (V.xdot - V.xdot * cos(_angle) + V.ydot * sin(_angle));
        turtlelib::Transform2D tf(vec, _angle);
        return tf;
    }
}

turtlelib::Vector2D turtlelib::Transform2D::translation() const
{
    return p_vec;
}

double turtlelib::Transform2D::rotation() const
{
    return angle;
}

std::ostream &turtlelib::operator<<(std::ostream &os, const turtlelib::Transform2D &tf)
{
    os << "deg:" << rad2deg(tf.angle) << " x:" << tf.p_vec.x << " y:" << tf.p_vec.y;
    return os;
}

std::istream &turtlelib::operator>>(std::istream &is, turtlelib::Transform2D &tf)
{

    double a;
    double px, py;
    is >> a >> px >> py;

    tf.angle = deg2rad(a);
    tf.p_vec.x = px;
    tf.p_vec.y = py;

    return is;
}

turtlelib::Transform2D turtlelib::operator*(turtlelib::Transform2D lhs, const turtlelib::Transform2D &rhs)
{
    return lhs *= rhs;
}

/*
=========
Vector2D
=========
*/

std::ostream &turtlelib::operator<<(std::ostream &os, const turtlelib::Vector2D &v)
{
    os << "[" << v.x << " " << v.y << "]";
    return os;
}

std::istream &turtlelib::operator>>(std::istream &is, turtlelib::Vector2D &v)
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

turtlelib::Vector2D turtlelib::Vector2D::normalize() const
{
    turtlelib::Vector2D v_norm;

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

double turtlelib::Vector2D::dot(const turtlelib::Vector2D &rhs) const
{
    double result = (x * rhs.x) + (y * rhs.y);
    return result;
}

double turtlelib::Vector2D::magnitude() const
{
    return sqrt(x * x + y * y);
}

double turtlelib::Vector2D::angle(const turtlelib::Vector2D &rhs) const
{
    auto dot_prod = this->dot(rhs);
    auto prod_of_mag = this->magnitude() * rhs.magnitude();
    return acos(dot_prod / prod_of_mag);
}

turtlelib::Vector2D &turtlelib::Vector2D::operator+=(const turtlelib::Vector2D &rhs)
{
    x = x + rhs.x;
    y = y + rhs.y;
    return *this;
}

turtlelib::Vector2D turtlelib::operator+(turtlelib::Vector2D lhs, const turtlelib::Vector2D &rhs)
{
    return lhs += rhs;
}

turtlelib::Vector2D &turtlelib::Vector2D::operator-=(const turtlelib::Vector2D &rhs)
{
    x = x - rhs.x;
    y = y - rhs.y;
    return *this;
}

turtlelib::Vector2D turtlelib::operator-(turtlelib::Vector2D lhs, const turtlelib::Vector2D &rhs)
{
    return lhs -= rhs;
}

turtlelib::Vector2D &turtlelib::Vector2D::operator*=(double scalar)
{
    x = x * scalar;
    y = y * scalar;
    return *this;
}

turtlelib::Vector2D turtlelib::operator*(turtlelib::Vector2D v, double scalar)
{
    return v *= scalar;
}

/*
========
Twist2D
========
*/

std::ostream &turtlelib::operator<<(std::ostream &os, const turtlelib::Twist2D &v)
{
    os << "[" << v.thetadot << " " << v.xdot << " " << v.ydot << "]";
    return os;
}

std::istream &turtlelib::operator>>(std::istream &is, turtlelib::Twist2D &v)
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

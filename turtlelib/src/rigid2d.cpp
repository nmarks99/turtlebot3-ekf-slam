#include "turtlelib/rigid2d.hpp"
#include <iostream>
#include <cassert>


/* 
=========================
Transform2D constructors 
=========================
*/

turtlelib::Transform2D::Transform2D() {

    std::vector<std::vector<double>> _tf
    {
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        {0.0, 0.0, 1.0}
    };
    tf_vec = _tf;
   
    angle = 0.0;
}


turtlelib::Transform2D::Transform2D(turtlelib::Vector2D trans){
    
    std::vector<std::vector<double>> _tf
    {
        {1.0, 0.0, trans.x},
        {0.0, 1.0, trans.y},
        {0.0, 0.0, 1.0}
    };
    tf_vec = _tf;
    
    p_vec = trans;

}


turtlelib::Transform2D::Transform2D(double radians) {

    std::vector<std::vector<double>> _tf
    {
        {cos(radians), -sin(radians), 0.0},
        {sin(radians), cos(radians), 0.0},
        {0.0, 0.0, 1.0}
    };

    tf_vec = _tf;

    angle = radians;
}


turtlelib::Transform2D::Transform2D(Vector2D trans, double radians) {
    
    std::vector<std::vector<double>> _tf
    {
        {cos(radians), -sin(radians), trans.x},
        {sin(radians), cos(radians), trans.y},
        {0.0, 0.0, 1.0}
    };

    tf_vec = _tf;

    angle = radians;
    p_vec = trans;

}


/* 
============================
Transform2D member functions
============================
*/

turtlelib::Vector2D turtlelib::Transform2D::operator()(turtlelib::Vector2D v) const {
    
    turtlelib::Vector2D res; 

    res.x = v.x*cos(angle) - v.y*sin(angle) + p_vec.x;
    res.y = v.x*sin(angle) + v.y*cos(angle) + p_vec.y;
        
    return res;
}


turtlelib::Transform2D turtlelib::Transform2D::inv() const {

    turtlelib::Transform2D tf_out;

    // Set new rotation angle 
    tf_out.angle = -angle;

    // Set new translation vector
    tf_out.p_vec.x = -p_vec.x * cos(angle) - p_vec.y * sin(angle);
    tf_out.p_vec.y = p_vec.x * sin(angle) - p_vec.y * cos(angle);

    // Return new Transform2D object
    return tf_out; 
    
}

turtlelib::Transform2D &turtlelib::Transform2D::operator*=(const turtlelib::Transform2D &rhs) {
    
    // Set new rotation angle 
    angle = angle + rhs.angle;

    // Set new translation vector
    p_vec.x = p_vec.x + rhs.p_vec.x*cos(angle) - rhs.p_vec.y*sin(angle);
    p_vec.y = p_vec.y + rhs.p_vec.x*cos(angle) + rhs.p_vec.y*cos(angle);

    // Return modified Transform2D object
    return *this;
}

turtlelib::Twist2D turtlelib::Transform2D::map_twist(turtlelib::Twist2D V) const {
    turtlelib::Twist2D V_new;

    // Result obtained from V_i = Adjoint_ij*V_j and plugging in 
    // values for given V and current Transform2D
    V_new.thetadot = V.thetadot;
    V_new.xdot = p_vec.y*V.thetadot + V.xdot*cos(angle) - V.ydot*sin(angle);
    V_new.ydot = -p_vec.x*V.thetadot + V.xdot*sin(angle) + V.ydot*cos(angle);

    return V_new;
}

turtlelib::Vector2D turtlelib::Transform2D::translation() const {
    return p_vec;
}


double turtlelib::Transform2D::rotation() const {
    return angle; 
}

std::ostream &turtlelib::operator<<(std::ostream &os, const turtlelib::Transform2D &tf)
{
    os<<
        "deg:"<<
        rad2deg(tf.angle)<<
        " x:"<<
        tf.p_vec.x<<
        " y:"<<
        tf.p_vec.y;
    return os;
}


/*
===========================
Vector2D operator overloads
===========================
*/

std::ostream &turtlelib::operator<<(std::ostream &os, const turtlelib::Vector2D &v) {
    os << "[" << v.x << " " << v.y << "]";
    return os;
}

std::istream &turtlelib::operator>>(std::istream &is, turtlelib::Vector2D & v) {
    
    char ch = is.peek();
    if (ch == '['){
        is.get(); // pop [
    }
    
    // First two chars should be v.x and v.y
    is >> v.x;
    is >> v.y;

    return is;
}


/*
================
Vector2D methods
================
*/

turtlelib::Vector2D turtlelib::Vector2D::normalize() const {
    turtlelib::Vector2D v_norm;

    if (x != 0 && y != 0) {
        auto mag = sqrt(pow(x,2) + pow(y,2));
        v_norm.x = x/mag;
        v_norm.y = y/mag;
    }
    else {
        v_norm.x = 0.0;
        v_norm.y = 0.0;
    }

    return v_norm;
}



/*
===========================
Twist2D operator overloads
===========================
*/

std::ostream &turtlelib::operator<<(std::ostream &os, const turtlelib::Twist2D &v){
    os << "[" << v.thetadot << " " << v.xdot << " " << v.ydot << "]";
    return os;
}

std::istream &turtlelib::operator>>(std::istream &is, turtlelib::Twist2D &v) {
    char ch = is.peek();
    if (ch == '['){
        is.get();
    }

    // First three chars should be thetadot, xdot, ydot
    is >> v.thetadot;
    is >> v.xdot;
    is >> v.ydot;

    return is;
}


/*
==============================
Transform2D operator overloads
==============================
*/

std::istream & turtlelib::operator>>(std::istream &is, turtlelib::Transform2D &tf) {
    
    double a;
    double px, py;
    is >> a >> px >> py;

    tf.angle = deg2rad(a);
    tf.p_vec.x = px;
    tf.p_vec.y = py;

    return is;

}

turtlelib::Transform2D turtlelib::operator*(turtlelib::Transform2D lhs, const turtlelib::Transform2D &rhs){
    return lhs *= rhs;
}





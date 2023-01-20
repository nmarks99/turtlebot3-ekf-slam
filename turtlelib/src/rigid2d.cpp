#include "turtlelib/rigid2d.hpp"
#include <iostream>
#include <cassert>


/*
===========================
Vector2D operator overloads
===========================
*/

std::ostream &turtlelib::operator<<(std::ostream &os, const turtlelib::Vector2D &v)
{
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
=========================
Transform2D Class Methods
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
   
    rotation_rad = 0.0;
}


turtlelib::Transform2D::Transform2D(turtlelib::Vector2D trans){
    
    std::vector<std::vector<double>> _tf
    {
        {1.0, 0.0, trans.x},
        {0.0, 1.0, trans.y},
        {0.0, 0.0, 1.0}
    };
    tf_vec = _tf;
    
    translation_vec = trans;

}


turtlelib::Transform2D::Transform2D(double radians) {

    std::vector<std::vector<double>> _tf
    {
        {cos(radians), -sin(radians), 0.0},
        {sin(radians), cos(radians), 0.0},
        {0.0, 0.0, 1.0}
    };

    tf_vec = _tf;

    rotation_rad = radians;
}


turtlelib::Transform2D::Transform2D(Vector2D trans, double radians) {
    
    std::vector<std::vector<double>> _tf
    {
        {cos(radians), -sin(radians), trans.x},
        {sin(radians), cos(radians), trans.y},
        {0.0, 0.0, 1.0}
    };

    tf_vec = _tf;

    rotation_rad = radians;
    translation_vec = trans;

}


turtlelib::Vector2D turtlelib::Transform2D::operator()(turtlelib::Vector2D v) const {
    
    turtlelib::Vector2D res; 

    res.x = v.x*cos(rotation_rad) - v.y*sin(rotation_rad) + translation_vec.x;
    res.y = v.x*sin(rotation_rad) + v.y*cos(rotation_rad) + translation_vec.y;
        
    return res;
}


turtlelib::Transform2D turtlelib::Transform2D::inv() const {

    turtlelib::Transform2D tf_out;
    tf_out.rotation_rad = -rotation_rad;

    tf_out.translation_vec.x =
        -translation_vec.x * cos(rotation_rad) - translation_vec.y * sin(rotation_rad);
    
    tf_out.translation_vec.y =
        translation_vec.x * sin(rotation_rad) - translation_vec.y * cos(rotation_rad);

    return tf_out;
    
}

// Transform2D & operator*=(const Transform2D & rhs) {

// }

turtlelib::Vector2D turtlelib::Transform2D::translation() const {
    return translation_vec;
}


double turtlelib::Transform2D::rotation() const {
    return rotation_rad; 
}


std::ostream &turtlelib::operator<<(std::ostream &os, const turtlelib::Transform2D &tf)
{
    os<<
        "deg:"<<
        rad2deg(tf.rotation_rad)<<
        " x:"<<
        tf.translation_vec.x<<
        " y:"<<
        tf.translation_vec.y;
    return os;
}



/*
=============================
Additional operator overloads
=============================
*/

std::ostream & turtlelib::operator<<(std::ostream &os, const turtlelib::Transform2D & tf);

std::istream & turtlelib::operator>>(std::istream &is, turtlelib::Transform2D &tf);

turtlelib::Transform2D turtlelib::operator*(turtlelib::Transform2D lhs, const turtlelib::Transform2D &rhs);





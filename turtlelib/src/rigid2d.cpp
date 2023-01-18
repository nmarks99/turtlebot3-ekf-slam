#include "turtlelib/rigid2d.hpp"
#include <iostream>

std::ostream &turtlelib::operator<<(std::ostream &os, const turtlelib::Vector2D &v)
{
    os << "[" << v.x << ", " << v.y << "]";
    return os;
}


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

    std::vector<double> v_vec = {v.x, v.y, 1.0};
    std::vector<double> v_res = {0.0, 0.0, 0.0};
    double _row = 0;
    for (auto i = 0; i < 3; i++){
        for (auto j = 0; j < 3; j++){
            _row += v_vec.at(j) * turtlelib::Transform2D::tf_vec.at(i).at(j); 
        }
        v_res[i] = _row;
        _row = 0;
    }
    
    res.x = v_res.at(0);
    res.y = v_res.at(1);
        
    return res;
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


turtlelib::Vector2D turtlelib::Transform2D::translation() const {
    return translation_vec;
}


double turtlelib::Transform2D::rotation() const {
    return rotation_rad; 
}


























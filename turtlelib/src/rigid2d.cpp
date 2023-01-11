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
    
    tf = _tf;

}


turtlelib::Transform2D::Transform2D(turtlelib::Vector2D trans){
    
    std::vector<std::vector<double>> _tf
    {
        {1.0, 0.0, trans.x},
        {0.0, 1.0, trans.y},
        {0.0, 0.0, 1.0}
    };

    tf = _tf;
}


turtlelib::Transform2D::Transform2D(double radians) {

    std::vector<std::vector<double>> _tf 
    {
        {cos(radians), -sin(radians), 0.0},
        {sin(radians), cos(radians), 0.0},
        {0.0, 0.0, 1.0}
    };

    tf = _tf;
}

        
void turtlelib::Transform2D::display() {
    
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++){
            if (almost_equal(0.0,tf[i][j])){
                tf[i][j] = 0.0; 
            }
            std::cout << tf[i][j] <<" ";
        }
        std::cout << std::endl;
    }

}









#include "turtlelib/rigid2d.hpp"
#include <iostream>

int main() {
    
    turtlelib::Vector2D v;
    v.x = 2.4;
    v.y = 3.3;
    // std::cout << v << std::endl;

    double rad = M_PI/2;
    turtlelib::Transform2D tf(v,rad);
    std::cout << tf << std::endl;

    return 0;
}

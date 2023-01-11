#include "turtlelib/rigid2d.hpp"
#include <iostream>

int main() {
    
    // turtlelib::Vector2D v;
    // v.x = 2.0;
    // v.y = 3.0;
    // std::cout << v << std::endl;

    double rad = M_PI/2;
    turtlelib::Transform2D tf(rad);
    tf.display();

    return 0;
}

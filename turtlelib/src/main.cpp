#include "turtlelib/rigid2d.hpp"
#include <iostream>

int main() {
    
    turtlelib::Vector2D v{x: 3.2, y:6.1};
    // double rad = M_PI/2;

    turtlelib::Transform2D tf;
    auto res = tf(v);
    std::cout << res << std::endl;
    
    return 0;
}

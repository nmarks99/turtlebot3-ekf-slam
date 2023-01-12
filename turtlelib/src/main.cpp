#include "turtlelib/rigid2d.hpp"
#include <iostream>

int main() {
    
    turtlelib::Vector2D v{x: 3.42, y:2.19};
    double rad = M_PI/2;

    turtlelib::Transform2D tf(v,rad);
    std::cout << "tf = " << tf << std::endl << std::endl;

    turtlelib::Vector2D tr = tf.translation();
    double rot = tf.rotation();

    std::cout << "translation = " << tr << std::endl;
    std::cout << "rotation = " << rot << std::endl;

    return 0;
}

#include "turtlelib/rigid2d.hpp"
#include <iostream>

// prints out a 3x3 matrix, useful for debugging 
void display(std::vector<std::vector<double>> vec) {
    std::cout.precision(3);
    unsigned int i, j;
    for(i = 0; i < vec.size(); i++) {
        for(j = 0; j < vec.at(0).size(); j++){
            if (fabs(vec[i][j]) <= 1e-6) {
                vec[i][j] = 0.0;
            }
            std::cout << vec[i][j] <<"\t";
        }
        std::cout << std::endl;
    }
}

int main() {

    // turtlelib::Twist2D V{0,1,0};
    // turtlelib::Vector2D vec{0,0};
    // turtlelib::Transform2D tf(vec, M_PI/2);
    // auto V_new = tf.map_twist(V);
    // std::cout << V_new << std::endl;

    turtlelib::Vector2D v{0,0};
    auto v_norm = v.normalize();
    std::cout << v_norm << std::endl;

}

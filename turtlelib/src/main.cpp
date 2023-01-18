#include "turtlelib/rigid2d.hpp"
#include <iostream>

// prints out a 3x3 matrix, useful for debugging 
void display(std::vector<std::vector<double>> vec) {

    for(auto i = 0; i < 3; i++) {
        for(auto j = 0; j < 3; j++){
            if (abs(vec[i][j]) <= 1e-6) {
                vec[i][j] = 0.0;
            }
            std::cout << vec[i][j] <<"\t";
        }
        std::cout << std::endl;
    }
}

int main() {
    
    double rad = M_PI/2;
    turtlelib::Vector2D p{x: 0.0, y:0.0};

    turtlelib::Transform2D tf(p, rad);

    turtlelib::Vector2D v{x: 1.0, y:1.0};
    auto res = tf(v);
    std::cout << res << std::endl;

    auto vec = tf.tf_vec;
    display(vec);

    
    return 0;
}

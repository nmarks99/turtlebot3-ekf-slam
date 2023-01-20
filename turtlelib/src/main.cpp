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

    turtlelib::Transform2D tf;
    
    std::cout << tf << std::endl;
    std::cin >> tf;
    std::cout << tf << std::endl;

}

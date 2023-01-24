#include "turtlelib/rigid2d.hpp"
#include <iostream>

int main() {

    // Create some transforms
    turtlelib::Transform2D Tab;
    turtlelib::Transform2D Tbc;

    // Prompt user to specify the transforms Tab and Tbc
    std::cout << "Enter transform T_{a,b}:" << std::endl;
    std::cin >> Tab;
    std::cout << "Enter transform T_{b,c}:" << std::endl;
    std::cin >> Tbc;

    // Compute the rest of the transforms we want
    auto Tba = Tab.inv();
    auto Tcb = Tbc.inv();
    auto Tac = Tab*Tbc;
    auto Tca = Tac.inv();

    // Show all the transforms
    std::cout << "T_{a,b} = " << Tab << std::endl;
    std::cout << "T_{b,a} = " << Tba << std::endl;
    std::cout << "T_{b,c} = " << Tbc << std::endl;
    std::cout << "T_{c,b} = " << Tcb << std::endl;
    std::cout << "T_{a,c} = " << Tac << std::endl;
    std::cout << "T_{c,a} = " << Tca << std::endl;

    // Prompt user to enter a vector in the {b} frame
    turtlelib::Vector2D v_b;
    std::cout << "Enter vector v_b:" << std::endl;
    std::cin >> v_b;

    // Find the unit vector v_bhat and the vector in frames {a} and {c}
    auto v_bhat = v_b.normalize();
    auto v_a = Tab(v_b);
    auto v_c = Tcb(v_b);

    // Show all the vectors
    std::cout << "v_bhat: " << v_bhat << std::endl;
    std::cout << "v_a: " << v_a << std::endl;
    std::cout << "v_b: " << v_b << std::endl;
    std::cout << "v_c: " << v_c << std::endl;

    // Prompt user to enter a twist in the {b} frame
    turtlelib::Twist2D V_b;
    std::cout << "Enter twist V_b:" << std::endl;
    std::cin >> V_b;

    // Find the twist represented in the {a} and {c} frames
    auto V_a = Tab.map_twist(V_b);
    auto V_c = Tac.map_twist(V_a);

    // Show all the twists
    std::cout << "V_a: " << V_a << std::endl;
    std::cout << "V_b: " << V_b << std::endl;
    std::cout << "V_c: " << V_c << std::endl;


}

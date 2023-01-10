#include "turtlelib/rigid2d.hpp"
#include <iostream>
#include <cmath>

int main(int argc, char * argv[]) {


    static_assert(turtlelib::almost_equal(0, 0), "is_zero failed");

    static_assert(turtlelib::almost_equal(turtlelib::deg2rad(0.0), 0.0), "deg2rad failed");

    static_assert(turtlelib::almost_equal(turtlelib::rad2deg(0.0), 0.0), "rad2deg) failed");

    static_assert(
            turtlelib::almost_equal(
                turtlelib::deg2rad(turtlelib::rad2deg(2.1)),
                2.1
            ),
            "deg2rad failed"
    );

    std::cout <<
        "1.57rad = " <<
        ceil(turtlelib::rad2deg(1.57)) <<
        "deg" <<
    std::endl;

    return 0;
}

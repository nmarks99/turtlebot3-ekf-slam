#include "turtlelib/rigid2d.hpp"
#include <iostream>

std::ostream &turtlelib::operator<<(std::ostream &os, const turtlelib::Vector2D &v)
{
    os << "[" << v.x << " " << v.y << "]";
    return os;
}

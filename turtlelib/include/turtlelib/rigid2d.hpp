#ifndef RIGID2D_INCLUDE_GUARD_HPP
#define RIGID2D_INCLUDE_GUARD_HPP
/// \file
/// \brief Two-dimensional rigid body transformations.

#include <iosfwd>  // contains forward definitions for iostream objects
#include <cstdlib> // C standard library
#include <vector>
#include <cmath>
#include <iostream>
#include <cassert>

namespace turtlelib
{

    /// \brief PI.  Not in C++ standard until C++20.
    constexpr double PI = 3.14159265358979323846;

    /// \brief approximately compare two floating-point numbers using
    ///        an absolute comparison
    /// \param d1 - a number to compare
    /// \param d2 - a second number to compare
    /// \param epsilon - absolute threshold required for equality
    /// \return true if abs(d1 - d2) < epsilon
    /// NOTE: implement this in the header file
    /// constexpr means that the function can be computed at compile time
    /// if given a compile-time constant as input
    constexpr bool almost_equal(double d1, double d2, double epsilon = 1.0e-12)
    {
        if (fabs(d1 - d2) < epsilon)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    /// \brief Normalizes angle to be in the interval (-pi,pi]
    /// \param rad - an angle in radians
    /// \return angle (radians) in (-pi,pi]
    constexpr double normalize_angle(double rad)
    {
        if (turtlelib::almost_equal(-M_PI, rad))
        {
            return M_PI;
        }
        return atan2(sin(rad), cos(rad));
    }

    /// \brief convert degrees to radians
    /// \param deg - angle in degrees
    /// \returns radians
    constexpr double deg2rad(double deg)
    {
        double rad = deg * (PI / 180);
        return rad;
    }

    /// \brief convert radians to degrees
    /// \param rad - angle in radians
    /// \returns the angle in degrees
    constexpr double rad2deg(double rad)
    {
        double deg = rad * (180 / PI);
        return deg;
    }

    /// static_assertions test compile time assumptions.
    /// You should write at least one more test for each function
    /// You should also purposely (and temporarily) make one of these tests fail
    /// just to see what happens
    static_assert(almost_equal(0, 0), "is_zero failed");

    static_assert(almost_equal(deg2rad(0.0), 0.0), "deg2rad failed");

    static_assert(almost_equal(rad2deg(0.0), 0.0), "rad2deg) failed");

    static_assert(almost_equal(deg2rad(rad2deg(2.1)), 2.1), "deg2rad failed");

    /// \brief A 2-Dimensional Vector
    struct Vector2D
    {
        /// \brief the x coordinate
        double x = 0.0;

        /// \brief the y coordinate
        double y = 0.0;

        /// \brief computes the L2 norm of the 2D vector
        /// \return the normalized vector
        Vector2D normalize() const;

        /// \brief computes the dot product of the vector and another vector
        /// \param rhs - the right hand operand
        /// \return the dot product of the two vectors
        double dot(const Vector2D &rhs) const;

        /// \brief computes the magnitude of the vector and returns the result
        /// \return the magnitude of the vector
        double magnitude() const;

        /// \brief computes the angle between the vector and another vector
        /// \param rhs - the right hand operand
        /// \return the angle between the two vectors
        double angle(const Vector2D &rhs) const;

        /// \brief add this vector with another and store the result
        /// in this object
        /// \param rhs - the vector to add to
        /// \return a reference to the vector after addition
        Vector2D &operator+=(const Vector2D &rhs);

        /// \brief subtract this vector with another and store the result
        /// in this object
        /// \param rhs - the vector to subtract
        /// \return a reference to the vector after subtraction
        Vector2D &operator-=(const Vector2D &rhs);

        /// \brief multiply this vector by a scalar and store the result
        /// in this object
        /// \param scalar - the scalar to multiply by
        /// \return a reference to the vector after multiplication
        Vector2D &operator*=(double scalar);
    };

    /// \brief add two vectors together, returning their sum
    /// \param lhs - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the sum of the two transforms
    Vector2D operator+(Vector2D lhs, const Vector2D &rhs);

    /// \brief subtract two vectors, returning their difference
    /// \param lhs - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the difference of the two transforms
    Vector2D operator-(Vector2D lhs, const Vector2D &rhs);

    /// \brief  multiply a vector and a scalar, returning the result
    /// \param v - the vector
    /// \param scalar - the scalar
    /// \return product of the vector and scalar
    Vector2D operator*(Vector2D v, double scalar);

    // /// \brief multiply a vector by a scalar, returning the new vector
    // /// \param v - the vector
    // /// \param scalar - the scalar
    // /// \return the difference of the two transforms
    // Vector2D operator*(Vector2D v, double scalar);

    /// \brief output a 2 dimensional vector as [xcomponent ycomponent]
    /// os - stream to output to
    /// v - the vector to print
    std::ostream &operator<<(std::ostream &os, const Vector2D &v);

    /// \brief input a 2 dimensional vector
    ///   You should be able to read vectors entered as follows:
    ///   [x y] or x y
    /// \param is - stream from which to read
    /// \param v [out] - output vector
    std::istream &operator>>(std::istream &is, Vector2D &v);

    /// \brief A 2-Dimensional Twist
    struct Twist2D
    {
        /// \brief angular velocity
        double thetadot = 0.0;

        /// \brief x velocity
        double xdot = 0.0;

        /// \brief y velocity
        double ydot = 0.0;
    };

    /// \brief output a 2 dimensional twist as [thetadot xdot ydot]
    /// os - stream to output to
    /// v - the twist to print
    std::ostream &operator<<(std::ostream &os, const Twist2D &v);

    /// \brief input a 2 dimensional twist
    ///   You should be able to read vectors entered as follows:
    ///   [thetadot xdot ydot] or thetadot xdot ydot
    /// \param is - stream from which to read
    /// \param v [out] - output vector
    std::istream &operator>>(std::istream &is, Twist2D &v);

    /// \brief a rigid body transformation in 2 dimensions
    class Transform2D
    {

    private:
        double angle;
        Vector2D p_vec;
        // std::vector<std::vector<double>> tf_vec; // probably not needed

    public:
        /// \brief Create an identity transformation
        Transform2D();

        /// \brief create a transformation that is a pure translation
        /// \param trans - the vector by which to translate
        explicit Transform2D(Vector2D trans);

        /// \brief create a pure rotation
        /// \param radians - angle of the rotation, in radians
        explicit Transform2D(double radians);

        /// \brief Create a transformation with a translational and rotational
        /// component
        /// \param trans - the translation
        /// \param radians - the rotation, in radians
        Transform2D(Vector2D trans, double radians);

        /// \brief apply a transformation to a Vector2D
        /// \param v - the vector to transform
        /// \return a vector in the new coordinate system
        Vector2D operator()(Vector2D v) const;

        /// \brief invert the transformation
        /// \return the inverse transformation.
        Transform2D inv() const;

        /// \brief compose this transform with another and store the result
        /// in this object
        /// \param rhs - the first transform to apply
        /// \return a reference to the newly transformed operator
        Transform2D &operator*=(const Transform2D &rhs);

        /// \brief the translational component of the transform
        /// \return the x,y translation
        Vector2D translation() const;

        /// \brief get the angular displacement of the transform
        /// \return the angular displacement, in radians
        double rotation() const;

        /// \brief use the adjoint to find the representation of
        /// the given twist in another reference frame specified
        /// by the transform described by the Transform2D object (T_ij)
        /// \param V - a twist represented the j frame
        /// \return a twist represented in the i frame
        Twist2D map_twist(const Twist2D &V) const;

        /// \brief computes the transform cooresponding to a rigid body
        /// following a constant twist in its original body frame for
        /// one time unit
        /// \param V - a twist
        /// \return the transform T_{B,B'} after following twist for t=1
        Transform2D integrate_twist(const Twist2D &V) const;

        /// \brief \see operator<<(...) (declared outside this class)
        /// for a description
        friend std::ostream &operator<<(std::ostream &os, const Transform2D &tf);

        /// \brief \see operator>>(...) (declared outside this class)
        /// for a description
        friend std::istream &operator>>(std::istream &is, Transform2D &tf);
    };

    /// \brief should print a human readable version of the transform:
    /// An example output:
    /// deg: 90 x: 3 y: 5
    /// \param os - an output stream
    /// \param tf - the transform to print
    std::ostream &operator<<(std::ostream &os, const Transform2D &tf);

    /// \brief Read a transformation from stdin
    /// Should be able to read input either as output by operator<< or
    /// as 3 numbers (degrees, dx, dy) separated by spaces or newlines
    /// For example:
    /// 90 2 3
    std::istream &operator>>(std::istream &is, Transform2D &tf);

    /// \brief multiply two transforms together, returning their composition
    /// \param lhs - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the composition of the two transforms
    /// HINT: This function should be implemented in terms of *=
    Transform2D operator*(Transform2D lhs, const Transform2D &rhs);

}

#endif

#include "turtlelib/rigid2d.hpp"
#include <iostream>

std::ostream &turtlelib::operator<<(std::ostream &os, const turtlelib::Vector2D &v)
{
    os << "[" << v.x << ", " << v.y << "]";
    return os;
}

// turtlelib::Transform2D matmul(Matrix A) {
    // auto rows_A = A.size();
    // auto cols_A = A.at(0).size();
    // auto rows_B = B.size();
    // auto cols_B = B.at(0).size();
    // assert(cols_A == rows_B);
//
    // Matrix C(
        // rows_A,
        // std::vector<double> (cols_B, 0)
    // );
    // for (auto i = 0; i < rows_A; i++) {
        // for (auto j = 0; j < cols_B; j++) {
            // for (auto k = 0; k < rows_B; k++) {
                // C[i][j] += B[i][k] * A[k][j];
            // }
        // }
    // }
    //
    // turtlelib::Transform2D tf;
    // tf.tf_vec = C;
//
    // return tf;
// }


turtlelib::Transform2D::Transform2D() {

    std::vector<std::vector<double>> _tf
    {
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        {0.0, 0.0, 1.0}
    };
    tf_vec = _tf;
   
    rotation_rad = 0.0;
}


turtlelib::Transform2D::Transform2D(turtlelib::Vector2D trans){
    
    std::vector<std::vector<double>> _tf
    {
        {1.0, 0.0, trans.x},
        {0.0, 1.0, trans.y},
        {0.0, 0.0, 1.0}
    };
    tf_vec = _tf;
    
    translation_vec = trans;

}


turtlelib::Transform2D::Transform2D(double radians) {

    std::vector<std::vector<double>> _tf
    {
        {cos(radians), -sin(radians), 0.0},
        {sin(radians), cos(radians), 0.0},
        {0.0, 0.0, 1.0}
    };

    tf_vec = _tf;

    rotation_rad = radians;
}


turtlelib::Transform2D::Transform2D(Vector2D trans, double radians) {
    
    std::vector<std::vector<double>> _tf
    {
        {cos(radians), -sin(radians), trans.x},
        {sin(radians), cos(radians), trans.y},
        {0.0, 0.0, 1.0}
    };

    tf_vec = _tf;

    rotation_rad = radians;
    translation_vec = trans;

}


turtlelib::Vector2D turtlelib::Transform2D::operator()(turtlelib::Vector2D v) const {
    
    turtlelib::Vector2D res; 

    std::vector<double> v_vec = {v.x, v.y, 1.0};
    std::vector<double> v_res = {0.0, 0.0, 0.0};
    double _row = 0;
    for (auto i = 0; i < 3; i++){
        for (auto j = 0; j < 3; j++){
            _row += v_vec.at(j) * turtlelib::Transform2D::tf_vec.at(i).at(j); 
        }
        v_res[i] = _row;
        _row = 0;
    }
    
    res.x = v_res.at(0);
    res.y = v_res.at(1);
        
    return res;
}



std::ostream &turtlelib::operator<<(std::ostream &os, const turtlelib::Transform2D &tf)
{
    os<<
        "deg:"<<
        rad2deg(tf.rotation_rad)<<
        " x:"<<
        tf.translation_vec.x<<
        " y:"<<
        tf.translation_vec.y;
    return os;
}


turtlelib::Transform2D turtlelib::Transform2D::inv() const {

    auto tf_vec0 = turtlelib::Transform2D::tf_vec;

    // transpose a 2d matrix by swapping elements across main diagonal
    std::vector<std::vector<double>> R_T
    {
        {tf_vec0[0][0], tf_vec0[1][0]},
        {tf_vec0[0][1], tf_vec0[1][1]}
    };
    
    auto p_vec = std::vector<double> {
        turtlelib::Transform2D::translation_vec.x,
        turtlelib::Transform2D::translation_vec.y
    };

    // p_res(2x2) = -R_T * p_vec 
    auto _row = 0;
    auto p_res = std::vector<double> {0.0,0.0};
    for (auto i = 0; i < 2; i++){
        for (auto j = 0; j < 2; j++){
            _row += p_vec.at(j) * -R_T.at(i).at(j); 
        }
        p_res[i] = _row;
        _row = 0;
    }
    
    // Fill in the inverse into tf_vec of a new tf
    turtlelib::Transform2D tf_out;
    tf_out.tf_vec[0][0] = R_T[0][0];
    tf_out.tf_vec[0][1] = R_T[0][1];
    tf_out.tf_vec[1][0] = R_T[1][0];
    tf_out.tf_vec[1][1] = R_T[1][1];
    // std::cout << p_res[0] << ", " << p_res[1] << std::endl;
    tf_out.tf_vec[0][2] = p_res[0];
    tf_out.tf_vec[1][2] = p_res[1];

    return tf_out;
    
}


turtlelib::Vector2D turtlelib::Transform2D::translation() const {
    return translation_vec;
}


double turtlelib::Transform2D::rotation() const {
    return rotation_rad; 
}









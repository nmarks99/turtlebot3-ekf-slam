#include "nuslam/circle_fitting.hpp"
#include <cstddef>
#include <tuple>
#include <turtlelib/rigid2d.hpp>

using turtlelib::almost_equal;
using turtlelib::Vector2D;

// =============
//    Cluster
// =============

/// @cond
Cluster::Cluster() {}


Cluster::Cluster(double threshold) : THRESHOLD(threshold) {}


Cluster::Cluster(Vector2D p_new, double threshold) : THRESHOLD(threshold)
{
    cluster_vec.push_back(p_new);
}


Cluster::Cluster(Vector2D p_new)
{
    cluster_vec.push_back(p_new);
}

bool Cluster::contains(Vector2D p_new)
{
    for (auto &p : cluster_vec)
    {
        if (almost_equal(p.x, p_new.x) and almost_equal(p.y, p_new.y))
        {
            return true;
        }
    }
    return false;
}


bool Cluster::belongs(Vector2D p_new)
{
    for (auto &p : cluster_vec)
    {
        if (not contains(p_new))
        {
            // if the new point is within the threshold
            // distance of any of the other points, add it
            // and return true
            const double d = turtlelib::distance(p, p_new);
            if (d > 0.0 and d < THRESHOLD)
            {
                cluster_vec.push_back(p_new);
                return true;
            }
        }
    }
    return false;
}

bool Cluster::blind_add(Vector2D p_new)
{
    if (contains(p_new))
    {
        return false;
    }
    else
    {
        cluster_vec.push_back(p_new);
        return true;
    }

}


Vector2D Cluster::centroid() const
{
    double sum_x = 0.0;
    double sum_y = 0.0;
    auto n = cluster_vec.size();
    for (auto &c : cluster_vec)
    {
        sum_x += c.x;
        sum_y += c.y;
    }
    
    return Vector2D{sum_x/n, sum_y/n};

}


std::vector<Vector2D> Cluster::get_vector() const
{
    return cluster_vec;
}

size_t Cluster::count() const
{
    return cluster_vec.size();
}

// ===================
//    Circle Fitting
// ===================

double vector_mean(const std::vector<double> &v)
{
    return std::reduce(v.begin(),v.end())/static_cast<double>(v.size());
}

namespace
{

    // Shifts the ith point p to the centroid
    // and computes zi = x^2 + y^2
    double compute_zi(const Vector2D &p, const Vector2D &centroid)
    {
        const double xi = p.x - centroid.x;
        const double yi = p.y - centroid.y;
        return std::pow(xi,2.0) + std::pow(yi,2.0);
    }

    arma::mat compute_Z(const Cluster &cluster, const std::vector<double> &z_vec, const Vector2D &centroid)
    {
        const size_t n = cluster.count();
        arma::mat Z = arma::mat(n,4, arma::fill::zeros);
        Z.submat(0,3,n-1,3) = arma::mat(n,1,arma::fill::ones);

        for (size_t i = 0; i < Z.n_rows; i++)
        {
            Z(i,0) = z_vec.at(i);
            Z(i,1) = cluster.get_vector().at(i).x - centroid.x;
            Z(i,2) = cluster.get_vector().at(i).y - centroid.y;
        }
        return Z;
    }

    arma::mat compute_M(arma::mat Z)
    {
        return (1.0/Z.n_rows) * Z.t() * Z;
    }

    arma::mat compute_H(double z_bar)
    {
        arma::mat H = arma::mat(4,4,arma::fill::eye);
        H(0,0) = 8.0 * z_bar;
        H(3,3) = 0.0;
        H(0,3) = 2.0;
        H(3,0) = 2.0;

        return H;
    }

    arma::mat compute_Hinv(double z_bar)
    {
        arma::mat Hinv = arma::mat(4,4,arma::fill::eye);
        Hinv(0,0) = 0.0;
        Hinv(3,3) = -2.0 * z_bar;
        Hinv(0,3) = 0.5;
        Hinv(3,0) = 0.5;

        return Hinv;
    }
}


std::tuple<Vector2D, double> fit_circle(Cluster cluster)
{
    // Compute the centroid of the cluster
    Vector2D centroid = cluster.centroid();

    // Compute the z for each point, and mean z_bar
    std::vector<double> z_vec;
    for (auto &p : cluster.get_vector())
    {
        z_vec.push_back(compute_zi(p,centroid));
    }
    double z_bar = vector_mean(z_vec);

    // Form the data matrix Z
    arma::mat Z = compute_Z(cluster,z_vec,centroid);
    
    // Form the moment matrix M
    arma::mat M = compute_M(Z);

    // Form the constraint matrix H and its inverse
    arma::mat H = compute_H(z_bar);
    arma::mat Hinv = compute_Hinv(z_bar);

    // Compute the SVD of Z
    arma::mat U;
    arma::vec sigma;
    arma::mat V;
    arma::svd(U,sigma,V,Z);
    
    arma::mat A = arma::mat(V.n_rows,1);
    if (sigma(sigma.n_rows-1,0) < 10e-12)
    {
        A = V.col(3);
    }
    else
    {
        arma::mat Y = V * arma::diagmat(sigma) * V.t();
        arma::mat Q = Y * Hinv * Y;
        
        // Find eigenvalues and eigenvectors of Q
        arma::vec eigval;
        arma::mat eigvec;
        arma::eig_sym(eigval, eigvec, Q); 

        // Find the index of the smallest positive eigenvalue
        arma::uvec pos_inds = arma::find(eigval > 0.0);
        const int ind = static_cast<int>(pos_inds(0)); // eigval ordered least..greatest

        // Get the eigenvector cooresponding to the smallest positive eignevalue
        arma::vec A_star = eigvec.col(ind);

        // Solve YA = A* for A
        A = arma::solve(Y,A_star);

    }
    
    // Compute and return the center and radius of the circle
    const double a = -A(1)/(2.0*A(0)) + centroid.x;
    const double b = -A(2)/(2.0*A(0)) + centroid.y;
    const double R = std::sqrt(
            ( std::pow(A(1),2.0) + std::pow(A(2),2.0) - (4*A(0)*A(3)) ) /
            (4*std::pow(A(0),2.0))
            );

    return std::tuple<Vector2D,double>{Vector2D{a,b}, R};
}

/// @endcond

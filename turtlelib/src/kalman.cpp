#include "turtlelib/kalman.hpp"

namespace turtlelib
{

    std::mt19937 &get_random()
    {
        // Credit Matt Elwin: https://nu-msr.github.io/navigation_site/lectures/gaussian.html

        // static variables inside a function are created once and persist for the remainder of the program
        static std::random_device rd{};
        static std::mt19937 mt{rd()};
        // we return a reference to the pseudo-random number genrator object. This is always the
        // same object every time get_random is called
        return mt;
    }

    KalmanFilter::KalmanFilter()
        : qt_hat{0.0, 0.0, 0.0},
          sigma_hat(3, 3, arma::fill::zeros),
          Q_mat(3, 3, arma::fill::zeros) {}

    void KalmanFilter::predict(const Twist2D &V)
    {
        // Note for the prediction step here, we set the noise
        // equal to zero and the map stays stationary

        arma::mat A_t(3, 3, arma::fill::zeros);
        Pose2D qt_hat_new;

        // zero rotational velocity
        if (almost_equal(V.thetadot, 0.0))
        {
            qt_hat_new.theta = 0.0;
            qt_hat_new.x = qt_hat.x + V.xdot * std::cos(qt_hat.theta);
            qt_hat_new.y = qt_hat.y + V.xdot * std::sin(qt_hat.theta);

            A_t(1, 0) = -V.xdot * std::sin(qt_hat.theta);
            A_t(2, 0) = V.xdot * std::cos(qt_hat.theta);
        }
        else // non-zero rotational velocity
        {
            qt_hat_new.theta = normalize_angle(qt_hat.theta + V.thetadot);
            qt_hat_new.x = qt_hat.x -
                           (V.xdot / V.thetadot) * std::sin(qt_hat.theta) +
                           (V.xdot / V.thetadot) * std::sin(qt_hat.theta + V.thetadot);
            qt_hat_new.y = qt_hat.y +
                           (V.xdot / V.thetadot) * std::cos(qt_hat.theta) -
                           (V.xdot / V.thetadot) * std::cos(qt_hat.theta + V.thetadot);

            A_t(1, 0) = -(V.xdot / V.thetadot) * std::cos(qt_hat.theta) +
                        (V.xdot / V.thetadot) * std::cos(qt_hat.theta + V.thetadot);
            A_t(2, 0) = -(V.xdot / V.thetadot) * std::sin(qt_hat.theta) +
                        (V.xdot / V.thetadot) * std::sin(qt_hat.theta + V.thetadot);
        }
        A_t = arma::eye(arma::size(A_t)) + A_t;

        // Save the new prediction of the robot's configuration
        qt_hat = qt_hat_new;

        // Now we propagate the uncertainty using the linear state transition model
        sigma_hat = A_t * sigma_hat * A_t.t() + Q_mat;
    }
}
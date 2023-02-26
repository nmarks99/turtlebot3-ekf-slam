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

    LandmarkMeasurement::LandmarkMeasurement()
        : r(0.0), phi(0.0), marker_id(0) {}

    LandmarkMeasurement::LandmarkMeasurement(double _r, double _phi, int _marker_id)
        : r(_r), phi(_phi), marker_id(_marker_id) {}

    LandmarkMeasurement LandmarkMeasurement::from_cartesian(double _x, double _y, int _marker_id)
    {
        LandmarkMeasurement measurement;
        measurement.r = std::sqrt(std::pow(_x, 2.0) + std::pow(_y, 2.0));
        measurement.phi = normalize_angle(std::atan2(_y, _x));
        measurement.marker_id = _marker_id;
        return measurement;
    }

    arma::mat LandmarkMeasurement::to_mat() const
    {
        arma::mat z(2, 1, arma::fill::zeros);
        z(0, 0) = r;
        z(1, 0) = normalize_angle(phi);
        return z;
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
        {    // may need to do normalize_angle()
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
        sigma_hat = (A_t * sigma_hat * A_t.t()) + Q_mat;
    }

    void KalmanFilter::new_measurement(const LandmarkMeasurement &measurement)
    {

        // Landmark must be initialized if it hasn't been seen
        auto mx_j = 0.0;
        auto my_j = 0.0;
        auto mt_j = arma::mat(2, 1, arma::fill::zeros);
        if (not landmarks.count(measurement.marker_id))
        {
            // add new landmark
            mx_j = qt_hat.x +
                   measurement.r * std::cos(normalize_angle(measurement.phi + qt_hat.theta));
            my_j = qt_hat.y +
                   measurement.r * std::sin(normalize_angle(measurement.phi + qt_hat.theta));
            mt_j = arma::mat{mx_j, my_j}.t();

            landmarks[measurement.marker_id] = mt_j;

            // Update dimensions of the covariance matrix Sigma

            // Update the dimensions of the process noise matrix Q
        }

        // Update the complete state estimate
        Xi = arma::join_cols(Xi, mt_j);
        std::cout << "Xi = " << Xi << std::endl;
    }

    void KalmanFilter::update()
    {
        // First you must associate incoming measurements with a landmark
        // 1. Check if measurement i corresponds to a landmark we already have
        // 2. If a measurement i's marker_id is not present in our current list of
        // known landmarks, call initialize_landmark(measurement_i)

        // For each measurement i
        // 1. Compute theoretical measurement z_t_hat = h_j
        // 2. Compute the Kalman gain (Eq 26)
        // 3. Compute the posterior state update Xi_t_hat
        // 4. Compute the posterior covariance sigma_t
    }

    Pose2D KalmanFilter::pose_prediction() const
    {
        return qt_hat;
    }
}
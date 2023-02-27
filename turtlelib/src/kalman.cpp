#include "turtlelib/kalman.hpp"

namespace turtlelib
{

    // std::mt19937 &get_random()
    // {
    //     // Credit Matt Elwin: https://nu-msr.github.io/navigation_site/lectures/gaussian.html

    //     // static variables inside a function are created once and persist for the remainder of the program
    //     static std::random_device rd{};
    //     static std::mt19937 mt{rd()};
    //     // we return a reference to the pseudo-random number genrator object. This is always the
    //     // same object every time get_random is called
    //     return mt;
    // }

    static constexpr double BIG_NUMBER = 1e6;

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
        : qt_hat(arma::mat(3, 1, arma::fill::zeros)),
          Xi_hat(qt_hat), // mt appended to this as new measurements are added
          sigma_hat(arma::mat(5, 5, arma::fill::zeros)),
          Q_bar(arma::mat(5, 5, arma::fill::zeros))
    //   R_bar(arma::mat(2, 2, arma::fill::zeros)) // fix dimension
    {
        sigma_hat(3, 3) = BIG_NUMBER;
        sigma_hat(4, 4) = BIG_NUMBER;
        // std::cout << sigma_hat << std::endl;
        // std::cout << arma::size(sigma_hat) << std::endl;
    }

    KalmanFilter::KalmanFilter(double Q, double R)
        : qt_hat(arma::mat(3, 1, arma::fill::zeros)),
          Xi_hat(qt_hat), // mt appended to this as new measurements are added
          sigma_hat(arma::mat(5, 5, arma::fill::zeros)),
          Q_bar(arma::mat(5, 5, arma::fill::zeros))
    //   R_bar(R * arma::mat(2, 2, arma::fill::eye))
    {
        sigma_hat(3, 3) = 1e6;
        sigma_hat(4, 4) = 1e6;
        Q_bar.submat(0, 0, 2, 2) = Q * arma::mat(3, 3, arma::fill::eye);
    }

    void KalmanFilter::update_measurements(const LandmarkMeasurement &measurement)
    {

        auto mx_j = 0.0;
        auto my_j = 0.0;
        arma::mat mt_j(2, 1, arma::fill::zeros);

        // Landmark must be initialized if it hasn't been seen
        if (not landmarks_dict.count(measurement.marker_id))
        {
            // add new landmark, converting to (x,y) from (r,phi)
            mx_j = Xi_hat(1, 0) +
                   measurement.r * std::cos(normalize_angle(measurement.phi + Xi_hat(0, 0)));
            my_j = Xi_hat(2, 0) +
                   measurement.r * std::sin(normalize_angle(measurement.phi + Xi_hat(0, 0)));
            mt_j = arma::mat{mx_j, my_j}.t();

            // Update the complete state estimate by adding in the new mt_j vector
            Xi_hat = arma::join_cols(Xi_hat, mt_j);

            // store the index of the x_j component for this landmark
            landmarks_dict[measurement.marker_id] = Xi_hat.n_rows - 2;

            n = (Xi_hat.n_rows - 3) / 2; // number of landmarks

            // Update dimensions of the covariance matrix Sigma (3+2n x 3+2n)
            // sigma_hat is initialed to the correct size already for n=1

            if (n != 1)
            {
                // n has increased by 1 since the last time this function was called
                // Sigma_hat goes from 3+2n x 3+2n to 3+2(n+1) x 3+2(n+1)
                sigma_hat = arma::join_cols(sigma_hat, arma::mat(2, 3 + 2 * (n - 1), arma::fill::zeros));
                sigma_hat = arma::join_rows(sigma_hat, arma::mat(3 + 2 * n, 2, arma::fill::zeros));

                // not sure if we need to do this each time a measurment is added...
                sigma_hat(sigma_hat.n_rows - 1, sigma_hat.n_cols - 1) = BIG_NUMBER;
                sigma_hat(sigma_hat.n_rows - 2, sigma_hat.n_cols - 2) = BIG_NUMBER;

                // Update the dimensions of the process noise matrix Q_bar
                Q_bar = arma::join_cols(Q_bar, arma::mat(2, 3 + 2 * (n - 1), arma::fill::zeros));
                Q_bar = arma::join_rows(Q_bar, arma::mat(3 + 2 * n, 2, arma::fill::zeros));

                // Verify dimensions are correct
                assert(sigma_hat.n_rows == sigma_hat.n_cols);
                assert(sigma_hat.n_rows == (3 + 2 * n));
                assert(Q_bar.n_rows == Q_bar.n_cols);
                assert(Q_bar.n_rows == (3 + 2 * n));
            }

        } // else, Xi_hat gets updated in the EKF update step, and it's dimenions are already correct
    }

    void KalmanFilter::predict(const Twist2D &V)
    {
        // This must only be called once update_measurements() has been called

        // Note for the prediction step here, we set the noise
        // equal to zero and the map stays stationary

        arma::mat A_t(3, 3, arma::fill::zeros);
        arma::mat qt_hat_new(3, 1, arma::fill::zeros);

        // zero rotational velocity
        if (almost_equal(V.thetadot, 0.0))
        {
            qt_hat_new(0, 0) = 0.0;
            qt_hat_new(1, 0) = Xi_hat(1, 0) + V.xdot * std::cos(Xi_hat(0, 0));
            qt_hat_new(2, 0) = Xi_hat(2, 0) + V.xdot * std::sin(Xi_hat(0, 0));

            A_t(1, 0) = -V.xdot * std::sin(Xi_hat(0, 0));
            A_t(2, 0) = V.xdot * std::cos(Xi_hat(0, 0));
        }
        else // non-zero rotational velocity
        {    // may need to do normalize_angle()
            qt_hat_new(0, 0) = normalize_angle(Xi_hat(0, 0) + V.thetadot);
            qt_hat_new(1, 0) = Xi_hat(1, 0) -
                               (V.xdot / V.thetadot) * std::sin(Xi_hat(0, 0)) +
                               (V.xdot / V.thetadot) * std::sin(Xi_hat(0, 0) + V.thetadot);
            qt_hat_new(2, 0) = Xi_hat(2, 0) +
                               (V.xdot / V.thetadot) * std::cos(Xi_hat(0, 0)) -
                               (V.xdot / V.thetadot) * std::cos(Xi_hat(0, 0) + V.thetadot);

            A_t(1, 0) = -(V.xdot / V.thetadot) * std::cos(Xi_hat(0, 0)) +
                        (V.xdot / V.thetadot) * std::cos(Xi_hat(0, 0) + V.thetadot);
            A_t(2, 0) = -(V.xdot / V.thetadot) * std::sin(Xi_hat(0, 0)) +
                        (V.xdot / V.thetadot) * std::sin(Xi_hat(0, 0) + V.thetadot);
        }

        // A_t should be (3+2n x 3+2n)
        A_t = arma::join_rows(A_t, arma::mat(3, 2 * n, arma::fill::zeros));
        A_t = arma::join_cols(A_t, arma::mat(2 * n, 3 + 2 * n, arma::fill::zeros));
        A_t = A_t + arma::mat(arma::size(A_t), arma::fill::eye);
        assert(A_t.n_rows == A_t.n_cols);
        assert(A_t.n_rows == (3 + 2 * n));

        // Save the new prediction of the robot's configuration
        qt_hat = qt_hat_new;

        // Now we propagate the uncertainty using the linear state transition model
        sigma_hat = (A_t * sigma_hat * A_t.t()) + Q_bar;
    }

    arma::mat KalmanFilter::compute_h(int j) const
    {
        const int ind_in_Xi = j + (n + 1); // mt_j vector is at (j+3, j+4)
        arma::mat mj = {Xi_hat(ind_in_Xi, 0), Xi_hat(ind_in_Xi + 1, 0)};
        mj = mj.t();
        const auto r_j = std::sqrt(
            std::pow((mj(0, 0) - Xi_hat(1, 0)), 2.0) +
            std::pow((mj(1, 0) - Xi_hat(2, 0)), 2.0));
        const auto phi_j = normalize_angle(
            std::atan2(mj(1, 0) - Xi_hat(2, 0), mj(0, 0) - Xi_hat(1, 0)) - Xi_hat(0, 0));

        return arma::mat{r_j, phi_j}; // may need to transpose
    }

    arma::mat KalmanFilter::compute_H(int j) const
    {
        const int ind_in_Xi = j + (n + 1); // mt_j vector is at (j+3, j+4)
        arma::mat mj = {Xi_hat(ind_in_Xi, 0), Xi_hat(ind_in_Xi + 1, 0)};
        mj = mj.t();
        const auto del_x = (mj(0, 0) - Xi_hat(1, 0));
        const auto del_y = (mj(1, 0) - Xi_hat(2, 0));
        const auto d = std::pow(del_x, 2.0) + std::pow(del_y, 2.0);

        // number of obstancles n is equal to number of rows, minus 3 (for qt = [theta x y]) / 2
        const int n = (Xi_hat.n_rows - 3) / 2;

        arma::mat H1(2, 3, arma::fill::zeros);
        H1(1, 0) = -1.0;
        H1(0, 1) = -del_x / std::sqrt(d);
        H1(0, 2) = -del_y / std::sqrt(d);
        H1(1, 1) = del_y / d;
        H1(1, 2) = -del_x / d;

        arma::mat H2(2, (2 * (j - 1)), arma::fill::zeros);

        arma::mat H3(2, 2, arma::fill::zeros);
        H3(0, 0) = del_x / std::sqrt(d);
        H3(0, 1) = del_y / std::sqrt(d);
        H3(1, 0) = -del_y / d;
        H3(1, 1) = del_x / d;

        arma::mat H4(2, ((2 * n) - (2 * j)), arma::fill::zeros);

        return arma::join_rows(H1, H2, H3, H4);
    }

    void KalmanFilter::update(const std::vector<LandmarkMeasurement> &measurements)
    {
        // for each measurement
        for (size_t i = 0; i < measurements.size(); i++)
        {
            // First you must associate incoming measurements with a landmark
            // update_measurements(measurements.at(i));

            // 1. Compute theoretical measurement z_t_hat = h_j
            const unsigned int ind_in_Xi = landmarks_dict[measurements.at(i).marker_id];
            const unsigned int j = ind_in_Xi - (n + 1);
            const arma::mat zi_hat = compute_h(j); // index is location in Xi, we want j for compute_h

            // 2. Compute the Kalman gain (Eq 26)
            const arma::mat R(2, 2, arma::fill::zeros);
            const arma::mat H = compute_H(j);
            assert(H.n_rows == 2);
            assert(H.n_cols == 3 + (2 * n));
            const arma::mat K = (sigma_hat * H.t()) * arma::inv((H * sigma_hat * H.t()) + R);

            // 3. Compute the posterior state update Xi_t_hat
            const arma::mat zi{Xi_hat(ind_in_Xi, 0), Xi_hat(ind_in_Xi + 1, 0)};
            Xi_hat = Xi_hat + K * (zi.t() - zi_hat.t());

            // 4. Compute the posterior covariance sigma_t
            const arma::mat I = arma::mat(arma::size(sigma_hat), arma::fill::eye);
            sigma_hat = (I - K * H) * sigma_hat;
            std::cout << sigma_hat << std::endl;
        }
    }

    void KalmanFilter::run(const Twist2D &V, const std::vector<LandmarkMeasurement> &measurements)
    {

        // Add new measurments and update dimensions if needed
        for (size_t i = 0; i < measurements.size(); i++)
        {
            update_measurements(measurements.at(i));
        }

        /// Kalman filter prediction step
        predict(V);

        // Kalman filter update step
        update(measurements);
    }

    arma::mat KalmanFilter::pose_prediction() const
    {
        return qt_hat;
    }

    arma::mat KalmanFilter::map_prediction() const
    {
        return Xi_hat.submat(3, 0, Xi_hat.n_rows, 0);
    }

    arma::mat KalmanFilter::state_prediction() const
    {
        return Xi_hat;
    }

}
#include "turtlelib/kalman.hpp"
#include "turtlelib/rigid2d.hpp"

static constexpr double BIG_NUMBER = 1e5;

namespace turtlelib
{

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
        : Xi_hat(arma::mat(3, 1, arma::fill::zeros)), // mt appended to this as new measurements are added
          sigma_hat(arma::mat(5, 5, arma::fill::zeros)),
          Q_bar(arma::mat(5, 5, arma::fill::zeros)),
          R_bar(arma::mat(2, 2, arma::fill::zeros)) // fix dimension
    {
        sigma_hat(3, 3) = BIG_NUMBER;
        sigma_hat(4, 4) = BIG_NUMBER;
    }

    KalmanFilter::KalmanFilter(double Q, double R)
        : Xi_hat(arma::mat(3, 1, arma::fill::zeros)), // mt appended to this as new measurements are added
          sigma_hat(arma::mat(5, 5, arma::fill::zeros)),
          Q_bar(arma::mat(5, 5, arma::fill::zeros)),
          R_bar(R * arma::mat(2, 2, arma::fill::eye))
    {
        sigma_hat(3, 3) = BIG_NUMBER;
        sigma_hat(4, 4) = BIG_NUMBER;
        Q_bar.submat(0, 0, 2, 2) = Q * arma::mat(3, 3, arma::fill::eye);
    }

    void KalmanFilter::update_measurements(const LandmarkMeasurement &measurement)
    {
        // double mx_j = 0.0;
        // double my_j = 0.0;
        arma::mat mt_j(2, 1, arma::fill::zeros);

        // Landmark must be initialized if it hasn't been seen
        if (not landmarks_dict.count(measurement.marker_id))
        {
            // add new landmark, converting to (x,y) from (r,phi)
            double mx_j = Xi_hat(1, 0) +
                   measurement.r * std::cos(normalize_angle(measurement.phi + Xi_hat(0, 0)));
            double my_j = Xi_hat(2, 0) +
                   measurement.r * std::sin(normalize_angle(measurement.phi + Xi_hat(0, 0)));

            mt_j = arma::mat{mx_j, my_j}.t();

            // Update the complete state estimate by adding in the new mt_j vector
            Xi_hat = arma::join_cols(Xi_hat, mt_j);

            // store the index of the x_j component for this landmark
            landmarks_dict[measurement.marker_id] = Xi_hat.n_rows - 2;

            n = (Xi_hat.n_rows - 3) / 2; // number of landmarks

            // Update dimensions of the covariance matrix Sigma (3+2n x 3+2n)
            // sigma_hat is initialed to the correct size already for n=1
            if (n > 1)
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
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("KalmanFilter"), "Landmarks updated");

        } // else, Xi_hat gets updated in the EKF update step, and it's dimenions are already correct
        else
        {
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("KalmanFilter"), "No new landmarks added");
        }
    }

    void KalmanFilter::predict_from_odometry(const Pose2D &pose, const Twist2D &V)
    {
        // This must only be called once update_measurements() has been called

        // Note for the prediction step here, we set the noise
        // equal to zero and the map stays stationary
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("KalmanFilter"), "-----------Beginning prediction step----------");
        arma::mat A_t(3, 3, arma::fill::zeros);
        arma::mat qt_hat_new(3, 1, arma::fill::zeros);
        arma::mat w_t(3, 1, arma::fill::zeros); // w_t = 0

        Xi_hat(0, 0) = normalize_angle(Xi_hat(0, 0));

        qt_hat_new(0, 0) = pose.theta;
        qt_hat_new(1, 0) = pose.x;
        qt_hat_new(2, 0) = pose.y;

        // zero rotational velocity
        if (almost_equal(V.thetadot, 0.0))
        {
            // A_t = derivative of g
            A_t(0, 0) = 0.0;
            A_t(1, 0) = -V.xdot * std::sin(Xi_hat(0, 0));
            A_t(2, 0) = V.xdot * std::cos(Xi_hat(0, 0));
        }
        else // non-zero rotational velocity
        {
            // A_t = derivative of g
            A_t(0, 0) = 0.0;
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
        Xi_hat.submat(0, 0, 2, 0) = qt_hat_new;

        // Now we propagate the uncertainty using the linear state transition model
        sigma_hat = (A_t * sigma_hat * A_t.t()) + Q_bar;

        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("KalmanFilter"), "sigma_hat size = " << arma::size(sigma_hat));
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("KalmanFilter"), "Xi_hat = \n"
                                                                   << Xi_hat);
        // RCLCPP_DEBUG_STREAM(rclcpp::get_logger("KalmanFilter"), "sigma_hat = \n"
        //                                                            << sigma_hat);
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("KalmanFilter"), "-----------Finished prediciton step-----------");
    }

    arma::mat KalmanFilter::compute_h(unsigned int ind_in_Xi) const
    {

        const double mj_x = Xi_hat(ind_in_Xi, 0);
        const double mj_y = Xi_hat(ind_in_Xi + 1, 0);

        const double r_j = std::sqrt(
            std::pow((mj_x - Xi_hat(1, 0)), 2.0) +
            std::pow((mj_y - Xi_hat(2, 0)), 2.0));

        const double phi_j = normalize_angle(
                std::atan2(mj_y - Xi_hat(2, 0), mj_x - Xi_hat(1, 0)) - Xi_hat(0, 0));

        arma::mat h = arma::mat{r_j, phi_j};

        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("KalmanFilter"), "h = \n"
                                                                   << h);
        return h;

    }

    arma::mat KalmanFilter::compute_H(unsigned int ind_in_Xi) const
    {
        const unsigned int j = (ind_in_Xi - 1) / 2;
        const double del_x = (Xi_hat(ind_in_Xi, 0) - Xi_hat(1, 0));
        const double del_y = (Xi_hat(ind_in_Xi + 1, 0) - Xi_hat(2, 0));
        const double d = std::pow(del_x, 2.0) + std::pow(del_y, 2.0);

        arma::mat H = arma::mat(2, sigma_hat.n_cols, arma::fill::zeros);
        H(0, 1) = -del_x / std::sqrt(d);
        H(0, 2) = -del_y / std::sqrt(d);
        H(1, 0) = -1.0;
        H(1, 1) = del_y / d;
        H(1, 2) = -del_x / d;

        H(0, 3 + j) = del_x / std::sqrt(d);
        H(0, 4 + j) = del_y / std::sqrt(d);
        H(1, 3 + j) = -del_y / d;
        H(1, 4 + j) = del_x / d;

        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("KalmanFilter"), "H = \n"
                                                                   << H);
        return H;
    }

    void KalmanFilter::update(const std::vector<LandmarkMeasurement> &measurements)
    {
        // for each measurement
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("KalmanFilter"), "------------Beginning update step-----------");

        // Note: if the vector has multiple measurments with the same id,
        // this will fail! The duplicate id measurement will not get added
        // measurements.size() will be greater than the number of unique measurements
        for (size_t i = 0; i < measurements.size(); i++)
        {
            // Get the index of this landmark in the state vector
            const unsigned int ind_in_Xi = landmarks_dict[measurements.at(i).marker_id];

            // 1. Compute theoretical measurement zi_hat = h_j
            arma::mat zi_hat = compute_h(ind_in_Xi); // index is location in Xi, we want j for compute_h

            // 2. Compute the Kalman gain (Eq 26)
            arma::mat H = compute_H(ind_in_Xi);
            arma::mat K = (sigma_hat * H.t()) * arma::inv(H * sigma_hat * H.t() + R_bar);

            // 3. Compute the posterior state update Xi_t_hat
            arma::mat zi = measurements.at(i).to_mat();
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("KalmanFilter"), "(zi - zi_hat)\n"
                                                                       << zi << " - " << zi_hat.t());
            arma::mat z_diff = zi - zi_hat.t();
            z_diff(1, 0) = normalize_angle(z_diff(1, 0));

            Xi_hat = Xi_hat + K * z_diff;
            Xi_hat(0, 0) = normalize_angle(Xi_hat(0, 0)); // Normalize the angle

            // 4. Compute the posterior covariance sigma_t
            arma::mat I = arma::mat(arma::size(sigma_hat), arma::fill::eye);
            sigma_hat = (I - K * H) * sigma_hat;
        }

        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("KalmanFilter"), "---------Finished update step---------");
    }

    void KalmanFilter::run(const Pose2D &pose, const Twist2D &V, const std::vector<LandmarkMeasurement> &measurements)
    {
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("KalmanFilter"), "-------------Start run-------------");
        // Add new measurments and update dimensions if needed
        for (size_t i = 0; i < measurements.size(); i++)
        {
            update_measurements(measurements.at(i));
        }

        /// Kalman filter prediction step
        predict_from_odometry(pose, V);

        // Kalman filter update step
        update(measurements);

        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("KalmanFilter"), "State = " << Xi_hat);
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("KalmanFilter"), "-------------Run complete-------------");
    }

    arma::mat KalmanFilter::pose_prediction() const
    {
        arma::mat robot_pose = Xi_hat.submat(0, 0, 2, 0);
        robot_pose(0, 0) = normalize_angle(robot_pose(0, 0));
        return robot_pose;
    }

    arma::mat KalmanFilter::map_prediction() const
    {
        return Xi_hat.submat(3, 0, Xi_hat.n_rows - 1, 0);
    }

    arma::mat KalmanFilter::state_prediction() const
    {
        arma::mat full_state = Xi_hat;
        full_state(0, 0) = normalize_angle(full_state(0, 0));
        return Xi_hat;
    }

}

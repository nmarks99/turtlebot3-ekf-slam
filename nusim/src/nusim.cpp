#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <rclcpp/logging.hpp>
#include "std_msgs/msg/u_int64.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/empty.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#define UNUSED(x) (void)(x) // used to suppress "unused-variable" warnings

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

const double DEFAULT_X0 = 0.0;
const double DEFAULT_Y0 = 0.0;
const double DEFAULT_THETA0 = 0.0;

class Nusim : public rclcpp::Node {

    public:
        Nusim() : Node("nusim"), step(0) {
            
            // Declare parameters
            this->declare_parameter<double>("x0",DEFAULT_X0);
            this->declare_parameter<double>("y0",DEFAULT_Y0);
            this->declare_parameter<double>("theta0",DEFAULT_THETA0);

            // Create timestep publisher
            timestep_pub = this->create_publisher<std_msgs::msg::UInt64>("~/timestep",10);

            // Create reset service 
            reset_service = this->create_service<std_srvs::srv::Empty>(
                "~/reset",
                std::bind(&Nusim::reset_callback, this, _1, _2)
            );

            // Create transform broadcaster
            tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

            // Create timer
            _timer = this->create_wall_timer(
                5ms,
                std::bind(&Nusim::timer_callback, this)
            );

            // Ground truth pose of the robot known only to the simulator
            // Initial values are passed as parameters to the node
            true_pose.x = this->get_parameter("x0").get_value<double>();
            true_pose.y = this->get_parameter("y0").get_value<double>();
            true_pose.theta = this->get_parameter("theta0").get_value<double>();

            RCLCPP_INFO(this->get_logger(), "x0 = %lf", true_pose.x);
            RCLCPP_INFO(this->get_logger(), "y0 = %lf", true_pose.x);
            RCLCPP_INFO(this->get_logger(), "z0 = %lf", true_pose.x);
        }


    private:
        double _x0, _y0, _theta0;
        // const double  DEFAULT_X0, DEFAULT_Y0, DEFAULT_THETA0;
        rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_pub;
        rclcpp::TimerBase::SharedPtr _timer;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
        uint64_t step;

        struct Pose2D {
            double x;
            double y;
            double theta;
        } true_pose;
        

        void reset_callback(
            const std::shared_ptr<std_srvs::srv::Empty::Request> request,
            std::shared_ptr<std_srvs::srv::Empty::Response> response)
        {
            // empty request and response are unused
            UNUSED(request); 
            UNUSED(response);

            step = 0;
            true_pose.x = DEFAULT_X0;
            true_pose.y = DEFAULT_Y0;
            true_pose.theta = DEFAULT_THETA0;
        }

        void timer_callback() {

            // publish timestep
            auto timestep_message = std_msgs::msg::UInt64();
            timestep_message.data = step++;
            timestep_pub->publish(timestep_message);

            // publish transform between nusim/world and red/base_footprint frames
            geometry_msgs::msg::TransformStamped world_red_tf;
            world_red_tf.header.stamp = this->get_clock()->now();
            world_red_tf.header.frame_id = "nusim/world";
            world_red_tf.child_frame_id = "red/base_footprint";

            world_red_tf.transform.translation.x = true_pose.x;
            world_red_tf.transform.translation.y = true_pose.y;
            world_red_tf.transform.translation.z = true_pose.theta;

            tf2::Quaternion q;
            q.setRPY(0.0,0.0,0.0);
            world_red_tf.transform.rotation.x = q.x();
            world_red_tf.transform.rotation.y = q.y();
            world_red_tf.transform.rotation.z = q.z();
            world_red_tf.transform.rotation.w = q.w();
            tf_broadcaster->sendTransform(world_red_tf);

        }
};


int main(int argc, char *argv[]) {
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<Nusim>());
    rclcpp::shutdown();
    return 0;
}

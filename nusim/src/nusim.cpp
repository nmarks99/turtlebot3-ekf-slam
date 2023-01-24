#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <rclcpp/logging.hpp>

#include "std_msgs/msg/u_int64.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/empty.hpp"
#include "nusim/srv/teleport.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#define UNUSED(x) (void)(x) // used to suppress "unused-variable" warnings

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

const double DEFAULT_X0 = 0.0;
const double DEFAULT_Y0 = 0.0;
const double DEFAULT_THETA0 = 0.0;
const std::vector<double> DEFAULT_OBSTACLES_X;
const std::vector<double> DEFAULT_OBSTACLES_Y;
const double DEFAULT_OBSTACLES_R = 0.038;
const double OBSTACLE_HEIGHT = 0.25;


class Nusim : public rclcpp::Node {

    public:
        Nusim() : Node("nusim"), step(0) {
            
            // Declare parameters
            this->declare_parameter<double>("x0",DEFAULT_X0);
            this->declare_parameter<double>("y0",DEFAULT_Y0);
            this->declare_parameter<double>("theta0",DEFAULT_THETA0);
            this->declare_parameter<std::vector<double>>("obstacles/x", DEFAULT_OBSTACLES_X);
            this->declare_parameter<std::vector<double>>("obstacles/y", DEFAULT_OBSTACLES_Y);
            this->declare_parameter<double>("obstacles/r", DEFAULT_OBSTACLES_R);

            // Create timestep publisher
            timestep_pub = this->create_publisher<std_msgs::msg::UInt64>("~/timestep",10);

            // Create Marker publisher 
            // marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("~/obstacles",10);
            marker_arr_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
                "~/obstacles",
                10
            );

            // Create reset service 
            // resets the timestamp and teleports the robot to its starting pose
            reset_service = this->create_service<std_srvs::srv::Empty>(
                "~/reset",
                std::bind(&Nusim::reset_callback, this, _1, _2)
            );

            // Create teleport service
            // teleports the robot in the simulation to the specified pose
            teleport_service = this->create_service<nusim::srv::Teleport>(
                "~/teleport",
                std::bind(&Nusim::teleport_callback, this, _1, _2)
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

            // Get the requested obstacle locations and size
            obstacles_r = this->get_parameter("obstacles/r").get_value<double>();
            obstacles_x = this->get_parameter("obstacles/x").get_value<std::vector<double>>();
            obstacles_y = this->get_parameter("obstacles/y").get_value<std::vector<double>>();

            RCLCPP_INFO(this->get_logger(), "x0 = %lf", true_pose.x);
            RCLCPP_INFO(this->get_logger(), "y0 = %lf", true_pose.y);
            RCLCPP_INFO(this->get_logger(), "theta0 = %lf", true_pose.theta);
            RCLCPP_INFO(this->get_logger(),"obstacles/x length = %ld", obstacles_x.size());
            RCLCPP_INFO(this->get_logger(),"obstacles/y length = %ld", obstacles_y.size());
            RCLCPP_INFO(this->get_logger(),"obstacles/r = %lf", obstacles_r);

            assert(obstacles_x.size() == obstacles_y.size());

            for (size_t i = 0; i < obstacles_x.size(); i++) {
                mark1.header.frame_id = "nusim/world";
                mark1.id = i;
                mark1.type = visualization_msgs::msg::Marker::CYLINDER;
                mark1.action = visualization_msgs::msg::Marker::ADD;
                mark1.scale.x = obstacles_r;
                mark1.scale.y = obstacles_r;
                mark1.scale.z = OBSTACLE_HEIGHT;
                mark1.pose.position.x = obstacles_x.at(i);
                mark1.pose.position.y = obstacles_y.at(i);
                mark1.color.r = 0.0;
                mark1.color.g = 1.0;
                mark1.color.b = 0.0;
                mark1.color.a = 1.0;

                mark_arr.markers.push_back(mark1);
            }

            // // Define Markers
            // mark1.header.frame_id = "nusim/world";
            // mark1.id = 0;
            // mark1.type = visualization_msgs::msg::Marker::CYLINDER;
            // mark1.action = visualization_msgs::msg::Marker::ADD;
            // mark1.scale.x = 0.1;
            // mark1.scale.y = 0.1;
            // mark1.scale.z = 0.25;
            // mark1.pose.position.x = 1.0;
            // mark1.pose.position.y = 0.5;
            // mark1.color.r = 0.0;
            // mark1.color.g = 1.0;
            // mark1.color.b = 0.0;
            // mark1.color.a = 1.0;

            // mark2.header.frame_id = "nusim/world";
            // mark2.id = 1;
            // mark2.type = visualization_msgs::msg::Marker::CYLINDER;
            // mark2.action = visualization_msgs::msg::Marker::ADD;
            // mark2.scale.x = 0.1;
            // mark2.scale.y = 0.1;
            // mark2.scale.z = 0.25;
            // mark2.pose.position.x = -1.0;
            // mark2.pose.position.y = 0.5;
            // mark2.color.r = 0.0;
            // mark2.color.g = 1.0;
            // mark2.color.b = 0.0;
            // mark2.color.a = 1.0;

            // mark_arr.markers.push_back(mark1);
            // mark_arr.markers.push_back(mark2);

        }


    private:

        std::vector<double> obstacles_x;
        std::vector<double> obstacles_y;
        double obstacles_r;

        rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_pub;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_arr_pub;
        rclcpp::TimerBase::SharedPtr _timer;

        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service;
        rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_service;
        
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

        visualization_msgs::msg::MarkerArray mark_arr;
        visualization_msgs::msg::Marker mark1;
        visualization_msgs::msg::Marker mark2;

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
            UNUSED(request); 
            UNUSED(response);

            step = 0;
            true_pose.x = DEFAULT_X0;
            true_pose.y = DEFAULT_Y0;
            true_pose.theta = DEFAULT_THETA0;
        }

        void teleport_callback(const std::shared_ptr<nusim::srv::Teleport::Request> request,
            std::shared_ptr<nusim::srv::Teleport::Response> response)
        {
            UNUSED(response);

            true_pose.x = request->x;
            true_pose.y = request->y;
            true_pose.theta = request->theta;
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

            tf2::Quaternion q;
            q.setRPY(0.0,0.0,true_pose.theta);
            world_red_tf.transform.rotation.x = q.x();
            world_red_tf.transform.rotation.y = q.y();
            world_red_tf.transform.rotation.z = q.z();
            world_red_tf.transform.rotation.w = q.w();
            tf_broadcaster->sendTransform(world_red_tf);


            marker_arr_pub->publish(mark_arr);


        }
};


int main(int argc, char *argv[]) {
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<Nusim>());
    rclcpp::shutdown();
    return 0;
}

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/empty.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

class Nusim : public rclcpp::Node {

    public:
        Nusim() : Node("nusim"), step(0) {

            publisher = this->create_publisher<std_msgs::msg::UInt64>("/nusim/timestep",10);
            
            reset_service = this->create_service<std_srvs::srv::Empty>(
                "~/reset",
                std::bind(&Nusim::reset_callback, this, _1, _2)
            );

            _timer = this->create_wall_timer(
                5ms,
                std::bind(&Nusim::timer_callback, this)
            );
        }


    private:
        rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher;
        rclcpp::TimerBase::SharedPtr _timer;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service;
        uint64_t step;

        void reset_callback(
            const std::shared_ptr<std_srvs::srv::Empty::Request> request,
            std::shared_ptr<std_srvs::srv::Empty::Response> response)
        {
            step = 0;
        }

        void timer_callback() {
            auto message = std_msgs::msg::UInt64();
            message.data = step++;
            publisher->publish(message);
        }
};


int main(int argc, char *argv[]) {
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<Nusim>());
    rclcpp::shutdown();
    return 0;
}

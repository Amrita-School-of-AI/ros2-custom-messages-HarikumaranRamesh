#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
// Include the generated message header
#include "custom_msgs/msg/robot_status.hpp"

using namespace std::chrono_literals;

class StatusPublisher : public rclcpp::Node
{
public:
    StatusPublisher()
        : Node("status_publisher"),
          battery_level_(100.0),
          mission_count_(0)
    {
        // Create publisher
        publisher_ = this->create_publisher<custom_msgs::msg::RobotStatus>(
            "/robot_status", 10);

        // Create timer (1000 ms)
        timer_ = this->create_wall_timer(
            1000ms,
            std::bind(&StatusPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        custom_msgs::msg::RobotStatus msg;

        msg.robot_name = "Explorer1";
        msg.battery_level = battery_level_;
        msg.is_active = true;
        msg.mission_count = mission_count_;

        publisher_->publish(msg);

        RCLCPP_INFO(this->get_logger(),
                    "Robot: %s | Battery: %.2f | Active: %s | Missions: %d",
                    msg.robot_name.c_str(),
                    msg.battery_level,
                    msg.is_active ? "true" : "false",
                    msg.mission_count);

        battery_level_ -= 0.5;
        mission_count_++;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<custom_msgs::msg::RobotStatus>::SharedPtr publisher_;
    double battery_level_;
    int32_t mission_count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StatusPublisher>());
    rclcpp::shutdown();
    return 0;
}

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <cmath>

using namespace std::chrono_literals;

class EightDriver : public rclcpp::Node
{
public:
    EightDriver()
    : Node("drive_eight"), phase_(0.0)
    {
        RCLCPP_INFO(this->get_logger(), "Dynamic eight movement node started");
        publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&EightDriver::drive_in_eight, this));

        linear_velocity_ = 0.15;   // m/s
        angular_velocity_ = 1;  // max rad/s
        stop_velocity_ = 0.0;
        phase_step_ = 0.05;        // controls how fast phase_ increases
    }

    void stopping()
    {
        RCLCPP_INFO(this->get_logger(), "Eight node shutting down");
        auto twist_stamped = geometry_msgs::msg::TwistStamped();
        twist_stamped.header.stamp = this->get_clock()->now();
        twist_stamped.header.frame_id = "base_link";
        twist_stamped.twist.linear.x = stop_velocity_;
        twist_stamped.twist.angular.z = stop_velocity_;
        publisher_->publish(twist_stamped);
    }

private:
    void drive_in_eight()
    {
        phase_ += phase_step_;

        auto twist_stamped = geometry_msgs::msg::TwistStamped();
        twist_stamped.header.stamp = this->get_clock()->now();
        twist_stamped.header.frame_id = "base_link";

        twist_stamped.twist.linear.x = linear_velocity_;
        twist_stamped.twist.angular.z = angular_velocity_ * std::sin(phase_);

        publisher_->publish(twist_stamped);
    }

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    double linear_velocity_;
    double angular_velocity_;
    double stop_velocity_;
    double phase_;
    double phase_step_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EightDriver>();
    rclcpp::spin(node);
    node->stopping();
    rclcpp::shutdown();
    return 0;
}

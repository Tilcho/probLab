#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

using namespace std::chrono_literals;

class CircleDriver : public rclcpp::Node
{
public:
    CircleDriver()
    : Node("circle_driver")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&CircleDriver::drive_in_circle, this));

        linear_velocity_ = 0.2;   // m/s
        angular_velocity_ = 0.5;  // rad/s
        stop_velocity_ = 0;
    }
    void stop_robot()
    {
        //RCLCPP_INFO(this->get_logger(), "Node shutting down, sending zero velocity...");
        auto twist_stamped = geometry_msgs::msg::TwistStamped();
        twist_stamped.header.stamp = this->get_clock()->now();
        twist_stamped.header.frame_id = "base_link";

        twist_stamped.twist.linear.x = stop_velocity_;
        twist_stamped.twist.angular.z = stop_velocity_;

        publisher_->publish(twist_stamped);
        rclcpp::sleep_for(100ms);  // give time for message to be sent
    }
private:
    void drive_in_circle()
    {
        auto twist_stamped = geometry_msgs::msg::TwistStamped();
        twist_stamped.header.stamp = this->get_clock()->now();
        twist_stamped.header.frame_id = "base_link";

        twist_stamped.twist.linear.x = linear_velocity_;
        twist_stamped.twist.angular.z = angular_velocity_;

        publisher_->publish(twist_stamped);
    }

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    double linear_velocity_;
    double angular_velocity_;
    double stop_velocity_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CircleDriver>();
    rclcpp::spin(node);
    node->stop_robot();  // Call explicitly before shutdown
    rclcpp::shutdown();
    return 0;
}


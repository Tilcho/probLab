#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

using namespace std::chrono_literals;

class CmdVelWatchdog : public rclcpp::Node
{
public:
    CmdVelWatchdog()
    : Node("cmd_vel_watchdog"),
      timeout_duration_(rclcpp::Duration::from_seconds(0.2))  // ✅ FIXED: initialize here
    {
        using std::placeholders::_1;

        sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/cmd_vel", 10,
            std::bind(&CmdVelWatchdog::cmd_vel_callback, this, _1)
        );

        pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);

        timer_ = this->create_wall_timer(50ms, std::bind(&CmdVelWatchdog::check_timeout, this));

        last_msg_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "CmdVel watchdog node started. Timeout = 200ms");
    }

private:
    void cmd_vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr /*msg*/)
    {
        last_msg_time_ = this->now();
    }

    void check_timeout()
    {
        if ((this->now() - last_msg_time_) > timeout_duration_)
        {
            // Publish zero velocity
            geometry_msgs::msg::TwistStamped stop_msg;
            stop_msg.header.stamp = this->get_clock()->now();
            stop_msg.header.frame_id = "base_link";
            stop_msg.twist.linear.x = 0.0;
            stop_msg.twist.angular.z = 0.0;

            pub_->publish(stop_msg);
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_msg_time_;
    rclcpp::Duration timeout_duration_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelWatchdog>());
    rclcpp::shutdown();
    return 0;
}

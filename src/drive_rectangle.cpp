#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

using namespace std::chrono_literals;

enum class State { DRIVE, TURN };

class RectangleDriver : public rclcpp::Node
{
public:
    RectangleDriver()
    : Node("drive_rectangle"),
      state_(State::DRIVE),
      side_count_(0),
      linear_velocity_(0.2),        // m/s
      angular_velocity_(0.55),       // rad/s
      drive_duration_(rclcpp::Duration::from_seconds(0.5 / linear_velocity_)),
      turn_duration_(rclcpp::Duration::from_seconds(3.9))
    {
        RCLCPP_INFO(this->get_logger(), "Rectangle driving node started");

        publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&RectangleDriver::step, this));

        action_start_time_ = this->now();
    }

    void stop()
    {
        RCLCPP_INFO(this->get_logger(), "Rectangle driving node stopped");
        publish_velocity(0.0, 0.0);
    }

private:
    void step()
    {
        rclcpp::Time now = this->now();
        rclcpp::Duration elapsed = now - action_start_time_;

        if (state_ == State::DRIVE)
        {
            if (elapsed < drive_duration_)
            {
                publish_velocity(linear_velocity_, 0.0);
            }
            else
            {
                state_ = State::TURN;
                action_start_time_ = now;
                publish_velocity(0.0, angular_velocity_);
            }
        }
        else if (state_ == State::TURN)
        {
            if (elapsed < turn_duration_)
            {
                publish_velocity(0.0, angular_velocity_);
            }
            else
            {
                side_count_++;
                if (side_count_ >= 4)
                {
                    stop();
                    rclcpp::shutdown();  // Stop the node after completing the rectangle
                    return;
                }
                state_ = State::DRIVE;
                action_start_time_ = now;
                publish_velocity(linear_velocity_, 0.0);
            }
        }
    }

    void publish_velocity(double lin, double ang)
    {
        auto msg = geometry_msgs::msg::TwistStamped();
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "base_link";
        msg.twist.linear.x = lin;
        msg.twist.angular.z = ang;
        publisher_->publish(msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time action_start_time_;

    State state_;
    int side_count_;
    double linear_velocity_, angular_velocity_;
    rclcpp::Duration drive_duration_, turn_duration_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RectangleDriver>();
    rclcpp::spin(node);
    node->stop();  // Send zero velocity once more (optional)
    return 0;
}

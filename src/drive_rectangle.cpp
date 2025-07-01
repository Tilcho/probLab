#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

using namespace std::chrono_literals;

enum class State { DRIVE, ARC };

class RoundedSquareDriver : public rclcpp::Node
{
public:
    RoundedSquareDriver()
    : Node("drive_rounded_square"),
      state_(State::DRIVE),
      segment_count_(0),
      linear_velocity_(0.2),        // m/s
      angular_velocity_(1),       // rad/s (for arcs)
      straight_duration_(rclcpp::Duration::from_seconds(1.5)),  // time per side
      arc_duration_(rclcpp::Duration::from_seconds(M_PI_2 / angular_velocity_)) // ~90° turn as arc
    {
        RCLCPP_INFO(this->get_logger(), "Rounded square node started");

        publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&RoundedSquareDriver::step, this));

        action_start_time_ = this->now();
    }

    void stop()
    {
        RCLCPP_INFO(this->get_logger(), "Rounded square node stopped");
        publish_velocity(0.0, 0.0);
    }

private:
    void step()
    {
        rclcpp::Time now = this->now();
        rclcpp::Duration elapsed = now - action_start_time_;

        if (state_ == State::DRIVE)
        {
            if (elapsed < straight_duration_)
            {
                publish_velocity(linear_velocity_, 0.0);
            }
            else
            {
                state_ = State::ARC;
                action_start_time_ = now;
                publish_velocity(linear_velocity_, angular_velocity_);
            }
        }
        else if (state_ == State::ARC)
        {
            if (elapsed < arc_duration_)
            {
                publish_velocity(linear_velocity_, angular_velocity_);
            }
            else
            {
                segment_count_++;
                if (segment_count_ >= 4)
                {
                    stop();
                    rclcpp::shutdown();
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
    int segment_count_;
    double linear_velocity_, angular_velocity_;
    rclcpp::Duration straight_duration_, arc_duration_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RoundedSquareDriver>();
    rclcpp::spin(node);
    node->stop();
    return 0;
}

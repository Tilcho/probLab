#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2/LinearMath/Quaternion.h>

using std::placeholders::_1;

class Navigator : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  Navigator()
  : Node("navigator"), current_index_(0)
  {
    this->declare_parameter("waypoints", std::vector<double>{});
    auto flat_waypoints = this->get_parameter("waypoints").as_double_array();

    if (flat_waypoints.size() % 3 != 0) {
      RCLCPP_ERROR(this->get_logger(), "Waypoints must be a flat list of x, y, yaw triples.");
      return;
    }

    for (size_t i = 0; i < flat_waypoints.size(); i += 3) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.pose.position.x = flat_waypoints[i];
      pose.pose.position.y = flat_waypoints[i + 1];

      tf2::Quaternion q;
      q.setRPY(0, 0, flat_waypoints[i + 2]);
      pose.pose.orientation.x = q.x();
      pose.pose.orientation.y = q.y();
      pose.pose.orientation.z = q.z();
      pose.pose.orientation.w = q.w();

      waypoints_.push_back(pose);
    }

    client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);

    timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&Navigator::start_navigation, this));
  }

private:
  void publish_initial_pose()
  {
    auto pose = waypoints_.front();

    geometry_msgs::msg::PoseWithCovarianceStamped init_pose;
    init_pose.header.frame_id = "map";
    init_pose.header.stamp = now();
    init_pose.pose.pose = pose.pose;
    init_pose.pose.covariance[0] = 0.25;
    init_pose.pose.covariance[7] = 0.25;
    init_pose.pose.covariance[35] = 0.0685;

    RCLCPP_INFO(this->get_logger(), "Publishing initial pose to AMCL.");
    initial_pose_pub_->publish(init_pose);
  }

  void start_navigation()
  {
    if (!client_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_WARN(this->get_logger(), "Waiting for navigate_to_pose action server...");
      return;
    }

    publish_initial_pose();
    rclcpp::sleep_for(std::chrono::seconds(2));  // wait for AMCL to accept pose
    send_next_goal();
  }

  void send_next_goal()
  {
    if (current_index_ >= waypoints_.size()) {
      RCLCPP_INFO(this->get_logger(), "All waypoints completed.");
      return;
    }

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = waypoints_[current_index_];
    goal_msg.pose.header.stamp = now();
    goal_msg.pose.header.frame_id = "map";

    RCLCPP_INFO(this->get_logger(), "Sending goal %lu: (%.2f, %.2f)", current_index_,
                goal_msg.pose.pose.position.x, goal_msg.pose.pose.position.y);

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&Navigator::goal_result_callback, this, _1);
    client_->async_send_goal(goal_msg, send_goal_options);
  }

  void goal_result_callback(const GoalHandle::WrappedResult & result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(this->get_logger(), "Goal %lu succeeded.", current_index_);
    } else {
      RCLCPP_WARN(this->get_logger(), "Goal %lu failed with code %d", current_index_,
                  static_cast<int>(result.code));
    }

    ++current_index_;
    send_next_goal();
  }

  std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t current_index_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Navigator>());
  rclcpp::shutdown();
  return 0;
}

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Dense>
#include <mutex>
#include <cmath>

class EKFLocalization : public rclcpp::Node {
public:
  EKFLocalization() : Node("ekf_localization_node") {
    wheel_base_ = declare_parameter("wheel_base", 0.287);
    wheel_radius_ = declare_parameter("wheel_radius", 0.033);

    x_ = Eigen::VectorXd::Zero(6);
    u_ = Eigen::VectorXd::Zero(2);

    P_ = Eigen::MatrixXd::Identity(6, 6);
    P_.diagonal() << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01;

    Q_ = Eigen::MatrixXd::Zero(6, 6);
    Q_.diagonal() << 0.001, 0.002, 0.001, 0.002, 0.001, 0.005;

    R_ = Eigen::MatrixXd::Identity(3, 3);
    R_.diagonal() << 0.02, 0.02, 0.05;  // Gyro bekommt weniger Vertrauen

    imu_sub_.subscribe(this, "/imu");
    joint_sub_.subscribe(this, "/joint_states");
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), imu_sub_, joint_sub_);
    sync_->registerCallback(std::bind(&EKFLocalization::sensorCallback, this, std::placeholders::_1, std::placeholders::_2));

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "/cmd_vel", 10,
      [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        u_ << msg->twist.linear.x, msg->twist.angular.z;
      });

    ekf_pub_ = create_publisher<nav_msgs::msg::Odometry>("/ekf_state", 10);
    last_time_ = this->now();
    RCLCPP_INFO(get_logger(), "EKF gestartet.");
  }

private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Imu, sensor_msgs::msg::JointState>;

  Eigen::VectorXd x_, u_;
  Eigen::MatrixXd P_, Q_, R_;
  double wheel_base_, wheel_radius_;
  rclcpp::Time last_time_;
  bool initialized_ = false;
  std::mutex state_mutex_;

  message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sub_;
  message_filters::Subscriber<sensor_msgs::msg::JointState> joint_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ekf_pub_;

  void sensorCallback(const sensor_msgs::msg::Imu::ConstSharedPtr imu,
                      const sensor_msgs::msg::JointState::ConstSharedPtr joint) {
    std::lock_guard<std::mutex> lock(state_mutex_);

    rclcpp::Time now_time(imu->header.stamp);
    double dt = (now_time - last_time_).seconds();
    if (dt <= 0.0 || dt > 1.0) return;
    last_time_ = now_time;

    if (!initialized_) {
      auto [vx, vy] = computeVelocities(joint);
      
      // 🧭 Initialisierung mit IMU-Yaw
      tf2::Quaternion q;
      tf2::fromMsg(imu->orientation, q);
      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

      x_ << 0.0, vx, 0.0, vy, yaw, imu->angular_velocity.z;
      initialized_ = true;
      RCLCPP_INFO(get_logger(), "EKF initialisiert mit yaw = %.3f", yaw);
      return;
    }

    predict(dt);
    update(imu, joint);
    publish(now_time);
  }

  void predict(double dt) {
    double theta = x_(4);
    double v = u_(0);
    double omega = u_(1);

    // 🔄 Nichtlineares Bewegungsmodell
    x_(0) += v * std::cos(theta) * dt;
    x_(2) += v * std::sin(theta) * dt;
    x_(4) += omega * dt;

    x_(1) = v * std::cos(theta);
    x_(3) = v * std::sin(theta);
    x_(5) = omega;

    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6, 6);
    F(0, 4) = -v * std::sin(theta) * dt;
    F(2, 4) =  v * std::cos(theta) * dt;

    P_ = F * P_ * F.transpose() + Q_;
    normalizeAngle();
  }

  void update(const sensor_msgs::msg::Imu::ConstSharedPtr imu,
              const sensor_msgs::msg::JointState::ConstSharedPtr joint) {
    auto [vx, vy] = computeVelocities(joint);
    double omega = imu->angular_velocity.z;

    Eigen::VectorXd z(3); z << vx, vy, omega;
    Eigen::VectorXd h = x_.segment<3>(1);

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 6);
    H(0, 1) = 1.0; H(1, 3) = 1.0; H(2, 5) = 1.0;

    Eigen::MatrixXd S = H * P_ * H.transpose() + R_;
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

    x_ += K * (z - h);
    P_ = (Eigen::MatrixXd::Identity(6, 6) - K * H) * P_;
    normalizeAngle();
  }

  std::pair<double, double> computeVelocities(const sensor_msgs::msg::JointState::ConstSharedPtr joint) {
    double v_left = 0.0, v_right = 0.0;
    for (size_t i = 0; i < joint->name.size(); ++i) {
      if (joint->name[i] == "wheel_left_joint") v_left = joint->velocity[i];
      if (joint->name[i] == "wheel_right_joint") v_right = joint->velocity[i];
    }

    double v = (v_left + v_right) * 0.5 * wheel_radius_;
    double theta = x_(4);
    return {v * std::cos(theta), v * std::sin(theta)};
  }

  void publish(const rclcpp::Time& stamp) {
    nav_msgs::msg::Odometry msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "odom";
    msg.child_frame_id = "base_footprint";

    msg.pose.pose.position.x = x_(0);
    msg.pose.pose.position.y = x_(2);

    tf2::Quaternion q;
    q.setRPY(0, 0, x_(4));
    msg.pose.pose.orientation = tf2::toMsg(q);

    msg.twist.twist.linear.x = x_(1);
    msg.twist.twist.linear.y = x_(3);
    msg.twist.twist.angular.z = x_(5);

    msg.pose.covariance[0] = P_(0, 0);
    msg.pose.covariance[7] = P_(2, 2);
    msg.pose.covariance[35] = P_(4, 4);
    msg.twist.covariance[0] = P_(1, 1);
    msg.twist.covariance[7] = P_(3, 3);
    msg.twist.covariance[35] = P_(5, 5);

    ekf_pub_->publish(msg);
  }

  void normalizeAngle() {
    while (x_(4) > M_PI) x_(4) -= 2.0 * M_PI;
    while (x_(4) < -M_PI) x_(4) += 2.0 * M_PI;
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EKFLocalization>());
  rclcpp::shutdown();
  return 0;
}
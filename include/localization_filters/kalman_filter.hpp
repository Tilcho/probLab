#ifndef LOCALIZATION_FILTERS_KALMAN_FILTER_HPP
#define LOCALIZATION_FILTERS_KALMAN_FILTER_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"

#include "Eigen/Dense"

class KalmanFilter {
public:
    KalmanFilter();
    void predict(
        const geometry_msgs::msg::Twist& u, double dt);
    geometry_msgs::msg::PoseWithCovarianceStamped correctIMU(
        const sensor_msgs::msg::Imu::ConstSharedPtr& imu,
        double dt);
    geometry_msgs::msg::PoseWithCovarianceStamped correct(
        const sensor_msgs::msg::Imu::ConstSharedPtr& imu,
        const sensor_msgs::msg::JointState::ConstSharedPtr& joint,
        double dt);
private:
    Eigen::VectorXd x_; // State vector (6x1): [x_pos, x_vel, y_pos, y_vel, theta, omega]
    Eigen::MatrixXd S_; // Covariance matrix (6x6)
    Eigen::MatrixXd A_; // State transition matrix (6x6)
    Eigen::MatrixXd B_; // Control input matrix (6x2)
    Eigen::MatrixXd R_; // Process noise covariance (6x6)
    Eigen::MatrixXd C_; // Measurement matrix (3x6)
    Eigen::MatrixXd Q_; // Measurement noise covariance (3x3)

    const double wheelbase_ = 0.160;  // TurtleBot3 wheelbase in meters
    const double wheel_rad_ = 0.033;  // TurtleBot3 wheel radius in meters
};

class FilterNode : public rclcpp::Node {
public:
    FilterNode();

private:
    void synced_callback(
        const nav_msgs::msg::Odometry::ConstSharedPtr& odom,
        const sensor_msgs::msg::Imu::ConstSharedPtr& imu,
        const sensor_msgs::msg::JointState::ConstSharedPtr& joint);

    KalmanFilter kf_;
    rclcpp::Time last_time_;
    bool first_msg_received_ = false;

    message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sub_;
    message_filters::Subscriber<sensor_msgs::msg::JointState> joint_sub_;

    using TripleSyncPolicy = message_filters::sync_policies::ApproximateTime<
    nav_msgs::msg::Odometry,
    sensor_msgs::msg::Imu,
    sensor_msgs::msg::JointState>;

    std::shared_ptr<message_filters::Synchronizer<TripleSyncPolicy>> triple_sync_;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pred_pub_;
};

#endif  // LOCALIZATION_FILTERS_KALMAN_FILTER_HPP

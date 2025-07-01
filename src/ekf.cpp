#include "localization_filters/ekf.hpp"

// ===============================
// ExtendedKalmanFilter Implementation
// ===============================

ExtendedKalmanFilter::ExtendedKalmanFilter() {
    x_ = Eigen::VectorXd::Zero(6); // [x_pos, x_vel, y_pos, y_vel, theta, omega]
    P_ = Eigen::MatrixXd::Identity(6, 6) * 0.1;
    F_ = Eigen::MatrixXd::Identity(6, 6); // Jacobian of motion model
    H_ = Eigen::MatrixXd::Zero(3, 6);     // Jacobian of observation model
    Q_ = Eigen::MatrixXd::Identity(6, 6); // Process noise
    Q_(0, 0) = 0.01;  // x_pos
    Q_(1, 1) = 0.1;   // x_vel
    Q_(2, 2) = 0.01;  // y_pos
    Q_(3, 3) = 0.1;   // y_vel
    Q_(4, 4) = 0.001; // theta
    Q_(5, 5) = 0.01;  // omega
    R_ = Eigen::MatrixXd::Identity(3, 3); // Measurement noise
    R_(0, 0) = 0.05;  // x_vel (wheel encoders)
    R_(1, 1) = 0.1;   // y_vel (often assumed zero)
    R_(2, 2) = 0.02;  // omega (IMU)

}

void ExtendedKalmanFilter::predict(const geometry_msgs::msg::Twist& u, double dt) {
    double v = u.linear.x;
    double w = u.angular.z;
    double theta = x_(4);

    // Nonlinear motion model
    x_(0) += v * std::cos(theta) * dt; // x_pos
    x_(2) += v * std::sin(theta) * dt; // y_pos
    x_(4) += w * dt;                   // theta

    // Update velocity components
    x_(1) = v * std::cos(theta);
    x_(3) = v * std::sin(theta);
    x_(5) = w;

    // Jacobian of the motion model (F_)
    F_.setIdentity();
    F_(0, 4) = -v * std::sin(theta) * dt;
    F_(2, 4) = v * std::cos(theta) * dt;
    F_(0, 1) = dt;
    F_(2, 3) = dt;
    F_(4, 5) = dt;

    // Predict covariance
    P_ = F_ * P_ * F_.transpose() + Q_;

    while (x_(4) > M_PI) x_(4) -= 2 * M_PI;
    while (x_(4) < -M_PI) x_(4) += 2 * M_PI;
}

geometry_msgs::msg::PoseWithCovarianceStamped ExtendedKalmanFilter::correct(
    const sensor_msgs::msg::Imu::ConstSharedPtr& imu,
    const sensor_msgs::msg::JointState::ConstSharedPtr& joint,
    double dt) {

    double v_left = 0, v_right = 0;
    for (size_t i = 0; i < joint->name.size(); ++i) {
        if (joint->name[i] == "wheel_left_joint") {
            v_left = joint->velocity[i] * wheel_rad_;
        } else if (joint->name[i] == "wheel_right_joint") {
            v_right = joint->velocity[i] * wheel_rad_;
        }
    }

    double v = (v_left + v_right) / 2;
    double w = (v_right - v_left) / wheelbase_;

    // Measurement vector z: [x_vel, y_vel, omega]
    Eigen::Vector3d z;
    z << v * std::cos(x_(4)), v * std::sin(x_(4)), w;

    // Measurement model Jacobian H_
    H_.setZero();
    H_(0, 1) = 1; // x_vel
    H_(1, 3) = 1; // y_vel
    H_(2, 5) = 1; // omega

    // Innovation
    Eigen::Vector3d y = z - H_ * x_;

    // Innovation covariance
    Eigen::Matrix3d S = H_ * P_ * H_.transpose() + R_;
    Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();

    // State update
    x_ = x_ + K * y;
    P_ = (Eigen::MatrixXd::Identity(6, 6) - K * H_) * P_;

    while (x_(4) > M_PI) x_(4) -= 2 * M_PI;
    while (x_(4) < -M_PI) x_(4) += 2 * M_PI;

    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.pose.pose.position.x = x_(0);
    pose.pose.pose.position.y = x_(2);
    pose.pose.pose.orientation.z = sin(x_(4) / 2.0);
    pose.pose.pose.orientation.w = cos(x_(4) / 2.0);

    pose.pose.covariance[0] = P_(0, 0);
    pose.pose.covariance[7] = P_(2, 2);
    pose.pose.covariance[35] = P_(4, 4);

    return pose;
}

// ==============================
// FilterNode Constructor for EKF
// ==============================

FilterNode::FilterNode() : Node("ekf_node") {
    RCLCPP_INFO(this->get_logger(), "Extended Kalman Filter node started.");

    odom_sub_.subscribe(this, "/odom");
    imu_sub_.subscribe(this, "/imu");
    joint_sub_.subscribe(this, "/joint_states");

    triple_sync_ = std::make_shared<message_filters::Synchronizer<TripleSyncPolicy>>(
        TripleSyncPolicy(10), odom_sub_, imu_sub_, joint_sub_);

    triple_sync_->registerCallback(std::bind(
        &FilterNode::synced_callback, this,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3));

    pred_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/ekf_pose", 10);
}

void FilterNode::synced_callback(
    const nav_msgs::msg::Odometry::ConstSharedPtr& odom,
    const sensor_msgs::msg::Imu::ConstSharedPtr& imu,
    const sensor_msgs::msg::JointState::ConstSharedPtr& joint) {

    rclcpp::Time current_time = this->now();
    double dt;

    if (!first_msg_received_) {
        first_msg_received_ = true;
        last_time_ = current_time;
        return;
    }

    dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

    ekf_.predict(odom->twist.twist, dt);
    auto corrected = ekf_.correct(imu, joint, dt);

    corrected.header.stamp = current_time;
    corrected.header.frame_id = "map";
    pred_pub_->publish(corrected);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FilterNode>());
    rclcpp::shutdown();
    return 0;
}

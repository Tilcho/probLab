#include "localization_filters/ekf.hpp"

// ===============================
// ExtendedKalmanFilter Implementation
// ===============================

// Constructor: Initializes state, covariance, process noise, and measurement noise
ExtendedKalmanFilter::ExtendedKalmanFilter() {
    x_ = Eigen::VectorXd::Zero(6); // State vector: [x_pos, x_vel, y_pos, y_vel, theta, omega]
    P_ = Eigen::MatrixXd::Identity(6, 6) * 0.01; // Initial covariance matrix with small uncertainty
    F_ = Eigen::MatrixXd::Identity(6, 6); // Jacobian of the motion model
    H_ = Eigen::MatrixXd::Zero(3, 6);     // Jacobian of the measurement model

    // Process noise covariance matrix Q_
    Q_ = Eigen::MatrixXd::Identity(6, 6);
    Q_(0, 0) = 0.01;  // Noise for x_pos
    Q_(1, 1) = 0.1;   // Noise for x_vel
    Q_(2, 2) = 0.01;  // Noise for y_pos
    Q_(3, 3) = 0.1;   // Noise for y_vel
    Q_(4, 4) = 0.001; // Noise for theta
    Q_(5, 5) = 0.01;  // Noise for omega

    // Measurement noise covariance matrix R_
    R_ = Eigen::MatrixXd::Identity(3, 3);
    R_(0, 0) = 0.05;  // Noise for x_vel (from encoders)
    R_(1, 1) = 0.01;   // Noise for y_vel (often assumed near-zero)
    R_(2, 2) = 0.02;  // Noise for angular velocity omega (from IMU)
}

// Prediction step of EKF using control input (velocity and angular rate)
void ExtendedKalmanFilter::predict(const geometry_msgs::msg::Twist& u, double dt) {
    double v = u.linear.x;     // Forward linear velocity
    double w = u.angular.z;    // Angular velocity
    double theta = x_(4);      // Current heading

    // Predict new position using motion model
    x_(0) += v * std::cos(theta) * dt; // x_pos update
    x_(2) += v * std::sin(theta) * dt; // y_pos update
    x_(4) += w * dt;                   // theta update

    // Update velocity components in state
    x_(1) = v * std::cos(theta); // x_vel
    x_(3) = v * std::sin(theta); // y_vel
    x_(5) = w;                   // omega

    // Update motion model Jacobian matrix F_
    F_.setIdentity();
    F_(0, 4) = -v * std::sin(theta) * dt;
    F_(2, 4) = v * std::cos(theta) * dt;
    F_(0, 1) = dt;
    F_(2, 3) = dt;
    F_(4, 5) = dt;

    // Update covariance matrix using Jacobian and process noise
    P_ = F_ * P_ * F_.transpose() + Q_;

    // Normalize angle to [-π, π]
    while (x_(4) > M_PI) x_(4) -= 2 * M_PI;
    while (x_(4) < -M_PI) x_(4) += 2 * M_PI;
}

// Correction step of EKF using IMU and wheel encoder data
geometry_msgs::msg::PoseWithCovarianceStamped ExtendedKalmanFilter::correct(
    const sensor_msgs::msg::Imu::ConstSharedPtr& imu,
    const sensor_msgs::msg::JointState::ConstSharedPtr& joint,
    double dt) {

    // Extract left and right wheel velocities from joint state
    double v_left = 0, v_right = 0;
    for (size_t i = 0; i < joint->name.size(); ++i) {
        if (joint->name[i] == "wheel_left_joint") {
            v_left = joint->velocity[i] * wheel_rad_;
        } else if (joint->name[i] == "wheel_right_joint") {
            v_right = joint->velocity[i] * wheel_rad_;
        }
    }

    // Compute linear and angular velocity from wheel speeds
    double v = (v_left + v_right) / 2;
    double w = (v_right - v_left) / wheelbase_;

    // Construct measurement vector: [x_vel, y_vel, omega]
    Eigen::Vector3d z;
    z << v * std::cos(x_(4)), v * std::sin(x_(4)), w;

    // Measurement model Jacobian H_
    H_.setZero();
    H_(0, 1) = 1; // x_vel
    H_(1, 3) = 1; // y_vel
    H_(2, 5) = 1; // omega

    // Compute innovation (difference between measurement and prediction)
    Eigen::Vector3d y = z - H_ * x_;

    // Compute innovation covariance
    Eigen::Matrix3d S = H_ * P_ * H_.transpose() + R_;
    // Compute Kalman gain
    Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();

    // Update state estimate
    x_ = x_ + K * y;
    // Update covariance
    P_ = (Eigen::MatrixXd::Identity(6, 6) - K * H_) * P_;

    // Normalize angle to [-π, π]
    while (x_(4) > M_PI) x_(4) -= 2 * M_PI;
    while (x_(4) < -M_PI) x_(4) += 2 * M_PI;

    // Prepare pose message with updated state and covariance
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.pose.pose.position.x = x_(0);
    pose.pose.pose.position.y = x_(2);
    pose.pose.pose.orientation.z = sin(x_(4) / 2.0);
    pose.pose.pose.orientation.w = cos(x_(4) / 2.0);

    pose.pose.covariance[0] = P_(0, 0);   // x_pos variance
    pose.pose.covariance[7] = P_(2, 2);   // y_pos variance
    pose.pose.covariance[35] = P_(4, 4);  // theta variance

    return pose;
}

// ==============================
// FilterNode Constructor for EKF
// ==============================

// Node constructor: sets up subscribers, synchronizer, and publisher
FilterNode::FilterNode() : Node("ekf_node") {
    RCLCPP_INFO(this->get_logger(), "Extended Kalman Filter node started.");

    // Subscribe to odometry, IMU, and joint states using message_filters
    odom_sub_.subscribe(this, "/odom");
    imu_sub_.subscribe(this, "/imu");
    joint_sub_.subscribe(this, "/joint_states");

    // Approximate time synchronizer for the three sensor inputs
    triple_sync_ = std::make_shared<message_filters::Synchronizer<TripleSyncPolicy>>(
        TripleSyncPolicy(10), odom_sub_, imu_sub_, joint_sub_);

    // Register callback to process synced messages
    triple_sync_->registerCallback(std::bind(
        &FilterNode::synced_callback, this,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3));

    // Publisher for the EKF-corrected pose
    pred_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/ekf_pose", 10);
}

// Callback to handle synchronized sensor messages
void FilterNode::synced_callback(
    const nav_msgs::msg::Odometry::ConstSharedPtr& odom,
    const sensor_msgs::msg::Imu::ConstSharedPtr& imu,
    const sensor_msgs::msg::JointState::ConstSharedPtr& joint) {

    rclcpp::Time current_time = this->now();
    double dt;

    // On first message, just initialize timestamp
    if (!first_msg_received_) {
        first_msg_received_ = true;
        last_time_ = current_time;
        return;
    }

    // Compute time delta
    dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

    // Run EKF prediction and correction steps
    ekf_.predict(odom->twist.twist, dt);
    auto corrected = ekf_.correct(imu, joint, dt);

    // Set timestamp and frame_id for output pose
    corrected.header.stamp = current_time;
    corrected.header.frame_id = "map";
    pred_pub_->publish(corrected); // Publish the corrected pose
}

// Entry point: initialize and spin the node
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FilterNode>());
    rclcpp::shutdown();
    return 0;
}

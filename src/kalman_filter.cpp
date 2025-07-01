#include "localization_filters/kalman_filter.hpp"  // Include the header for this node and Kalman filter class

// ===========================
// KalmanFilter Implementation
// ===========================

KalmanFilter::KalmanFilter() {
    x_ = Eigen::VectorXd::Zero(6);                         // Initial state vector: [x, x_dot, y, y_dot, theta, omega]
    S_ = Eigen::MatrixXd::Identity(6, 6) * 0.1;            // Initial state covariance matrix with small uncertainty
    A_ = Eigen::MatrixXd::Identity(6, 6);                  // State transition model (updated each step with dt)
    B_ = Eigen::MatrixXd::Zero(6, 2);                      // Control input model (to be filled with motion info)
    R_ = Eigen::MatrixXd::Identity(6, 6) * 0.001;          // Process noise covariance (small assumed noise)
    C_ = Eigen::MatrixXd::Zero(3, 6);                      // Observation model (updated in correct())
    Q_ = Eigen::MatrixXd::Identity(3, 3) * 0.1;            // Measurement noise covariance
}

void KalmanFilter::predict(const geometry_msgs::msg::Twist& u, double dt) {
    double v = u.linear.x;   // Linear velocity from odometry
    double w = u.angular.z;  // Angular velocity (yaw rate) from odometry

    Eigen::Vector2d u_vec;  // Control input vector: [v, w]
    u_vec << v, w;

    double theta = x_(4);  // Current orientation (theta)

    // Update state transition matrix with timestep
    A_(0, 1) = dt;  // x_pos += x_vel * dt
    A_(2, 3) = dt;  // y_pos += y_vel * dt
    A_(4, 5) = dt;  // theta += omega * dt

    // Update control matrix based on motion model
    B_(1, 0) = 1;  // x_vel influenced by angular motion cos(theta) * dt, theta close to zero
    B_(3, 0) = 0;  // y_vel influenced by angular motion sin(theta) * dt, theta close to zero
    B_(5, 1) = 1;  // omega updated by rotation rate

    x_ = A_ * x_ + B_ * u_vec;            // Predict next state
    S_ = A_ * S_ * A_.transpose() + R_;   // Predict next covariance

    while (x_(4) > M_PI) x_(4) -= 2 * M_PI;
    while (x_(4) < -M_PI) x_(4) += 2 * M_PI;

}

geometry_msgs::msg::PoseWithCovarianceStamped KalmanFilter::correctIMU(
    const sensor_msgs::msg::Imu::ConstSharedPtr& imu,
    double dt)
{
    // Compute estimated velocity from acceleration integration
    double delta_x_vel = imu->linear_acceleration.x * dt;
    double delta_y_vel = imu->linear_acceleration.y * dt;

    // std::cout << "acc x: " << imu->linear_acceleration.x << std::endl;
    // std::cout << "acc y: " << imu->linear_acceleration.y << std::endl;
    // std::cout << "acc y: " << imu->angular_velocity.z << std::endl;

    // Use current state's previous velocities and apply delta
    double x_vel_measured = x_(1) + delta_x_vel;
    double y_vel_measured = x_(3) + delta_y_vel;

    // Measurement vector now uses velocity, not raw acceleration
    Eigen::VectorXd z(3);
    z << x_vel_measured, y_vel_measured, imu->angular_velocity.z;

    // Observation model: C maps state to measured values
    C_ = Eigen::MatrixXd::Zero(3, 6);
    C_(0, 1) = 1.0; // x_vel
    C_(1, 3) = 1.0; // y_vel
    C_(2, 5) = 1.0; // omega

    // Covariance of the observation noise
    Q_(0, 0) = 0.05;   // x_vel noise
    Q_(1, 1) = 0.05;   // y_vel noise
    Q_(2, 2) = 0.01;  // omega

    Eigen::VectorXd y = z - C_ * x_;  // Innovation
    Eigen::MatrixXd S = C_ * S_ * C_.transpose() + Q_;  // Innovation covariance
    
    Eigen::MatrixXd K = S_ * C_.transpose() * S.inverse();  // Kalman Gain

    //std::cout << "Coveriance:\n" << S_ << std::endl;
    //std::cout << "S matrix:\n" << S << std::endl;
    //std::cout << "Kalman Gain:\n" << K << std::endl;

    x_ = x_ + K * y;  // Update state
    S_ = (Eigen::MatrixXd::Identity(6, 6) - K * C_) * S_;  // Update covariance

    // Output estimated pose
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.pose.pose.position.x = x_(0);
    pose.pose.pose.position.y = x_(2);
    pose.pose.pose.orientation.z = sin(x_(4) / 2.0);
    pose.pose.pose.orientation.w = cos(x_(4) / 2.0);

    pose.pose.covariance[0] = S_(0, 0);
    pose.pose.covariance[7] = S_(2, 2);
    pose.pose.covariance[35] = S_(4, 4);

    return pose;
}

geometry_msgs::msg::PoseWithCovarianceStamped KalmanFilter::correct(
    const sensor_msgs::msg::Imu::ConstSharedPtr& imu,
    const sensor_msgs::msg::JointState::ConstSharedPtr& joint,
    double dt)
{
    double y_vel = 0;
    double x_vel_calc, omega_calc;

    // Find wheel velocities in joint state message
    double v_left = 0, v_right = 0;
    
    for (size_t i = 0; i < joint->name.size(); ++i) {
      if (joint->name[i] == "wheel_left_joint") {
        v_left = joint->velocity[i] * wheel_rad_;  // Convert rad/s to m/s
      } else if (joint->name[i] == "wheel_right_joint") {
        v_right = joint->velocity[i] * wheel_rad_;  // Convert rad/s to m/s
      }
    }
    
    // Differential drive kinematics
    x_vel_calc = (v_left + v_right) / 2;          // Forward velocity
    omega_calc = (v_right - v_left) / wheelbase_;   // Angular velocity from wheels

    // Create measurement vector: [x_vel, y_vel, omega]
    Eigen::Vector3d z;
    z << x_vel_calc, y_vel, omega_calc;

    // Observation matrix C_: maps state to measurement
    C_ = Eigen::MatrixXd::Zero(3, 6);
    C_(0, 1) = 1; // x_vel
    C_(1, 3) = 1; // y_vel
    C_(2, 5) = 1; // omega

    // Measurement noise covariance (tune these)
    Q_ = Eigen::MatrixXd::Identity(3, 3);
    Q_(0, 0) = 0.05;  // x noise
    Q_(1, 1) = 0.05;  // y noise
    Q_(2, 2) = 0.02;  // theta noise

    Eigen::VectorXd y = z - C_ * x_;  // Innovation
    Eigen::MatrixXd S = C_ * S_ * C_.transpose() + Q_;  // Innovation covariance
    
    Eigen::MatrixXd K = S_ * C_.transpose() * S.inverse();  // Kalman Gain

    //std::cout << "Coveriance:\n" << S_ << std::endl;
    //std::cout << "S matrix:\n" << S << std::endl;
    //std::cout << "Kalman Gain:\n" << K << std::endl;

    x_ = x_ + K * y;  // Update state
    S_ = (Eigen::MatrixXd::Identity(6, 6) - K * C_) * S_;  // Update covariance

    // Output estimated pose
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.pose.pose.position.x = x_(0);
    pose.pose.pose.position.y = x_(2);
    pose.pose.pose.orientation.z = sin(x_(4) / 2.0);
    pose.pose.pose.orientation.w = cos(x_(4) / 2.0);

    pose.pose.covariance[0] = S_(0, 0);
    pose.pose.covariance[7] = S_(2, 2);
    pose.pose.covariance[35] = S_(4, 4);

    return pose;
}

// =======================
// FilterNode Constructor
// =======================

FilterNode::FilterNode() : Node("kalman_filter_node") {
    RCLCPP_INFO(this->get_logger(), "Synchronized Kalman Filter node started.");
    // Set up message_filters subscribers for /odom /imu and /joint_state
    odom_sub_.subscribe(this, "/odom");
    imu_sub_.subscribe(this, "/imu");
    joint_sub_.subscribe(this, "/joint_states");

    // Use approximate time synchronization with queue size 10
    triple_sync_ = std::make_shared<message_filters::Synchronizer<TripleSyncPolicy>>(
        TripleSyncPolicy(10), odom_sub_, imu_sub_, joint_sub_);

    // Register callback for synchronized message pairs
    triple_sync_->registerCallback(std::bind(
        &FilterNode::synced_callback, this,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3));

    // Publisher for filtered pose
    pred_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/kf_pose", 10);
}

// ==============================
// Synced Callback Implementation
// ==============================

void FilterNode::synced_callback(
    const nav_msgs::msg::Odometry::ConstSharedPtr& odom,
    const sensor_msgs::msg::Imu::ConstSharedPtr& imu,
    const sensor_msgs::msg::JointState::ConstSharedPtr& joint) {
    // Get current time from ROS clock (respects use_sim_time)
    rclcpp::Time current_time = this->now();
    double dt;

    // On the first message pair, initialize timing
    if (!first_msg_received_) {
        first_msg_received_ = true;
        last_time_ = current_time;
        return;  // Skip predict step
    }

    // Compute timestep since last callback
    dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

    // RCLCPP_INFO(this->get_logger(), "dt = %f", dt);

    // Run prediction and correction steps
    kf_.predict(odom->twist.twist, dt);
    //auto corrected = kf_.correctIMU(imu, dt);   // Old version of correction
    auto corrected = kf_.correct(imu, joint, dt);

    // Fill ROS message header
    corrected.header.stamp = current_time;
    corrected.header.frame_id = "map";

    //RCLCPP_INFO(this->get_logger(), "Publishing corrected pose");

    // Publish filtered pose
    pred_pub_->publish(corrected);
}

// ============
// Main Program
// ============

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);                               // Initialize ROS 2
    rclcpp::spin(std::make_shared<FilterNode>());           // Create and spin the filter node
    rclcpp::shutdown();                                     // Shutdown ROS 2 cleanly
    return 0;
}
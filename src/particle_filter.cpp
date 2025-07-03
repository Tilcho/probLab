#include "localization_filters/particle_filter.hpp"

// Constructor: initializes particles at origin with equal weights and Gaussian noise models
ParticleFilter::ParticleFilter(size_t num_particles)
    : noise_x_(0.0, 0.01), noise_y_(0.0, 0.01), noise_theta_(0.0, 0.01) {
    particles_.resize(num_particles);
    for (auto& p : particles_) {
        p.x = 0.0;
        p.y = 0.0;
        p.theta = 0.0;
        p.weight = 1.0 / num_particles;
    }
}

// Computes the Gaussian probability density for value x given mean mu and standard deviation sigma
double ParticleFilter::gaussian_prob(double mu, double sigma, double x) {
    return std::exp(-0.5 * std::pow((x - mu) / sigma, 2)) / (sigma * std::sqrt(2.0 * M_PI));
}

// Applies motion model with added Gaussian noise to each particle
void ParticleFilter::predict(double v, double w, double dt) {
    for (auto& p : particles_) {
        p.x += v * std::cos(p.theta) * dt + noise_x_(rng_);
        p.y += v * std::sin(p.theta) * dt + noise_y_(rng_);
        p.theta += w * dt + noise_theta_(rng_);
    }
}

// Updates particle weights based on how closely predicted omega matches the IMU measurement
void ParticleFilter::correct(double measured_omega, double predicted_omega) {
    double sigma = 0.02; // Assumed IMU angular velocity noise
    double total_weight = 0.0;

    for (auto& p : particles_) {
        p.weight = gaussian_prob(predicted_omega, sigma, measured_omega);
        total_weight += p.weight;
    }

    // Normalize weights
    if (total_weight > 0.0) {
        for (auto& p : particles_) {
            p.weight /= total_weight;
        }
    } else {
        // Fallback: reset to uniform if all weights collapsed
        for (auto& p : particles_) {
            p.weight = 1.0 / particles_.size();
        }
    }
}

// Resamples particles according to their weights using discrete distribution
void ParticleFilter::resample() {
    std::vector<Particle> new_particles;
    std::vector<double> weights;

    for (const auto& p : particles_) {
        weights.push_back(p.weight);
    }

    std::discrete_distribution<> dist(weights.begin(), weights.end());

    for (size_t i = 0; i < particles_.size(); ++i) {
        new_particles.push_back(particles_[dist(rng_)]);
    }

    particles_ = std::move(new_particles);
}

// Computes average position and orientation of all particles as estimated robot pose
geometry_msgs::msg::PoseWithCovarianceStamped ParticleFilter::estimate_pose() {
    double x_sum = 0, y_sum = 0, theta_sum = 0;
    for (const auto& p : particles_) {
        x_sum += p.x;
        y_sum += p.y;
        theta_sum += p.theta;
    }

    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.pose.pose.position.x = x_sum / particles_.size();
    pose.pose.pose.position.y = y_sum / particles_.size();
    pose.pose.pose.orientation.z = std::sin(theta_sum / particles_.size() / 2.0);
    pose.pose.pose.orientation.w = std::cos(theta_sum / particles_.size() / 2.0);
    return pose;
}

// ======================
// ROS Node Implementation
// ======================

// Constructor: sets up subscriptions and message synchronizer
ParticleFilterNode::ParticleFilterNode() : Node("particle_filter_node") {
    RCLCPP_INFO(this->get_logger(), "Particle Filter node started.");

    // Subscribe to odometry, IMU, and joint_states
    odom_sub_.subscribe(this, "/odom");
    imu_sub_.subscribe(this, "/imu");
    joint_sub_.subscribe(this, "/joint_states");

    // Synchronize the three topics using approximate time
    triple_sync_ = std::make_shared<message_filters::Synchronizer<TripleSyncPolicy>>(
        TripleSyncPolicy(10), odom_sub_, imu_sub_, joint_sub_);

    triple_sync_->registerCallback(std::bind(
        &ParticleFilterNode::synced_callback, this,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3));

    // Publisher for the estimated pose
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/pf_pose", 10);
}

// Callback triggered when /odom, /imu, and /joint_states arrive in sync
void ParticleFilterNode::synced_callback(
    const nav_msgs::msg::Odometry::ConstSharedPtr& odom,
    const sensor_msgs::msg::Imu::ConstSharedPtr& imu,
    const sensor_msgs::msg::JointState::ConstSharedPtr& joint) {

    rclcpp::Time current_time = this->now();
    double dt;

    // Skip first message to establish time baseline
    if (!first_msg_received_) {
        first_msg_received_ = true;
        last_time_ = current_time;
        return;
    }

    dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

    // Compute linear velocity from joint encoder data (wheel radii assumed as 0.033)
    double v_left = 0, v_right = 0;
    for (size_t i = 0; i < joint->name.size(); ++i) {
        if (joint->name[i] == "wheel_left_joint") {
            v_left = joint->velocity[i] * 0.033;
        } else if (joint->name[i] == "wheel_right_joint") {
            v_right = joint->velocity[i] * 0.033;
        }
    }

    double v = (v_left + v_right) / 2.0;     // Linear velocity
    double w = imu->angular_velocity.z;      // Angular velocity from IMU

    // Particle Filter steps
    pf_.predict(v, w, dt);         // Motion update
    pf_.correct(w, w);             // Measurement update (simple model using w)
    pf_.resample();                // Resampling step
    auto pose = pf_.estimate_pose(); // Estimate robot pose

    // Publish pose
    pose.header.stamp = current_time;
    pose.header.frame_id = "map";
    pose_pub_->publish(pose);
}

// Main function: initializes ROS and runs the node
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ParticleFilterNode>());
    rclcpp::shutdown();
    return 0;
}

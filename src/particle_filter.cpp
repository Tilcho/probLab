#include "localization_filters/particle_filter.hpp"

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

void ParticleFilter::predict(double v, double w, double dt) {
    for (auto& p : particles_) {
        p.x += v * std::cos(p.theta) * dt + noise_x_(rng_);
        p.y += v * std::sin(p.theta) * dt + noise_y_(rng_);
        p.theta += w * dt + noise_theta_(rng_);
    }
}

void ParticleFilter::resample() {
    std::vector<Particle> new_particles;
    std::vector<double> weights;

    // Extract weights
    for (const auto& p : particles_) {
        weights.push_back(p.weight);
    }

    std::discrete_distribution<> dist(weights.begin(), weights.end());

    for (size_t i = 0; i < particles_.size(); ++i) {
        new_particles.push_back(particles_[dist(rng_)]);
    }

    particles_ = std::move(new_particles);
}

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
// Node Implementation
// ======================

ParticleFilterNode::ParticleFilterNode() : Node("particle_filter_node") {
    RCLCPP_INFO(this->get_logger(), "Particle Filter node started.");

    odom_sub_.subscribe(this, "/odom");
    imu_sub_.subscribe(this, "/imu");
    joint_sub_.subscribe(this, "/joint_states");

    triple_sync_ = std::make_shared<message_filters::Synchronizer<TripleSyncPolicy>>(
        TripleSyncPolicy(10), odom_sub_, imu_sub_, joint_sub_);

    triple_sync_->registerCallback(std::bind(
        &ParticleFilterNode::synced_callback, this,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3));

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/pf_pose", 10);
}

void ParticleFilterNode::synced_callback(
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

    // Compute velocity from wheel encoders
    double v_left = 0, v_right = 0;
    for (size_t i = 0; i < joint->name.size(); ++i) {
        if (joint->name[i] == "wheel_left_joint") {
            v_left = joint->velocity[i] * 0.033;
        } else if (joint->name[i] == "wheel_right_joint") {
            v_right = joint->velocity[i] * 0.033;
        }
    }

    double v = (v_left + v_right) / 2.0;
    double w = imu->angular_velocity.z;

    pf_.predict(v, w, dt);
    pf_.resample();
    auto pose = pf_.estimate_pose();

    pose.header.stamp = current_time;
    pose.header.frame_id = "map";
    pose_pub_->publish(pose);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ParticleFilterNode>());
    rclcpp::shutdown();
    return 0;
}

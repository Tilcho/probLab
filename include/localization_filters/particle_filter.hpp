#ifndef LOCALIZATION_FILTERS_PARTICLE_FILTER_HPP
#define LOCALIZATION_FILTERS_PARTICLE_FILTER_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include <Eigen/Dense>
#include <vector>
#include <random>

struct Particle {
    double x;
    double y;
    double theta;
    double weight;
};

class ParticleFilter {
public:
    ParticleFilter(size_t num_particles = 100);
    void predict(double v, double w, double dt);
    void resample();
    geometry_msgs::msg::PoseWithCovarianceStamped estimate_pose();

private:
    std::vector<Particle> particles_;
    std::default_random_engine rng_;
    std::normal_distribution<double> noise_x_, noise_y_, noise_theta_;
};

class ParticleFilterNode : public rclcpp::Node {
public:
    ParticleFilterNode();

private:
    void synced_callback(
        const nav_msgs::msg::Odometry::ConstSharedPtr& odom,
        const sensor_msgs::msg::Imu::ConstSharedPtr& imu,
        const sensor_msgs::msg::JointState::ConstSharedPtr& joint);

    ParticleFilter pf_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;

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
};

#endif  // LOCALIZATION_FILTERS_PARTICLE_FILTER_HPP

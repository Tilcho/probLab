#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <fstream>
#include <iomanip>
#include <cmath>

class PoseLogger : public rclcpp::Node {
public:
    PoseLogger() : Node("pose_logger_node") {
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&PoseLogger::odomCallback, this, std::placeholders::_1));

        sub_kf_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/kf_pose", 10,
            std::bind(&PoseLogger::kfCallback, this, std::placeholders::_1));

        sub_ekf_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/ekf_pose", 10,
            std::bind(&PoseLogger::ekfCallback, this, std::placeholders::_1));

        sub_pf_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/pf_pose", 10,
            std::bind(&PoseLogger::pfCallback, this, std::placeholders::_1));

        log_.open("comparison.csv", std::ios::out | std::ios::trunc);
        log_ << "time,odom_x,odom_y,"
             << "kf_x,kf_y,kf_cov_x,kf_cov_y,kf_cov_yaw,"
             << "ekf_x,ekf_y,ekf_cov_x,ekf_cov_y,ekf_cov_yaw,"
             << "pf_x,pf_y,pf_cov_x,pf_cov_y,pf_cov_yaw,"
             << "kf_rmse,ekf_rmse,pf_rmse\n";
    }

    ~PoseLogger() {
        if (log_.is_open()) log_.close();
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        odom_x_ = msg->pose.pose.position.x;
        odom_y_ = msg->pose.pose.position.y;
        time_ = rclcpp::Time(msg->header.stamp).seconds();
        odom_received_ = true;
        tryLog();
    }

    void kfCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        kf_x_ = msg->pose.pose.position.x;
        kf_y_ = msg->pose.pose.position.y;
        kf_cov_x_ = msg->pose.covariance[0];
        kf_cov_y_ = msg->pose.covariance[7];
        kf_cov_yaw_ = msg->pose.covariance[35];
        kf_received_ = true;
        tryLog();
    }

    void ekfCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        ekf_x_ = msg->pose.pose.position.x;
        ekf_y_ = msg->pose.pose.position.y;
        ekf_cov_x_ = msg->pose.covariance[0];
        ekf_cov_y_ = msg->pose.covariance[7];
        ekf_cov_yaw_ = msg->pose.covariance[35];
        ekf_received_ = true;
        tryLog();
    }

    void pfCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        pf_x_ = msg->pose.pose.position.x;
        pf_y_ = msg->pose.pose.position.y;
        pf_cov_x_ = msg->pose.covariance[0];
        pf_cov_y_ = msg->pose.covariance[7];
        pf_cov_yaw_ = msg->pose.covariance[35];
        pf_received_ = true;
        tryLog();
    }

    void tryLog() {
        if (odom_received_ && kf_received_ && ekf_received_ && pf_received_) {
            ++sample_count_;

            // Compute RMSE
            kf_error_sq_ += std::pow(kf_x_ - odom_x_, 2) + std::pow(kf_y_ - odom_y_, 2);
            ekf_error_sq_ += std::pow(ekf_x_ - odom_x_, 2) + std::pow(ekf_y_ - odom_y_, 2);
            pf_error_sq_ += std::pow(pf_x_ - odom_x_, 2) + std::pow(pf_y_ - odom_y_, 2);

            double kf_rmse = std::sqrt(kf_error_sq_ / sample_count_);
            double ekf_rmse = std::sqrt(ekf_error_sq_ / sample_count_);
            double pf_rmse = std::sqrt(pf_error_sq_ / sample_count_);

            log_ << std::fixed << std::setprecision(6)
                 << time_ << ","
                 << odom_x_ << "," << odom_y_ << ","
                 << kf_x_ << "," << kf_y_ << "," << kf_cov_x_ << "," << kf_cov_y_ << "," << kf_cov_yaw_ << ","
                 << ekf_x_ << "," << ekf_y_ << "," << ekf_cov_x_ << "," << ekf_cov_y_ << "," << ekf_cov_yaw_ << ","
                 << pf_x_ << "," << pf_y_ << "," << pf_cov_x_ << "," << pf_cov_y_ << "," << pf_cov_yaw_ << ","
                 << kf_rmse << "," << ekf_rmse << "," << pf_rmse << "\n";
            log_.flush();

            odom_received_ = kf_received_ = ekf_received_ = pf_received_ = false;
        }
    }

    std::ofstream log_;
    double time_ = 0.0;

    // Pose values
    double odom_x_ = 0.0, odom_y_ = 0.0;
    double kf_x_ = 0.0, kf_y_ = 0.0, kf_cov_x_ = 0.0, kf_cov_y_ = 0.0, kf_cov_yaw_ = 0.0;
    double ekf_x_ = 0.0, ekf_y_ = 0.0, ekf_cov_x_ = 0.0, ekf_cov_y_ = 0.0, ekf_cov_yaw_ = 0.0;
    double pf_x_ = 0.0, pf_y_ = 0.0, pf_cov_x_ = 0.0, pf_cov_y_ = 0.0, pf_cov_yaw_ = 0.0;

    // RMSE running error accumulation
    double kf_error_sq_ = 0.0, ekf_error_sq_ = 0.0, pf_error_sq_ = 0.0;
    size_t sample_count_ = 0;

    bool odom_received_ = false, kf_received_ = false;
    bool ekf_received_ = false, pf_received_ = false;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_kf_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_ekf_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_pf_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseLogger>());
    rclcpp::shutdown();
    return 0;
}

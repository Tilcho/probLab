#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <fstream>
#include <iomanip>

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

        log_.open("pose_comparison.csv", std::ios::out | std::ios::trunc);
        log_ << "time,odom_x,kf_x,ekf_x,pf_x\n";
    }

    ~PoseLogger() {
        if (log_.is_open()) log_.close();
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        odom_x_ = msg->pose.pose.position.x;
        time_ = rclcpp::Time(msg->header.stamp).seconds();
        odom_received_ = true;
        tryLog();
    }

    void kfCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        kf_x_ = msg->pose.pose.position.x;
        kf_received_ = true;
        tryLog();
    }

    void ekfCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        ekf_x_ = msg->pose.pose.position.x;
        ekf_received_ = true;
        tryLog();
    }

    void pfCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        pf_x_ = msg->pose.pose.position.x;
        pf_received_ = true;
        tryLog();
    }

    void tryLog() {
        if (odom_received_ && kf_received_ && ekf_received_ && pf_received_) {
            log_ << std::fixed << std::setprecision(6)
                 << time_ << "," << odom_x_ << "," << kf_x_ << "," << ekf_x_ << "," << pf_x_ << "\n";
            log_.flush();  // Optional: flush after each write

            odom_received_ = kf_received_ = ekf_received_ = pf_received_ = false;
        }
    }

    std::ofstream log_;
    double time_ = 0.0;
    double odom_x_ = 0.0, kf_x_ = 0.0, ekf_x_ = 0.0, pf_x_ = 0.0;
    bool odom_received_ = false, kf_received_ = false, ekf_received_ = false, pf_received_ = false;

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

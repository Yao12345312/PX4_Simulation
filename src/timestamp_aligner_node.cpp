#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"

class ScanTimestampSyncNode : public rclcpp::Node {
public:
    ScanTimestampSyncNode() : Node("scan_timestamp_sync_node") {
        // 订阅 IMU 数据
imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/mavros/imu/data", 
    rclcpp::QoS(10).best_effort(),  // 改为 BEST_EFFORT，匹配 MAVROS
    std::bind(&ScanTimestampSyncNode::imu_callback, this, std::placeholders::_1)
);



        // 订阅 /scan，使用BEST_EFFORT QoS
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)),
            std::bind(&ScanTimestampSyncNode::scan_callback, this, std::placeholders::_1)
        );

        // 发布同步后的 /scan
        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "/synced_scan", rclcpp::QoS(10)
        );
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg) {
        last_imu_time_ = imu_msg->header.stamp;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
        // 确保已经接收到 IMU 数据
        if (last_imu_time_.nanoseconds() == 0) {
            RCLCPP_WARN(this->get_logger(), "Waiting for first IMU timestamp...");
            return;
        }

        auto synced_scan = *scan_msg;  // 复制激光雷达数据
        synced_scan.header.stamp = last_imu_time_;  // 替换时间戳

        RCLCPP_INFO(this->get_logger(), "Publishing synced scan with timestamp: %.9f", 
                    last_imu_time_.seconds());

        scan_pub_->publish(synced_scan);
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;

    rclcpp::Time last_imu_time_{0, 0, RCL_ROS_TIME};  // 初始化时间戳
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanTimestampSyncNode>());
    rclcpp::shutdown();
    return 0;
}


#ifndef LIDAR_FILTER__HPP
#define LIDAR_FILTER__HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LidarFilterNode : public rclcpp::Node
{
public:
    LidarFilterNode();

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;

    // Filtering parameters
    double min_range_;   // minimum range to keep
    double max_range_;   // maximum range to keep
    double fov_min_;     // minimum angle (radians)
    double fov_max_;     // maximum angle (radians)
};

#endif // LIDAR_FILTER__HPP

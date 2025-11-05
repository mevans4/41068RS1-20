#include "lidar_filter.h"

LidarFilterNode::LidarFilterNode()
: Node("lidar_filter_node")
{
    // Declare parameters with defaults
    this->declare_parameter<double>("min_range", 0.2);
    this->declare_parameter<double>("max_range", 10.0);
    this->declare_parameter<double>("fov_min", -1.57);  // -90 degrees
    this->declare_parameter<double>("fov_max", 1.57);   // +90 degrees

    // Get parameters
    this->get_parameter("min_range", min_range_);
    this->get_parameter("max_range", max_range_);
    this->get_parameter("fov_min", fov_min_);
    this->get_parameter("fov_max", fov_max_);

    // Subscriber to raw LIDAR scan
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan_raw", 10,   // <-- subscribe to raw data
        std::bind(&LidarFilterNode::scan_callback, this, std::placeholders::_1));

    // Publisher for filtered scan directly on /scan
    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);  // <-- output on /scan

    RCLCPP_INFO(this->get_logger(), "LidarFilterNode initialized with min_range=%.2f, max_range=%.2f, fov=[%.2f, %.2f]",
                min_range_, max_range_, fov_min_, fov_max_);
}

void LidarFilterNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    auto filtered_scan = *msg; // copy original message

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        double angle = msg->angle_min + i * msg->angle_increment;
        double range = msg->ranges[i];

        // Filter by angle (FOV)
        if (angle < fov_min_ || angle > fov_max_) {
            filtered_scan.ranges[i] = std::numeric_limits<float>::quiet_NaN();
            continue;
        }

        // Filter by range
        if (range < min_range_ || range > max_range_ || std::isnan(range)) {
            filtered_scan.ranges[i] = std::numeric_limits<float>::quiet_NaN();
        }
    }

    // Keep header info consistent
    filtered_scan.header.stamp = msg->header.stamp;
    filtered_scan.header.frame_id = msg->header.frame_id;

    scan_pub_->publish(filtered_scan);
}

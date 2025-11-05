#include "rclcpp/rclcpp.hpp"
#include "lidar_filter.h"

int main(int argc, char **argv)
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create the LidarFilterNode
    auto node = std::make_shared<LidarFilterNode>();

    // Spin the node until shutdown
    rclcpp::spin(node);

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}

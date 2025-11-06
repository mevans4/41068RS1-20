#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "path_planner_cpp/srv/path_planner.hpp"

class PathPlannerServiceNode : public rclcpp::Node
{
public:
  PathPlannerServiceNode() : Node("path_planner_service_node")
  {
    // Subscribe to the /path topic from planner_node
    path_subscription_ = this->create_subscription<nav_msgs::msg::Path>(
        "/path", 10,
        std::bind(&PathPlannerServiceNode::path_callback, this, std::placeholders::_1));

    // Create service to provide the stored path
    service_ = this->create_service<path_planner_cpp::srv::PathPlanner>(
        "path_planner_service",
        std::bind(&PathPlannerServiceNode::handle_path_request, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Path Planner Service Node ready!");
    RCLCPP_INFO(this->get_logger(), "Subscribed to: /path");
    RCLCPP_INFO(this->get_logger(), "Providing service: /path_planner_service");
  }

private:
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
  rclcpp::Service<path_planner_cpp::srv::PathPlanner>::SharedPtr service_;
  nav_msgs::msg::Path stored_path_;
  bool path_available_ = false;

  void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    stored_path_ = *msg;
    path_available_ = true;
    RCLCPP_INFO(this->get_logger(), "Received new path with %zu waypoints",
                stored_path_.poses.size());
  }

  void handle_path_request(
      const std::shared_ptr<path_planner_cpp::srv::PathPlanner::Request> request,
      std::shared_ptr<path_planner_cpp::srv::PathPlanner::Response> response)
  {
    (void)request; // Unused

    if (!path_available_) {
      RCLCPP_WARN(this->get_logger(), "No path available yet! Plan a path first.");
      response->success = false;
      response->message = "No path planned yet. Please use the UI to plan a path first.";
      return;
    }

    // Convert nav_msgs::msg::Path to vector of geometry_msgs::msg::Point
    response->path.clear();
    for (const auto& pose_stamped : stored_path_.poses) {
      geometry_msgs::msg::Point point;
      point.x = pose_stamped.pose.position.x;
      point.y = pose_stamped.pose.position.y;
      point.z = pose_stamped.pose.position.z;
      response->path.push_back(point);
    }

    response->success = true;
    response->message = "Path with " + std::to_string(response->path.size()) +
                       " waypoints provided.";

    RCLCPP_INFO(this->get_logger(), "Provided path with %zu waypoints",
                response->path.size());
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PathPlannerServiceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

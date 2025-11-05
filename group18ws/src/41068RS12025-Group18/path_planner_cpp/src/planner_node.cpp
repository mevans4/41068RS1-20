#include <chrono>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/trigger.hpp"   // <-- Service to trigger planning

using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
using GoalHandleComputePathToPose = rclcpp_action::ClientGoalHandle<ComputePathToPose>;

class PlannerNode : public rclcpp::Node
{
public:
  PlannerNode() : Node("planner_node")
  {
    publisher_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

    // Create the Nav2 planner action client
    action_client_ = rclcpp_action::create_client<ComputePathToPose>(
        this, "/compute_path_to_pose");

    // Create a service to trigger the planning
    service_ = this->create_service<std_srvs::srv::Trigger>(
        "trigger_path_plan",
        std::bind(&PlannerNode::handle_trigger, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(),
                "PlannerNode ready. Call the service /trigger_path_plan to start planning.");
  }

private:
  rclcpp_action::Client<ComputePathToPose>::SharedPtr action_client_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;

  void handle_trigger(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request; // Unused
    RCLCPP_INFO(this->get_logger(), "Service called: computing path...");

    if (!action_client_->wait_for_action_server(std::chrono::seconds(2))) {
      RCLCPP_ERROR(this->get_logger(), "Planner action server not available!");
      response->success = false;
      response->message = "Planner server unavailable.";
      return;
    }

    send_goal();

    response->success = true;
    response->message = "Path computation started.";
  }

  void send_goal()
  {
    auto goal_msg = ComputePathToPose::Goal();

    // Set only the goal pose (Nav2 will use the robot’s current position as start)
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = this->now();
    goal.pose.position.x = 0.0;
    goal.pose.position.y = 0.0;
    goal.pose.position.z = 0.0;
    goal.pose.orientation.w = 1.0;
    goal_msg.goal = goal;

    // Send goal asynchronously
    auto send_goal_options =
        rclcpp_action::Client<ComputePathToPose>::SendGoalOptions();
    send_goal_options.result_callback =
        [this](const GoalHandleComputePathToPose::WrappedResult &result) {
          if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(),
                        "Path successfully computed, publishing to /path");
            publisher_->publish(result.result->path);
          } else {
            RCLCPP_ERROR(this->get_logger(), "Path computation failed!");
          }
        };

    action_client_->async_send_goal(goal_msg, send_goal_options);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}



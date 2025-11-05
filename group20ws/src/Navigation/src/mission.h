#ifndef MISSION_H
#define MISSION_H

#include <vector>
#include "rclcpp/rclcpp.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "std_srvs/srv/set_bool.hpp"



using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class Mission : public rclcpp::Node
{
public:
    Mission();

private:
    // Add a new goal (position)
    void setGoal(const geometry_msgs::msg::Point & msg);
    void serviceCall(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                 std::shared_ptr<std_srvs::srv::SetBool::Response> resp);
    void cancel_active_goal();
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr goalPoint;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr missionService;
    std::shared_ptr<GoalHandleNavigateToPose> current_goal_handle_;
    std::atomic_bool active_{false};
    NavigateToPose::Goal goal;
    bool has_goal_;


};

#endif // MISSION_H
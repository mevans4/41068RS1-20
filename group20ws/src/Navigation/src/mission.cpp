#include "mission.h"

Mission::Mission() : Node("navigation")
{

    client_ = rclcpp_action::create_client<NavigateToPose>(this,"navigate_to_pose");
    // subscribe to if user inputs manual goals
    goalPoint = this->create_subscription<geometry_msgs::msg::Point>("/userGoal", 10, std::bind(&Mission::setGoal, this, std::placeholders::_1));

    missionService = this->create_service<std_srvs::srv::SetBool>("/drone/mission",std::bind(&Mission::serviceCall, this, std::placeholders::_1, std::placeholders::_2));

    has_goal_ = false;

}


void Mission::setGoal(const geometry_msgs::msg::Point & msg)
{
  geometry_msgs::msg::PoseStamped ps;
  ps.header.frame_id = "map";
  ps.header.stamp = this->now();
  ps.pose.position.x = msg.x;
  ps.pose.position.y = msg.y;

  goal.pose = ps;
  has_goal_ = true;
  RCLCPP_INFO(get_logger(), "Goal Set: x=%.2f, y=%.2f", msg.x, msg.y);
}

void Mission::serviceCall(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                        std::shared_ptr<std_srvs::srv::SetBool::Response> resp)
{
 if (req->data) {
    // START
    if (!has_goal_) {
      resp->success = false;
      resp->message = "No goal set. Set goal first.";
      return;
    }
    // Ensure Nav2 is up now (not in the sub callback)
    if (!client_->wait_for_action_server(std::chrono::milliseconds(500))) {
      resp->success = false;
      resp->message = "navigate_to_pose not available";
      RCLCPP_WARN(get_logger(), "%s", resp->message.c_str());
      return;
    }

    try {
      auto fut = client_->async_cancel_all_goals();
      (void)fut.wait_for(std::chrono::milliseconds(800));  // short grace; don't block long
    } catch (const std::exception &e) {
      RCLCPP_WARN(get_logger(), "async_cancel_all_goals threw: %s", e.what());
    }

    RCLCPP_INFO(get_logger(), "Navigating to goal: x=%.2f, y=%.2f",
                goal.pose.pose.position.x, goal.pose.pose.position.y);

    auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    opts.goal_response_callback = [this](GoalHandleNavigateToPose::SharedPtr handle){
      if (!handle) {
        RCLCPP_WARN(this->get_logger(), "Goal rejected by Nav2.");
      } else {
        current_goal_handle_ = handle;        // store handle for cancel
        active_ = true;                       // mark running
        RCLCPP_INFO(this->get_logger(), "Goal accepted by Nav2.");
      }
    };

    opts.result_callback = [this](const GoalHandleNavigateToPose::WrappedResult & result){
      RCLCPP_INFO(this->get_logger(), "Navigation finished with code: %d",
                  static_cast<int>(result.code));
      current_goal_handle_.reset();
      active_ = false;                        
    };

    client_->async_send_goal(goal, opts);
    resp->success = true;
    resp->message = "Goal sent";
    has_goal_ = false;
    return;
  }

  // STOP
  if (!active_.exchange(false)) {
    resp->success = true;
    resp->message = "Mission already stopped";
  } else {
    RCLCPP_INFO(this->get_logger(), "Mission STOP requested. Cancelling active goal...");
    cancel_active_goal();                      // uses current_goal_handle_
    resp->success = true;
    resp->message = "Mission stopped";
  }
}

void Mission::cancel_active_goal()
{
  if (!current_goal_handle_) return;

  auto gh = current_goal_handle_;
  current_goal_handle_.reset();  // avoid races
  try {
    client_->async_cancel_goal(gh, [this](auto /*result*/) {
      RCLCPP_INFO(this->get_logger(), "Cancel sent to Nav2.");
    });
  } catch (const std::exception &e) {
    RCLCPP_WARN(this->get_logger(), "Cancel failed: %s", e.what());
  }
}

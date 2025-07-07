#include "wormhole_navigation/nav_control.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

WormholeNav::WormholeNav() : Node("wormhole_navigator"){

    intial_pose_ = this->create_publisher<pose_msg_>("/initial_pose", 10);
    get_pose_ = this->create_client<pose_loader_srv_>("/wormhole/load_pose");
    load_map_ = this->create_client<load_map_srv_>("/map_server/load_map");

    nav_command_ = rclcpp_action::create_client<nav_to_pose_action_>(this, "/navigate_to_pose");
    control_server_ = rclcpp_action::create_server<worm_goal_action_>(
                        this,
                        "/wormhole/navigate",
                        std::bind(&WormholeNav::wnGoalCallback, this, _1, _2),
                        std::bind(&WormholeNav::wnCancelCallback, this, _1),
                        std::bind(&WormholeNav::wnAcceptedCallback, this, _1)
    );

    RCLCPP_INFO(this->get_logger(),"Wormhole Navigator is up and running... - wormhole_navigator");
}

rclcpp_action::GoalResponse WormholeNav::wnGoalCallback(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const worm_goal_action_::Goal> goal){

    (void)uuid;

    if(goal->map_id>3 || goal->map_id<1){

        RCLCPP_WARN(this->get_logger(),"Invalid map id");
        return rclcpp_action::GoalResponse::REJECT;
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (control_handle_) {
            if (control_handle_->is_active()) {
                    RCLCPP_WARN(this->get_logger(), "Rejecting new goal, as previous goal is still being executed");
                    return rclcpp_action::GoalResponse::REJECT;
                }
            }
    }

    RCLCPP_INFO(this->get_logger(), "Accepting the goal");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

}

rclcpp_action::CancelResponse WormholeNav::wnCancelCallback(const std::shared_ptr<worm_goal_handle_> goal_handle){

    direction_map_.clear();
    nav_command_->async_cancel_all_goals();
    return rclcpp_action::CancelResponse::ACCEPT;
}

void WormholeNav::wnAcceptedCallback(const std::shared_ptr<worm_goal_handle_> goal_handle){

    wnExecuteCallback(goal_handle);
}

void WormholeNav::wnExecuteCallback(const std::shared_ptr<worm_goal_handle_> goal_handle){
    
    if(goal_handle->get_goal()->map_id!=current_map_){
        calculate_traverse_order(current_map_, goal_handle->get_goal()->map_id);
    }
    else{
        
    }
}

void WormholeNav::calculate_traverse_order(int start, int end){

    std::vector<int> path;

    // Handling the traversal logic
    if (start == 1 && end == 3) {
        path = {1, 2, 3};
    } else if (start == 3 && end == 1) {
        path = {3, 2, 1};
    } else if (start == 2 && end == 1) {
        path = {2, 1};
    } else if (start == 2 && end == 3) {
        path = {2, 3};
    } else if (start == 1 && end == 2) {
        path = {1, 2};
    } else if (start == 3 && end == 2) {
        path = {3, 2};
    }

    direction_map_.clear();
    direction_map_.assign(path.begin(), path.end());
}
#include "wormhole_navigation/nav_control.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

WormholeNav::WormholeNav() : Node("wormhole_navigator"){

    intial_pose_ = this->create_publisher<pose_cov_>("/initial_pose", 10);
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

void WormholeNav::init(){
    pose_fetcher_ = std::make_shared<DBPoseFetcher>(shared_from_this());
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

    (void)goal_handle;

    direction_map_.clear();
    nav_command_->async_cancel_all_goals();
    return rclcpp_action::CancelResponse::ACCEPT;
}

void WormholeNav::wnAcceptedCallback(const std::shared_ptr<worm_goal_handle_> goal_handle){

    RCLCPP_INFO(this->get_logger(), "Reached wormhole - accepted callback");
    control_handle_ = goal_handle;
    wnExecuteCallback(goal_handle);
}

void WormholeNav::wnExecuteCallback(const std::shared_ptr<worm_goal_handle_> goal_handle){

    RCLCPP_INFO(this->get_logger(), "Reached wormhole - execute callback");
    goal_pose_ = goal_handle->get_goal()->goal_pose;
    
    if(goal_handle->get_goal()->map_id!=current_map_){
        RCLCPP_INFO(this->get_logger(), "Current map no: %d goal_map_no: %d", current_map_, goal_handle->get_goal()->map_id);
        calculateTraverseOrder(current_map_, goal_handle->get_goal()->map_id);
        sendNavGoal();
    }
    else{
        sendNavGoal();
    }
}

void WormholeNav::sendNavGoal(){

    RCLCPP_INFO(this->get_logger(), "Reached wormhole - sendNavGoal");
    
    auto nav_goal_ = nav_to_pose_action_::Goal();
    
    if(direction_map_.size() == 1){
        nav_goal_.pose = goal_pose_;
    }
    else {
        RCLCPP_INFO(this->get_logger(), "Entered else block of - sendNavGoal");
    
        if (direction_map_.size() < 2) {
            RCLCPP_FATAL(this->get_logger(), "direction_map_ has fewer than 2 elements. Cannot proceed.");
            return;
        }
    
        std::set<std::pair<int, int>> use_method2 = {
            {2, 3},
            {3, 2}
        };
    
        std::pair<int, int> move = {direction_map_[0], direction_map_[1]};
        RCLCPP_INFO(this->get_logger(), "Constructed move pair: (%d, %d)", move.first, move.second);
    
        if (!pose_fetcher_) {
            RCLCPP_FATAL(this->get_logger(), "pose_fetcher_ is nullptr!");
            return;
        }
    
        if (use_method2.count(move)) {
            map_pose_ = pose_fetcher_->get_pose_from_db(1);
        } else {
            map_pose_ = pose_fetcher_->get_pose_from_db(0);
        }
    
        RCLCPP_INFO(this->get_logger(), "Fetched data from DB - sendNavGoal");
    
        nav_goal_.pose.pose = map_pose_.pose.pose;
    }
    

    auto nav_goal_options_ = rclcpp_action::Client<nav_to_pose_action_>::SendGoalOptions();
    nav_goal_options_.goal_response_callback = std::bind(&WormholeNav::clientResponseCallback, this, _1);
    nav_goal_options_.result_callback = std::bind(&WormholeNav::clientResultCallback, this, _1);
    nav_goal_options_.feedback_callback = std::bind(&WormholeNav::clientFeedbackCallback, this, _1, _2);

    RCLCPP_INFO(this->get_logger(), "Sending goal to navigate to pose");
    nav_command_->async_send_goal(nav_goal_, nav_goal_options_);

    direction_map_.erase(direction_map_.begin());
}

void WormholeNav::clientResponseCallback(const rclcpp_action::ClientGoalHandle<nav_to_pose_action_>::SharedPtr &goal_handle){
    (void)goal_handle;
}

void WormholeNav::clientResultCallback(const rclcpp_action::ClientGoalHandle<nav_to_pose_action_>::WrappedResult &result){

    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {

        RCLCPP_INFO(this->get_logger(), "Navigate to Pose - success");
        if(!direction_map_.empty()){
            handleServices();
        }
        else{
            worm_result_.msg = "Success";
            auto result_ptr = std::make_shared<worm_goal_action_::Result>(worm_result_);
            control_handle_->succeed(result_ptr); 
        }
    }
    else if(result.code == rclcpp_action::ResultCode::ABORTED){

        worm_result_.msg = "Navigation aborted";
        auto result_ptr = std::make_shared<worm_goal_action_::Result>(worm_result_);
        control_handle_->abort(result_ptr); 
    }
    else if(result.code == rclcpp_action::ResultCode::CANCELED){
        worm_result_.msg = "Cancelled";
        auto result_ptr = std::make_shared<worm_goal_action_::Result>(worm_result_);
        control_handle_->canceled(result_ptr);
    }
}

void WormholeNav::clientFeedbackCallback(const rclcpp_action::ClientGoalHandle<nav_to_pose_action_>::SharedPtr &goal_handle,
    const std::shared_ptr<const nav_to_pose_action_::Feedback> feedback){
    
    (void)goal_handle;
    worm_feedback->current_pose = feedback->current_pose;
    control_handle_->publish_feedback(worm_feedback);
}

void WormholeNav::handleServices(){

    std::string pkg_name = "turtlebot3_spawn";

    // Construct full map path: <package_share>/maps/map_3.yaml
    std::string map_filename = "map_" + std::to_string(direction_map_.at(0)) + ".yaml";
    std::string full_path = ament_index_cpp::get_package_share_directory(pkg_name) + "/maps/" + map_filename;;
    
    auto map_change_request_ = load_map_srv_::Request();
    map_change_request_.map_url = full_path;

    intial_pose_->publish(map_pose_);

    sendNavGoal();
}

void WormholeNav::calculateTraverseOrder(int start, int end){

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

    RCLCPP_INFO(this->get_logger(), "Size of direction map after populating is: %ld", direction_map_.size());

    for(size_t i =0; i<direction_map_.size(); i++){
        RCLCPP_INFO(this->get_logger(), "Direction map value - %d", direction_map_[i]);
    }
}
#include <mutex>
#include <memory>
#include <set>
#include <algorithm>

#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/srv/load_map.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "wormhole_nav_msg/action/worm_goal.hpp"
#include "wormhole_nav_msg/srv/pose_loader.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "wormhole_navigation/data_fetcher.hpp"

/**
 * @class WormholeNav
 * @brief Main class where wormhole based navigation functionality takes place.
 */
class WormholeNav : public rclcpp::Node{

/**
 * @brief Creating shorter aliases for ROS msg, actions and services.
 */
using worm_goal_action_ = wormhole_nav_msg::action::WormGoal;
using worm_goal_handle_ = rclcpp_action::ServerGoalHandle<worm_goal_action_>;
using nav_to_pose_action_ = nav2_msgs::action::NavigateToPose;
using nav_to_pose_handle_ = rclcpp_action::ClientGoalHandle<nav_to_pose_action_>;

using load_map_srv_ = nav2_msgs::srv::LoadMap;
using pose_loader_srv_ = wormhole_nav_msg::srv::PoseLoader;
using pose_stamped_ = geometry_msgs::msg::PoseStamped;

public:
    WormholeNav();
    ~WormholeNav() = default;
    void init();

private:

    rclcpp_action::GoalResponse wnGoalCallback(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const worm_goal_action_::Goal> goal);
    rclcpp_action::CancelResponse wnCancelCallback(const std::shared_ptr<worm_goal_handle_> goal_handle);
    void wnAcceptedCallback(const std::shared_ptr<worm_goal_handle_> goal_handle);
    void wnExecuteCallback(const std::shared_ptr<worm_goal_handle_> goal_handle);

    void clientResultCallback(const rclcpp_action::ClientGoalHandle<nav_to_pose_action_>::WrappedResult &result);
    void clientResponseCallback(const rclcpp_action::ClientGoalHandle<nav_to_pose_action_>::SharedPtr &goal_handle);
    void clientFeedbackCallback(const rclcpp_action::ClientGoalHandle<nav_to_pose_action_>::SharedPtr &goal_handle,
        const std::shared_ptr<const nav_to_pose_action_::Feedback> feedback);

    void calculateTraverseOrder(int start, int end); 
    void sendNavGoal();
    void handleServices();

    std::mutex mutex_;
    std::shared_ptr<worm_goal_handle_> control_handle_{nullptr};
    worm_goal_action_::Result worm_result_;
    std::shared_ptr<worm_goal_action_::Feedback> worm_feedback{std::make_shared<worm_goal_action_::Feedback>()};

    rclcpp_action::Server<worm_goal_action_>::SharedPtr control_server_;
    rclcpp_action::Client<nav_to_pose_action_>::SharedPtr nav_command_;

    rclcpp::Client<load_map_srv_>::SharedPtr load_map_;
    rclcpp::Client<pose_loader_srv_>::SharedPtr get_pose_;
    rclcpp::Publisher<pose_cov_>::SharedPtr intial_pose_;

    int current_map_{1}, controller_result_{0};
    std::vector<int> direction_map_;
    pose_stamped_ goal_pose_;
    pose_cov_ map_pose_;

    std::shared_ptr<DBPoseFetcher> pose_fetcher_;
};

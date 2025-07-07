#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/srv/load_map.hpp"
#include "wormhole_nav_msg/action/worm_goal.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

/**
 * @class WormholeNav
 * @brief Main class where wormhole based navigation functionality takes place.
 */
class WormholeNav : public rclcpp::Node{

using worm_goal_action_ = wormhole_nav_msg::action::WormGoal;
using worm_goal_handle_ = rclcpp_action::ServerGoalHandle<worm_goal_action_>;
using nav_to_pose_action_ = nav2_msgs::action::NavigateToPose;
using nav_to_pose_handle_ = rclcpp_action::ClientGoalHandle<nav_to_pose_action_>;

using load_map_srv_ = nav2_msgs::srv::LoadMap;
using pose_msg_ = geometry_msgs::msg::PoseWithCovarianceStamped;

public:
    WormholeNav();
    ~WormholeNav() = default;

private:

    rclcpp_action::GoalResponse wnGoalCallback(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const worm_goal_action_::Goal> goal);
    rclcpp_action::CancelResponse wnCancelCallback(const std::shared_ptr<worm_goal_handle_> goal_handle);
    void wnAcceptedCallback(const std::shared_ptr<worm_goal_handle_> goal_handle);
    void wnExecuteCallback(const std::shared_ptr<worm_goal_handle_> goal_handle);

    void clientResultCallback(const rclcpp_action::ClientGoalHandle<nav_to_pose_action_>::WrappedResult &result);
    void clientResponseCallback(const rclcpp_action::ClientGoalHandle<nav_to_pose_action_>::SharedPtr &goal_handle);
    void clientFeedbackCallback(const rclcpp_action::ClientGoalHandle<nav_to_pose_action_>::SharedPtr &goal_handle,
        const std::shared_ptr<const nav_to_pose_action_::Feedback> feedback);

    std::shared_ptr<worm_goal_handle_> control_handle_;
    rclcpp_action::Server<worm_goal_action_>::SharedPtr control_server_;
    rclcpp_action::Client<nav_to_pose_action_>::SharedPtr nav_command_;

    rclcpp::Client<load_map_srv_>::SharedPtr load_map_;
    rclcpp::Publisher<pose_msg_>::SharedPtr intial_pose_;

    int current_map_{1};
    int direction_map[3];
};
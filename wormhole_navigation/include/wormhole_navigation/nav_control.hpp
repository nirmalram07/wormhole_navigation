#include <mutex>
#include <memory>
#include <set>
#include <algorithm>

#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/srv/load_map.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "wormhole_nav_msg/action/worm_goal.hpp"
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
using pose_stamped_ = geometry_msgs::msg::PoseStamped;

public:

    /**
     * @brief Constructor and default destructor intialization.
     */
    WormholeNav();
    ~WormholeNav() = default;


    /**
     * @brief Initializes the data_fetcher code to access DB
     */
    void init();

private:
    
    /**
     * @brief Callback for handling incoming WormGoal requests
     * @param uuid Unique ID for the goal.
     * @param goal Shared pointer to the goal message.
     * @return Goal response indicating acceptance or rejection.
     */
    rclcpp_action::GoalResponse wnGoalCallback(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const worm_goal_action_::Goal> goal);

    /**
     * @brief Callback to handle cancellation requests from clients for WormGoal
     * @param goal_handle Shared pointer to the goal handle.
     * @return Cancel response indicating whether the goal was cancelled.
     */
    rclcpp_action::CancelResponse wnCancelCallback(const std::shared_ptr<worm_goal_handle_> goal_handle);

    /**
     * @brief Called when a goal is accepted by the action server.
     * @param goal_handle Shared pointer to the accepted goal handle.
     */
    void wnAcceptedCallback(const std::shared_ptr<worm_goal_handle_> goal_handle);

    /**
     * @brief Called by wnAcceptedCallback to not block the action server.
     * @param goal_handle Shared pointer to the goal handle passed from wnAcceptedCallback.
     */
    void wnExecuteCallback(const std::shared_ptr<worm_goal_handle_> goal_handle);

    /**
     * @brief Callback for handling the result of a NavigateToPose action.
     * @param result The wrapped result from the action server.
     */
    void clientResultCallback(const rclcpp_action::ClientGoalHandle<nav_to_pose_action_>::WrappedResult &result);
    
    /**
     * @brief Callback triggered after sending a goal to NavigateToPose(Not being used, just for debugging purposes).
     * @param goal_handle Shared pointer to the goal handle.
     */
    void clientResponseCallback(const rclcpp_action::ClientGoalHandle<nav_to_pose_action_>::SharedPtr &goal_handle);

    /**
     * @brief Callback for receiving feedback during goal execution of navigate to pose, also pubs the feedback for worm_goal.
     * @param goal_handle Shared pointer to the goal handle.
     * @param feedback Shared pointer to the feedback message.
     */
    void clientFeedbackCallback(const rclcpp_action::ClientGoalHandle<nav_to_pose_action_>::SharedPtr &goal_handle,
        const std::shared_ptr<const nav_to_pose_action_::Feedback> feedback);

    /**
     * @brief Calculates traversal order between maps using custom logic.
     * @param start Starting map index.
     * @param end Ending map index.
     */
    void calculateTraverseOrder(int start, int end); 
    
    /**
     * @brief Fetches data from db using a shared_ptr and sends navigate to pose goal.
     */
    void sendNavGoal();

    /**
     * @brief Sets up and initiates service client (LoadMap).
     */
    void handleServices();

    /**
     * @brief Handles result from LoadMap service call also calls the sendNavGoal method again.
     * @param Future Future containing the response from the LoadMap service.
     */
    void serviceResultHandler(rclcpp::Client<load_map_srv_>::SharedFuture Future);

    /// Mutex to protect shared resources inside the goal callback
    std::mutex mutex_;

    /// Handle to the current WormGoal
    std::shared_ptr<worm_goal_handle_> control_handle_{nullptr};

    /// Action result for wormhole navigation
    worm_goal_action_::Result worm_result_;

    /// Feedback message for wormhole navigation
    std::shared_ptr<worm_goal_action_::Feedback> worm_feedback{std::make_shared<worm_goal_action_::Feedback>()};

    /// Action server variable declaration for custom wormhole navigation
    rclcpp_action::Server<worm_goal_action_>::SharedPtr control_server_;

    /// Action client variable declaration for NavigateToPose (Nav2)
    rclcpp_action::Client<nav_to_pose_action_>::SharedPtr nav_command_;
    
    /// Client to request LoadMap service
    rclcpp::Client<load_map_srv_>::SharedPtr load_map_;
    
    /// Track current map index and result from the controller
    int current_map_{1}, controller_result_{0};

    /// Ordered list of maps to traverse
    std::vector<int> direction_map_;

    /// Goal pose received from the wormhole action
    pose_stamped_ goal_pose_;

    /// Pose retrieved from the map database
    pose_cov_ map_pose_;

    /// To control the data_fetcher node
    std::shared_ptr<DBPoseFetcher> pose_fetcher_;
};

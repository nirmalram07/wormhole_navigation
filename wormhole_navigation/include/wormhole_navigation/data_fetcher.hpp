#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sqlite3.h>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

/**
 * @brief Creating shorter aliases for ROS message type and nlohmann::json
 */
using json = nlohmann::json;
using pose_cov_ = geometry_msgs::msg::PoseWithCovarianceStamped;

/**
 * @class DBPoseFetcher
 * @brief A ROS 2 node that interacts with an SQLite3 database to fetch poses with covariance.
 */
class DBPoseFetcher : public rclcpp::Node {

public:
    DBPoseFetcher();
    ~DBPoseFetcher();

    /**
     * @brief Retrieves a pose with covariance from the database.
     * @param row_number The row index of the pose to fetch.
     * @return A PoseWithCovarianceStamped message containing the retrieved pose.
     */
    pose_cov_ get_pose_from_db(int row_number);

private:
    /**
     * @brief Inserts predefined data to DB.
     */
    void insert_sample_data();

    /**
     * @brief Creates a table in the DB.
     */
    void create_table();

    /// Pointer to the DB connection.
    sqlite3 *db_{nullptr};
};
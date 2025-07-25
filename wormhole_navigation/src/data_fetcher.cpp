#include "wormhole_navigation/data_fetcher.hpp"

DBPoseFetcher::DBPoseFetcher(const rclcpp::Node::SharedPtr &node): node_(node){
    int rc = sqlite3_open("pose_coord.db", &db_);
    if (rc) {
        RCLCPP_ERROR(node_->get_logger(), "Can't open database: %s", sqlite3_errmsg(db_));
        db_ = nullptr;
        return;
    }
    //RCLCPP_INFO(node_->get_logger(), "Opened database successfully");

    create_table();
    insert_sample_data();
}

DBPoseFetcher::~DBPoseFetcher(){
    if (db_) {
        sqlite3_close(db_);
        //RCLCPP_INFO(node_->get_logger(), "Database connection closed");
    }
}

pose_cov_ DBPoseFetcher::get_pose_from_db(int from, int to) {

    pose_cov_ pose_msg;

    std::string query = "SELECT data FROM json_data";
    sqlite3_stmt *stmt;

    int rc = sqlite3_prepare_v2(db_, query.c_str(), -1, &stmt, nullptr);
    if (rc != SQLITE_OK) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to prepare query: %s", sqlite3_errmsg(db_));
        return pose_msg;
    }

    while (sqlite3_step(stmt) == SQLITE_ROW) {
        const unsigned char* json_text = sqlite3_column_text(stmt, 0);
        if (json_text) {
            json j = json::parse(reinterpret_cast<const char*>(json_text));

            if (j.contains("from") && j.contains("to") &&
                j["from"] == from && j["to"] == to) {

                pose_msg.header.stamp = node_->get_clock()->now();
                pose_msg.header.frame_id = "map";
                pose_msg.pose.pose.position.x = j["x_pose"];
                pose_msg.pose.pose.position.y = j["y_pose"];
                pose_msg.pose.pose.position.z = 0.0;

                pose_msg.pose.pose.orientation.x = 0.0;
                pose_msg.pose.pose.orientation.y = 0.0;
                pose_msg.pose.pose.orientation.z = j["z_orient"];
                pose_msg.pose.pose.orientation.w = j["w_orient"];

                pose_msg.pose.covariance[0] = 0.1;
                pose_msg.pose.covariance[7] = 0.1;
                pose_msg.pose.covariance[35] = 0.2;

                //RCLCPP_INFO(node_->get_logger(), "DB fetched pose for move (%d → %d)", from, to);
                sqlite3_finalize(stmt);
                return pose_msg;
            }
        }
    }

    RCLCPP_ERROR(node_->get_logger(), "No matching pose for move (%d → %d)", from, to);
    sqlite3_finalize(stmt);
    return pose_msg;
}


void DBPoseFetcher::create_table(){
    
    const char *sql = "CREATE TABLE IF NOT EXISTS json_data (data TEXT)";
    char *errmsg = nullptr;

    int rc = sqlite3_exec(db_, sql, nullptr, nullptr, &errmsg);
    if (rc != SQLITE_OK) {
        RCLCPP_ERROR(node_->get_logger(), "SQL error creating table: %s", errmsg);
        sqlite3_free(errmsg);
    } else {
        //RCLCPP_INFO(node_->get_logger(), "Table created successfully");
    }
}

void DBPoseFetcher::insert_sample_data(){

    const char *clear_sql = "DELETE FROM json_data";
    sqlite3_exec(db_, clear_sql, nullptr, nullptr, nullptr);

    std::vector<json> json_rows = {
        {{"from", 1}, {"to", 2}, {"x_pose", -0.942}, {"y_pose", 3.262}, {"z_orient", 0.913}, {"w_orient", 0.407}},
        {{"from", 2}, {"to", 1}, {"x_pose", 1.123}, {"y_pose", 2.221}, {"z_orient", 0.881}, {"w_orient", 0.471}},
        {{"from", 2}, {"to", 3}, {"x_pose", -4.678}, {"y_pose", 8.816}, {"z_orient", 0.999}, {"w_orient", 0.044}},
        {{"from", 3}, {"to", 2}, {"x_pose", -2.301}, {"y_pose", 7.100}, {"z_orient", 0.222}, {"w_orient", 0.974}}
    };

    sqlite3_stmt *stmt;
    const char *insert_sql = "INSERT INTO json_data (data) VALUES (?)";
    sqlite3_prepare_v2(db_, insert_sql, -1, &stmt, nullptr);

    for (const auto &j : json_rows) {
        std::string json_str = j.dump();
        sqlite3_bind_text(stmt, 1, json_str.c_str(), -1, SQLITE_TRANSIENT);
        if (sqlite3_step(stmt) != SQLITE_DONE) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to insert row");
        }
        sqlite3_reset(stmt);
    }

    sqlite3_finalize(stmt);
   // RCLCPP_INFO(node_->get_logger(), "Inserted 2 JSON rows of coordinates");
}
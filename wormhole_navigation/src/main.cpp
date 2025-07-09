#include <memory>
#include "wormhole_navigation/nav_control.hpp"

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto Node = std::make_shared<WormholeNav>();
    //Calling init method to initialize the data_fetcher node
    Node->init();
    rclcpp::spin(Node);
    rclcpp::shutdown();
}
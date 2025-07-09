#include <memory>
#include "wormhole_navigation/nav_control.hpp"

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto Node = std::make_shared<WormholeNav>();
    Node->init();
    rclcpp::spin(Node);
    rclcpp::shutdown();
}
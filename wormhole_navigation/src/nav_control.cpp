#include "wormhole_navigation/nav_control.hpp"

WormholeNav::WormholeNav() : Node("navigation_controller"){

    intial_pose_ = this->create_publisher<pose_msg_>("/initial_pose", 10);
}
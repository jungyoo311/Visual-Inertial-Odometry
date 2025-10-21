#include "../include/vio_node/VioNode.hpp"
#include "rclcpp/rclcpp.hpp"
#include <memory>
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VioNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
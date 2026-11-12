#include "bridge.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Bridge>();
    RCLCPP_INFO(node->get_logger(), "Bridge node started. Waiting for brave warriors");

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
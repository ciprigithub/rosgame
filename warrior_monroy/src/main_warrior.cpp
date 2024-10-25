//--------------------------------------------------------------------//
// University of Málaga
// MAPIR Research Group - Machine Perception and Intelligent Robotics
//--------------------------------------------------------------------//

#include "warrior_monroy/warrior.hpp"
#include <chrono>
#include <thread>

int main ( int argc, char * argv[] )
{
    rclcpp::init ( argc, argv );
    
    auto node=std::make_shared<Warrior>();
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    rclcpp::Rate rate(5);

    while(rclcpp::ok())
    {
        rclcpp::spin_some(node);
        node->publish_twist();
        rate.sleep();
    }
    
    rclcpp::shutdown();
    return 0;
}
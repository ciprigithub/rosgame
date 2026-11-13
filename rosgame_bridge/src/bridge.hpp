#pragma once

#include <chrono>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rosgame_msgs/msg/rosgame_twist.hpp"
#include "rosgame_bridge/srv/rosgame_register.hpp"

class Bridge : public rclcpp::Node
{
public:
    Bridge();
    ~Bridge() override = default;

    // Topic exchange callbacks (player vs Coppelia)
    void cmdvel_info_exchange(const rosgame_msgs::msg::RosgameTwist::SharedPtr msg);
    void laser_info_exchange(const sensor_msgs::msg::LaserScan::SharedPtr msg, const std::string &code);
    void scene_info_exchange(const std_msgs::msg::String::SharedPtr msg, const std::string &code);

private:
    // Registration service
    rclcpp::Service<rosgame_bridge::srv::RosgameRegister>::SharedPtr server_register_;
    void handle_register_service(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<rosgame_bridge::srv::RosgameRegister::Request> request,
        std::shared_ptr<rosgame_bridge::srv::RosgameRegister::Response> response
    );

    // Robot spawn trigger topic for CoppeliaSim
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;

    // event pub
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_events_;

    // Generates a random alphanumeric topic-safe code
    std::string generate_code(int length);

    struct possible_skills
    {
        bool hammer = false;
        bool shield = false;
    };

    struct Player
    {
        int robot_idx = 0;
        std::string username;
        possible_skills skills;
        std::string color;
        bool isbanned = false;

        // cmd_vel pub/sub
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmdvel_;
        rclcpp::Subscription<rosgame_msgs::msg::RosgameTwist>::SharedPtr sub_cmdvel_;

        // laser_scan pub/sub
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_laser_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser_;

        // scene_info pub/sub
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_scene_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_scene_;        
    };

    std::unordered_map<std::string, Player> RosgamePlayers;

    // Security monitor timer
    rclcpp::TimerBase::SharedPtr timer;
    void timer_callback();
    bool is_system_node(const std::string &node_name);
    
    std::vector<std::string> colors;
};
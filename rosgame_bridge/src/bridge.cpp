#include "bridge.hpp"
#include <cstdlib>
#include <ctime>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using namespace std::literals::chrono_literals;

Bridge::Bridge() : Node("bridge")
{
    // Offer Registration Srv to Players
    server_register_ = this->create_service<rosgame_bridge::srv::RosgameRegister>(
        "register_service", std::bind(&Bridge::handle_register_service, this, _1, _2, _3));

    pub_ = this->create_publisher<std_msgs::msg::String>("/create_robot", 10);
    pub_events_ = this->create_publisher<std_msgs::msg::String>("/game_events", 10);

    // Anti-cheat check every Xs
    timer = this->create_wall_timer(3s, std::bind(&Bridge::timer_callback, this));

    srand(static_cast<unsigned int>(time(nullptr)));
    colors = {"RED", "GREEN", "BLUE", "PURPLE", "CYAN", "YELLOW", "ORANGE", "PINK"};
}


bool Bridge::is_system_node(const std::string &node_name)
{
    return node_name == this->get_name() ||
           node_name == "referee" ||
           node_name == "match_referee" ||
           node_name == "ros2cli";
}


// Someone Cheating?
void Bridge::timer_callback()
{
    // 1. Protect /create_robot from unauthorized publishers
    for (const auto &info : this->get_publishers_info_by_topic("/create_robot"))
    {
        if (!is_system_node(info.node_name()))
        {
            RCLCPP_ERROR_STREAM(this->get_logger(),
                "[ANTI-CHEAT] Unauthorized node '" << info.node_name() << "' published to /create_robot!");
        }
    }

    // 2. Protect internal CoppeliaSim topics (/controlRobotN/* and /RobotN/*)
    for (auto &entry : RosgamePlayers)
    {
        const std::string &code = entry.first;
        Player &p = entry.second;
        std::string idx = std::to_string(p.robot_idx);

        // Nobody except Bridge may publish to /controlRobotN/cmd_vel
        for (const auto &info : this->get_publishers_info_by_topic("/controlRobot" + idx + "/cmd_vel"))
        {
            if (info.node_name() != this->get_name())
            {
                RCLCPP_ERROR_STREAM(this->get_logger(),
                    "[ANTI-CHEAT] Node '" << info.node_name()
                    << "' published directly to /controlRobot" << idx << "/cmd_vel! Banning " << p.username);
                p.isbanned = true;
            }
        }

        // Nobody except Bridge and Referee may subscribe to /RobotN/scene_info
        for (const auto &info : this->get_subscriptions_info_by_topic("/Robot" + idx + "/scene_info"))
        {
            if (!is_system_node(info.node_name()))
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(),
                        "[ANTI-CHEAT] Unauthorized node '" << info.node_name()
                        << "' snooped internal topic /Robot" << idx  << "/scene_info!");
                }
        }        
        
        // Nobody except Bridge should subscribe to /<code>/cmd_vel (prevents stealing msg->code)
        for (const auto &info : this->get_subscriptions_info_by_topic("/" + code + "/cmd_vel"))
        {
            if (info.node_name() != this->get_name())
            {
                RCLCPP_ERROR_STREAM(this->get_logger(),
                    "[ANTI-CHEAT] Node '" << info.node_name()
                    << "' sniffed secret code on /" << code << "/cmd_vel!");
            }
        }

        // Only the owning team's node may subscribe to /<code>/scene_info 
        for (const auto &topic : {"/" + code + "/scene_info"})
        {
            auto subs = this->get_subscriptions_info_by_topic(topic);
            int external_subs = 0;
            for (const auto &info : subs)
            {
                if (!is_system_node(info.node_name()))
                    external_subs++;
            }
            if (external_subs > 1)
            {
                RCLCPP_ERROR_STREAM(this->get_logger(),
                    "[ANTI-CHEAT] Multiple subscribers (" << external_subs
                    << ") detected on private topic " << topic << "!");
            }
        }
    }
}


void Bridge::ban_offender_node(const std::string &offender_node, const std::string &reason)
{
    if (offender_node == this->get_name() || offender_node == "match_referee" || offender_node == "ros2cli")
        return;

    for (auto &entry : RosgamePlayers)
    {
        Player &p = entry.second;
        // Match if the ROS node name contains the team's registered username
        if (!p.isbanned && offender_node.find(p.username) != std::string::npos)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(),
                "[ANTI-CHEAT] BANNED '" << p.username << "' (node: " << offender_node << ") -> " << reason);
            p.isbanned = true;
            return;
        }
    }
    RCLCPP_WARN_STREAM(this->get_logger(),
        "[ANTI-CHEAT] Unauthorized node '" << offender_node << "' detected -> " << reason);
}


std::string Bridge::generate_code(int length)
{
    static const char alfanum[] =
        "0123456789"
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz";

    std::string code;
    code.reserve(length);
    // First character cannot be a digit in ROS 2 topic names
    code += alfanum[(rand() % (sizeof(alfanum) - 11)) + 10];
    for (int i = 1; i < length; ++i)
    {
        code += alfanum[rand() % (sizeof(alfanum) - 1)];
    }
    return code;
}



void Bridge::handle_register_service(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<rosgame_bridge::srv::RosgameRegister::Request> request,
    std::shared_ptr<rosgame_bridge::srv::RosgameRegister::Response> response)
{
    RCLCPP_INFO_STREAM(this->get_logger(), "--------------------------------------------------");
    RCLCPP_INFO_STREAM(this->get_logger(), "Register request [" << request_header->sequence_number << "] for: " << request->username);

    // Username must be unique
    for (const auto &entry : RosgamePlayers)
    {
        if (entry.second.username == request->username)
        {
            response->code = "-1";
            RCLCPP_WARN_STREAM(this->get_logger(), "Username already registered.");
            return;
        }
    }

    std::string code = generate_code(8);
    response->code = code;

    int n_robot = static_cast<int>(RosgamePlayers.size()) + 1;
    std::string color_label = colors[(n_robot - 1) % colors.size()];
    RCLCPP_INFO_STREAM(this->get_logger(), "Assigned Robot" << n_robot << " (" << color_label << ")");

    // Robot topic names (Player and Coppelia)
    std::string coppelia_cmdvel_topic = "/controlRobot" + std::to_string(n_robot) + "/cmd_vel";
    std::string player_cmdvel_topic   = "/" + code + "/cmd_vel";
    std::string coppelia_laser_topic  = "/Robot" + std::to_string(n_robot) + "/laser_scan";
    std::string player_laser_topic    = "/" + code + "/laser_scan";
    std::string coppelia_scene_topic  = "/Robot" + std::to_string(n_robot) + "/scene_info";
    std::string player_scene_topic    = "/" + code + "/scene_info";

    Player new_player;
    new_player.robot_idx = n_robot;
    new_player.username  = request->username;
    new_player.isbanned  = false;
    new_player.skills    = {false, false};  //shield, hammer

    // Velocity commands: Player -> Bridge -> CoppeliaSim
    new_player.pub_cmdvel_ = this->create_publisher<geometry_msgs::msg::Twist>(coppelia_cmdvel_topic, 10);
    new_player.sub_cmdvel_ = this->create_subscription<rosgame_msgs::msg::RosgameTwist>(
        player_cmdvel_topic, 10, std::bind(&Bridge::cmdvel_info_exchange, this, _1));

    // LaserScan: CoppeliaSim -> Bridge -> Player (O(1) bound code)
    new_player.pub_laser_ = this->create_publisher<sensor_msgs::msg::LaserScan>(player_laser_topic, 10);
    new_player.sub_laser_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        coppelia_laser_topic, 10,
        [this, code](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
            this->laser_info_exchange(msg, code);
        });

    // SceneInfo: CoppeliaSim -> Bridge -> Player (O(1) bound code, zero-copy)
    new_player.pub_scene_ = this->create_publisher<std_msgs::msg::String>(player_scene_topic, 10);
    new_player.sub_scene_ = this->create_subscription<std_msgs::msg::String>(
        coppelia_scene_topic, 10,
        [this, code](const std_msgs::msg::String::SharedPtr msg) {
            this->scene_info_exchange(msg, code);
        });

    RosgamePlayers[code] = std::move(new_player);

    // Notify CoppeliaSim Robots_Generator to spawn the robot
    std_msgs::msg::String spawn_msg;
    spawn_msg.data = request->username;
    pub_->publish(spawn_msg);

    // Report event (robot creation)
    std_msgs::msg::String reg_event;
    reg_event.data = "REGISTER|Robot" + std::to_string(n_robot) + "|" + request->username + "|" + color_label;
    pub_events_->publish(reg_event);


    RCLCPP_INFO(this->get_logger(), "Player '%s' -> code='%s' | topics: /%s/{cmd_vel, laser_scan, scene_info}",
                request->username.c_str(), code.c_str(), code.c_str());
}


void Bridge::cmdvel_info_exchange(const rosgame_msgs::msg::RosgameTwist::SharedPtr msg)
{
    auto it = RosgamePlayers.find(msg->code);
    if (it != RosgamePlayers.end() && !it->second.isbanned)
    {
        geometry_msgs::msg::Twist velocity;
        velocity.linear.x = msg->vel.linear.x;
        velocity.angular.z = msg->vel.angular.z;
        it->second.pub_cmdvel_->publish(velocity);
    }
}

void Bridge::laser_info_exchange(const sensor_msgs::msg::LaserScan::SharedPtr msg, const std::string &code)
{
    auto it = RosgamePlayers.find(code);
    if (it != RosgamePlayers.end() && !it->second.isbanned)
    {
        sensor_msgs::msg::LaserScan out_scan = *msg;
        out_scan.header.frame_id = code + "/laser_scan";
        it->second.pub_laser_->publish(out_scan);
    }
}

void Bridge::scene_info_exchange(const std_msgs::msg::String::SharedPtr msg, const std::string &code)
{
    auto it = RosgamePlayers.find(code);
    if (it != RosgamePlayers.end() && !it->second.isbanned)
    {
        it->second.pub_scene_->publish(*msg);
    }
}
//--------------------------------------------------------------------//
//University of Málaga
//MAPIR Research Group - Machine Perception and Intelligent Robotics
//--------------------------------------------------------------------//

#include "warrior_monroy/warrior.hpp"

Warrior::Warrior(): Node ("robot_warrior")
{
    warrior_nick = "Monroy";
    int cont = 0;
    
    // Se crea un cliente de servicio y una solicitud para lanzar el servicio.
    auto client = create_client<rosgame_bridge::srv::RosgameRegister>("register_service");
    
    auto request = std::make_shared<rosgame_bridge::srv::RosgameRegister::Request>();
    request -> username = warrior_nick;
    
    // Se espera a que el servicio esté disponible.
    bool service_available = false;
    while(!service_available && rclcpp::ok())
    {
        if (client->wait_for_service(std::chrono::seconds(5)))
        {   service_available = true;   }
        else
        {   RCLCPP_INFO(this->get_logger(), "Service not available. Retrying...");  }
    }
    
    // Se llama al servicio hasta que la respuesta sea diferente a "-1". Este valor indica que ya existe un jugador registrado con el nombre de usuario proporcionado.
    while (code == "-1" && rclcpp::ok())
    {   
        auto future = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = future.get();
            code = response->code;

            if (code == "-1")
            {
                RCLCPP_WARN(this->get_logger(), "Username already exists. Calling the service again...");
                warrior_nick = warrior_nick + std::to_string(cont);
                request -> username = warrior_nick;
                cont = cont + 1;
            }
            else
            {   
                // Se definen los publicadores y suscriptores necesarios.
                pub1_ = create_publisher<rosgame_msgs::msg::RosgameTwist>( "/" + code + "/cmd_vel", 1 );
                pub2_ = create_publisher<rosgame_msgs::msg::RosgamePoint>( "/" + code + "/goal_x_y", 1 );
                sub1_ = create_subscription<sensor_msgs::msg::LaserScan>( "/" + code + "/laser_scan", 1, std::bind(&Warrior::process_laser_info, this, std::placeholders::_1));
                sub2_ = create_subscription<std_msgs::msg::String>( "/" + code + "/scene_info", 1, std::bind(&Warrior::process_scene_info, this, std::placeholders::_1));
                sub3_ = create_subscription<geometry_msgs::msg::Twist>( "/cmd_vel", 1, std::bind(&Warrior::process_keyboard, this, std::placeholders::_1));
                RCLCPP_INFO(this->get_logger(), "Player registered. Starting simulation.");           
            }
        }
    }

    // Monroy Vars
    turning = false;
    turn_count = 0;
    current_robot_speeds.vel.linear.x = 0.0;
    current_robot_speeds.vel.angular.z = 0.0;
    current_robot_speeds.code = code;

    desired_robot_speeds.vel.linear.x = 0.0;
    desired_robot_speeds.vel.angular.z = 0.0;
}

Warrior::~Warrior()
{
     RCLCPP_ERROR(this->get_logger(), "Game over for [%s]", warrior_nick.c_str());
}


void Warrior::process_keyboard(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    // update desired robot speeds
    desired_robot_speeds.vel.linear.x = msg->linear.x;
    desired_robot_speeds.vel.angular.z = msg->angular.z;
}


void Warrior::process_laser_info(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{    
    // if (!turning)
    // {
    //     float vel_lin = 0.8;
    //     float vel_ang = M_PI/4;

    //     // number of elements in array
    //     int n_ranges = msg->ranges.size();

    //     // search the minimum distance (closest object)
    //     std::vector<float>::iterator min_it = std::min_element(msg->ranges.begin(), msg->ranges.end());

    //     // get distance and position in array
    //     double nearest_obstacle_distance = *min_it;
    //     int pos_in_array = std::distance(msg->ranges.begin(), min_it);

    //     // inform
    //     //RCLCPP_INFO(this->get_logger(), "Nearest obstacle detected at %.2f[m] at vector position %i/%i", nearest_obstacle_distance, pos_in_array, n_ranges);

    //     // Check ranges
    //     if (nearest_obstacle_distance <= 1){
    //         // Danger!
    //         turning = true;
    //         if (pos_in_array <= floor(n_ranges/2)){
    //             //object on the right -> turn left
    //             desired_robot_speeds.vel.linear.x = 0.0;
    //             desired_robot_speeds.vel.angular.z = vel_ang;
    //             RCLCPP_INFO(this->get_logger(), "Turning Left");
    //         }else{
    //             //object on the left -> turn right
    //             desired_robot_speeds.vel.linear.x = 0.0;
    //             desired_robot_speeds.vel.angular.z = -vel_ang;
    //             RCLCPP_INFO(this->get_logger(), "Turning Right");
    //         }
    //     }else{
    //         // Free path ahead
    //         desired_robot_speeds.vel.linear.x = vel_lin;
    //         desired_robot_speeds.vel.angular.z = 0.0;
    //         //RCLCPP_INFO(this->get_logger(), "Going Straight!");
    //     }
    // }
}

void Warrior::publish_twist()
{
    //Executed at 5Hz
    if (turning)
    {
        // increase counter to turn a bit (called at 5Hz)
        turn_count++;    
        // turn completed?
        if (turn_count >= 5)
        {
            turning = false;
            turn_count = 0;
            RCLCPP_INFO(this->get_logger(), "Going Straight!");
        }
    }

    // Linear Acceleration
    float inc_v = 0.1;
    if (abs(current_robot_speeds.vel.linear.x-desired_robot_speeds.vel.linear.x) > inc_v)
    {
        if (current_robot_speeds.vel.linear.x > desired_robot_speeds.vel.linear.x)
            current_robot_speeds.vel.linear.x -= inc_v;
        else
            current_robot_speeds.vel.linear.x += inc_v;
    }
    else
        current_robot_speeds.vel.linear.x = desired_robot_speeds.vel.linear.x;
    
    // Angular Acceleration
    float inc_w = 0.3;
    if (abs(current_robot_speeds.vel.angular.z-desired_robot_speeds.vel.angular.z) > inc_w)
    {
        if (current_robot_speeds.vel.angular.z > desired_robot_speeds.vel.angular.z)
            current_robot_speeds.vel.angular.z -= inc_w;
        else
            current_robot_speeds.vel.angular.z += inc_w;
    }
    else
        current_robot_speeds.vel.angular.z = desired_robot_speeds.vel.angular.z;
    
    // Publish
    pub1_->publish(current_robot_speeds);
}


void Warrior::process_scene_info(const std_msgs::msg::String::SharedPtr msg)
{
    // Se convierte el msg de tipo string con formato en un JSON.
    Json::CharReaderBuilder reader;
    Json::Value JsonSceneData;
    std::istringstream jsonStream(msg->data);
    Json::parseFromStream(reader, jsonStream, &JsonSceneData, nullptr);
        
    // Se obtiene el valor de la batería de la clave "Battery_Level".
    battery = JsonSceneData["Battery_Level"].asFloat();

    // Se obtiene la pose del robot a partir de la clave "Robot_Pose".
    pos_x = JsonSceneData["Robot_Pose"]["x"].asFloat();
    pos_y = JsonSceneData["Robot_Pose"]["y"].asFloat();
    gamma = JsonSceneData["Robot_Pose"]["gamma"].asFloat();

    // Se obtienen las habilidades de la cave "Skills".
    autopilot_enabled = JsonSceneData["Skills"]["Autopilot"].asBool();
    hammer_enabled = JsonSceneData["Skills"]["Hammer"].asBool();
    shield_enabled = JsonSceneData["Skills"]["Shield"].asBool();

    // Se obtienen las posiciones de bloques de habilidades de la clave "Skills_Positions" en el campo "FOV".
    std::vector<std::vector<float>> skills_pos_array_aux;
    const Json::Value &skills_pos = JsonSceneData["FOV"]["Coins_Positions"];
    for (const Json::Value &skill : skills_pos)
    {
        std::vector<float> skillData;
        for (const Json::Value &value : skill)
        {   skillData.push_back(value.asFloat());   }
        skills_pos_array_aux.push_back(skillData);
    }
    skills_pos_array = skills_pos_array_aux;

    // Se obtienen las posiciones de plataformas de recarga de la clave "Chargers_Positions" en el campo "FOV".
    // Las plataformas de recarga están en posiciones fijas durante toda la simulación.
    // Objetivo: buscar las cinco plataformas y almacenarlas todas en la lista "chargers_pos_array".
    const Json::Value &chargers_pos = JsonSceneData["FOV"]["Chargers_Positions"];
    for (const Json::Value &charger : chargers_pos)
    {
        std::vector<float> chargerData;
        for (const Json::Value &value : charger)
        {   chargerData.push_back(value.asFloat());     }

        // Verifica si la posición ya está en "chargers_pos_array".
        // La función "find" devuelve un iterador que apunta al elemento del array si existe o al final del array si no lo ha encontrado.
        if (std::find(chargers_pos_array.begin(), chargers_pos_array.end(), chargerData) == chargers_pos_array.end())
        {   chargers_pos_array.push_back(chargerData);  }
    }

    // Se obtienen las posiciones de los oponentes de la clave "Players_Positions" en el campo "FOV".
    std::vector<std::vector<float>> players_pos_array_aux;
    const Json::Value &players_pos = JsonSceneData["FOV"]["Players_Positions"];
    for (const Json::Value &player : players_pos)
    {
        std::vector<float> playerData;
        for (const Json::Value &value : player)
        {   playerData.push_back(value.asFloat());  }
        players_pos_array_aux.push_back(playerData);
    }
    players_pos_array = players_pos_array_aux;
    
    // DEBUGGING
    RCLCPP_INFO(this->get_logger(), "Battery level: '%f'", battery);
    RCLCPP_INFO(this->get_logger(), "Robot Pose: [X] = '%f', [Y] = '%f', [GAMMA] = '%f'", pos_x, pos_y, gamma);
    RCLCPP_INFO(this->get_logger(), "Msg: '%s'", msg->data.c_str());
    
    /*
    RCLCPP_INFO(this->get_logger(), "Msg: '%s'", msg->data.c_str());

    RCLCPP_INFO(this->get_logger(), "Battery level: '%f'", battery);

    RCLCPP_INFO(this->get_logger(), "Is teleport enabled? '%s'", (teleport_enabled ? "True" : "False"));
    RCLCPP_INFO(this->get_logger(), "Is the hammer enabled? '%s'", (hammer_enabled ? "True" : "False"));
    RCLCPP_INFO(this->get_logger(), "Is the shield enabled? '%s'", (shield_enabled ? "True" : "False"));

    if (!skills_pos_array.empty())
    {
        RCLCPP_INFO(this->get_logger(), "Skills Positions Array:");
        for (const auto &skill_pos : skills_pos_array)
        {   RCLCPP_INFO(this->get_logger(), "[X] = '%f', [Y] = '%f'", skill_pos[0], skill_pos[1]);  }
    }

    RCLCPP_INFO(this->get_logger(), "Robot Pose: [X] = '%f', [Y] = '%f', [GAMMA] = '%f'", pos_x, pos_y, gamma);
    */
}



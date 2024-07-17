#include "player_definition.hpp"

int main ( int argc, char * argv[] )
{
    // La inicialización de ROS debe ser la primera instrucción.
    rclcpp::init ( argc, argv );
    
    auto node=std::make_shared<Player>();

    std::string user = "UsuarioRosgame";
    int cont = 0;

    // Se crea un cliente de servicio y una solicitud para lanzar el servicio.
    auto client = node->create_client<rosgame_bridge::srv::RosgameRegister>("register_service");
    auto request = std::make_shared<rosgame_bridge::srv::RosgameRegister::Request>();
    request -> username = user;
    
    // Se espera a que el servicio esté disponible.
    bool service_available = false;
    while(!service_available && rclcpp::ok())
    {
        if (client->wait_for_service(std::chrono::seconds(5)))
        {   service_available = true;   }
        else
        {   RCLCPP_INFO(node->get_logger(), "Service not available. Retrying...");  }
    }
    
    // Se llama al servicio hasta que la respuesta sea diferente a "-1". Este valor indica que ya existe un jugador registrado con el nombre de usuario proporcionado.
    while (node->code == "-1" && rclcpp::ok())
    {   
        auto future = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = future.get();
            node->code = response->code;

            if (node->code == "-1")
            {
                RCLCPP_WARN(node->get_logger(), "Username already exists. Calling the service again...");
                request -> username = user + std::to_string(cont);
                cont = cont + 1;
            }
            else
            {   
                // Se definen los publicadores y suscriptores necesarios.
                node->pub1_ = node->create_publisher<rosgame_msgs::msg::RosgameTwist>( "/" + node->code + "/cmd_vel", 10 );
                node->pub2_ = node->create_publisher<rosgame_msgs::msg::RosgamePoint>( "/" + node->code + "/goal_x_y", 10 );
                node->sub1_ = node->create_subscription<sensor_msgs::msg::LaserScan>( "/" + node->code + "/laser_scan", 10, std::bind(&Player::process_laser_info, node.get(), std::placeholders::_1));
                node->sub2_ = node->create_subscription<std_msgs::msg::String>( "/" + node->code + "/scene_info", 10, std::bind(&Player::process_scene_info, node.get(), std::placeholders::_1));
                RCLCPP_INFO(node->get_logger(), "Player registered. Starting simulation.");           
            }
        }
    }

    rclcpp::Rate rate(1);

    while(rclcpp::ok())
    {
        rclcpp::spin_some(node);
        rate.sleep();
    }
    
    rclcpp::shutdown();
    return 0;
}
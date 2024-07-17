#include "player_definition.hpp"

Player::Player(): Node ("pacific_player")
{
    // ...
}


void Player::process_laser_info(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // Se busca la distancia mínima, que representa el objeto más cercano.
    std::vector<float>::iterator min_it = std::min_element(msg->ranges.begin()+25, msg->ranges.end()-25);

    double nearest_obstacle_distance = *min_it;

    // Con la información del sensor láser y de la escena se establecen los parámetros de velocidad (lineal y angular).
    std::tuple<double,double,double,double> result = autonomous_navigation(nearest_obstacle_distance);
    double linear = std::get<0>(result);
    double angular = std::get<1>(result);

    publish_vel(linear,angular);
}


void Player::process_scene_info(const std_msgs::msg::String::SharedPtr msg)
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
    const Json::Value &skills_pos = JsonSceneData["FOV"]["Skills_Positions"];
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


std::tuple<double, double, double, double> Player::autonomous_navigation(double nearest_obstacle_distance)
{
    double linear, angular;

    // Si el obstáculo más cercano está por debajo de un umbral, hay que evitarlo.
    if ( (nearest_obstacle_distance < 0.3) or colision_detectada )
    {
        // Es necesario conocer cuando se detectó el obstáculo por primera vez para anotar la orientación inicial del robot.
        if (!colision_detectada)
        {
            RCLCPP_INFO(this->get_logger(), "¡Nueva colisión detectada a '%f' metros: ", nearest_obstacle_distance);
            colision_detectada = true;
            objetivo_fijado = false;
            fin_giro = false;
            
            float gamma_grados;
            float orientacion_final_grados;

            // La orientación viene dada en el rango (-pi,pi). Conversión al rango (0,360) para trabajar de forma más cómoda.
            if (gamma > 0)
            {   gamma_grados = gamma * 180/M_PI;    }
            else
            {   gamma_grados = 360 + gamma * 180/M_PI;  }
            
            // El objetivo es girar 90º desde la orientación en la que se detectó el obstáculo.
            // La orientación no puede estar por encima de 360º.
            orientacion_final_grados = gamma_grados + 90;
            if (orientacion_final_grados > 360)
            {   orientacion_final_grados = orientacion_final_grados - 360;   }

            // Conversión al rango inicial (-pi,pi).
            if (orientacion_final_grados>0 and orientacion_final_grados<180)
            {   orientacion_final = orientacion_final_grados * M_PI/180;    }
            else
            {   orientacion_final = (orientacion_final_grados-360) * M_PI/180;  }
        }
        else
        {
            // Una vez encontrado el obstáculo, primero hay que girar para evitarlo.
            if (!fin_giro)
            {
                RCLCPP_INFO(this->get_logger(), "Gira para evitar obstáculo. Progreso de la orientacion: '%f' rad / '%f' rad.", gamma, orientacion_final);
                linear = 0; angular = 1;

                // Cuando la diferencia angular entre la orientación actual y la final sea de unos 10º, se asume que ha completado el giro.
                if ( fabs(gamma-orientacion_final) < 10*M_PI/180 )
                {   fin_giro = true;    }
            }
            else
            {
                // Avanza en línea recta, a no ser que encuentre un obstáculo, en cuyo caso se reinicia el bucle.
                RCLCPP_INFO(this->get_logger(), "Avanza en línea recta para evitar obstáculo");
                RCLCPP_INFO(this->get_logger(), "El obstáculo está a: '%f'", nearest_obstacle_distance);
                linear = 1.5; angular = 0;

                // Cuando el obstáculo esté a una distancia superior a 1m se considera que se ha evitado.
                // Si la distancia es inferior a 0.2m se pone a "false" la variable de colisión para que reinicie el bucle de evitación de obstáculos.
                if ( (nearest_obstacle_distance < 0.2) or (nearest_obstacle_distance > 1) )
                {   colision_detectada = false;   }
            }
        } 
    }
    else
    {   
        // PASO 1. ESTABLECER EL OBJETIVO.
        if ( not objetivo_fijado )
        {
            // Se avanza en línea recta mientras se busca un nuevo objetivo.
            linear = 1; angular = 0;
            
            // a. El robot tiene un buen nivel de batería y se conoce la posición de algún bloque de habilidades.
            if ( not skills_pos_array.empty() and battery>50 )
            {
                // La distancia mínima se inicia a un valor elevado.
                float distance_min = std::numeric_limits<float>::max();

                // Se calcula la distancia a cada bloque de habilidades y se va al más cercano.
                for (size_t k = 0; k < skills_pos_array.size(); ++k)
                {
                    float delta_x = pos_x - skills_pos_array[k][0];
                    float delta_y = pos_y - skills_pos_array[k][1];
                    float distance = sqrt(delta_x*delta_x + delta_y*delta_y);

                    if ( (distance < distance_min) or k==0 )
                    {
                        distance_min = distance;
                        objetivo_x = skills_pos_array[k][0];
                        objetivo_y = skills_pos_array[k][1];
                    }
                }
                objetivo_fijado = true;
                tipo_objetivo = "Skill";
                RCLCPP_INFO(this->get_logger(), "¡Nuevo objetivo! Bloque de habilidades en la ubicación [X]='%f', [Y]='%f'.", objetivo_x, objetivo_y);
            }
            // b. El nivel de batería del robot está por debajo de un umbral y se conoce la posición de alguna plataforma de recarga.
            else if ( !chargers_pos_array.empty() and battery<50 )
            {
                // La distancia mínima se inicia a un valor elevado.
                float distance_min = std::numeric_limits<float>::max();

                // Se calcula la distancia a cada plataforma de recarga y se va a la más cercana.
                for (size_t k = 0; k < chargers_pos_array.size(); ++k)
                {
                    float delta_x = pos_x - chargers_pos_array[k][0];
                    float delta_y = pos_y - chargers_pos_array[k][1];
                    float distance = sqrt(delta_x*delta_x + delta_y*delta_y);

                    if ( (distance < distance_min) or k==0 )
                    {
                        distance_min = distance;
                        objetivo_x = chargers_pos_array[k][0];
                        objetivo_y = chargers_pos_array[k][1];
                    }
                }
                objetivo_fijado = true;
                tipo_objetivo = "Charger";
                RCLCPP_INFO(this->get_logger(), "¡Nuevo objetivo! Plataforma de recarga en la ubicación [X]='%f', [Y]='%f'.", objetivo_x, objetivo_y);
            }
            // c. En cualquier otro caso, avanzar en línea recta.
            else
            {
                linear = 1; angular = 0;
            }
        }
        // PASO 2. IR A POR EL OBJETIVO.
        else
        {
            // Se calcula la diferencia angular entre la orientación actual y la orientación hacia el objetivo.
            float diferencia_angular = atan2(objetivo_y - pos_y, objetivo_x - pos_x) - gamma;

            // La diferencia angular debe estar en el rango [-pi, pi].
            if (diferencia_angular > M_PI)
            {   diferencia_angular -= 2*M_PI;   } 
            else if (diferencia_angular < -M_PI)
            {   diferencia_angular += 2*M_PI;   }

            // DEBUGGING.
            RCLCPP_INFO(this->get_logger(), "Diferencia angular: '%f'.",diferencia_angular);
            RCLCPP_INFO(this->get_logger(), "Posición robot: [X]:'%f', [Y]:'%f'.", pos_x, pos_y);
            RCLCPP_INFO(this->get_logger(), "Posición target: [X]:'%f', [Y]:'%f'.", objetivo_x, objetivo_y);

            // Si la diferencia angular es superior al umbral, el robot sigue girando porque aún no está alineado.
            if ( (diferencia_angular > 0.15) or (diferencia_angular < -0.15) )
            {   
                angular = 0.5*diferencia_angular;
                linear = 0;
            }
            // Cuando esté alineado, avanza en línea recta.
            else
            {   angular = 0; linear = 1;    }

            // Se calcula la distancia al objetivo.
            float distance = sqrt( pow(objetivo_y - pos_y, 2) + pow(objetivo_x - pos_x, 2) );
            RCLCPP_INFO(this->get_logger(), "Distancia al objetivo: '%f'.", distance);
            
            if (tipo_objetivo=="Skill" and distance < 0.6)
            {
                RCLCPP_INFO(this->get_logger(), "¡Habilidad conseguida! Buscando un nuevo objetivo...");
                objetivo_fijado = false;
                tipo_objetivo = "";
            }
            else if (tipo_objetivo=="Charger" and battery>50)
            {
                RCLCPP_INFO(this->get_logger(), "¡Batería recargada! Buscando un nuevo objetivo...");
                objetivo_fijado = false;
                tipo_objetivo = "";
            }
        }
    }

    // Si tenemos el objetivo fijado en un bloque de habilidades y la batería es inferior a un umbral, se cancela el objetivo y se busca una plataforma de recarga.
    if (objetivo_fijado and tipo_objetivo=="Skill" and battery<50)
    {
        objetivo_fijado = false;
        tipo_objetivo = "";
    }

    return std::make_tuple(linear, angular,0,0);
}


void Player::publish_vel(double linear, double angular)
{
    rosgame_msgs::msg::RosgameTwist cmd_vel;

    cmd_vel.vel.linear.x = linear;
    cmd_vel.vel.angular.z = angular;
    cmd_vel.code = code;

    pub1_->publish(cmd_vel);
}
cmake_minimum_required(VERSION 3.8)
project(rosgame_players)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosgame_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosgame_bridge REQUIRED)

# Find and include JSONCPP package
find_package(jsoncpp REQUIRED)
include_directories(${jsoncpp_INCLUDE_DIRS})

# INSTANCIA Nº0 (WARRIOR) 
add_executable(warrior_player src/warrior.cpp src/main_warrior.cpp)
target_link_libraries(warrior_player jsoncpp)  

install(TARGETS
   warrior_player
   DESTINATION lib/${PROJECT_NAME})
   
ament_target_dependencies(warrior_player
                          rosgame_bridge
                          rosgame_msgs
                          rclcpp
                          std_msgs
                          geometry_msgs
                          sensor_msgs)

# INSTANCIA Nº1 DEL JUGADOR
add_executable(pacific_player src/pacific_player.cpp src/main_players.cpp)

# Link the player executables with JSONCPP
target_link_libraries(pacific_player jsoncpp)  

install(TARGETS
   pacific_player
   DESTINATION lib/${PROJECT_NAME})
   
ament_target_dependencies(pacific_player
                          rosgame_bridge
                          rosgame_msgs
                          rclcpp
                          std_msgs
                          geometry_msgs
                          sensor_msgs)

# INSTANCIA Nº2 DEL JUGADOR
add_executable(war_player src/war_player.cpp src/main_players.cpp)

# Link the player executables with JSONCPP
target_link_libraries(war_player jsoncpp)  

install(TARGETS
   war_player
   DESTINATION lib/${PROJECT_NAME})
   
ament_target_dependencies(war_player
                          rosgame_bridge
                          rosgame_msgs
                          rclcpp
                          std_msgs
                          geometry_msgs
                          sensor_msgs)        

                          

# INSTANCIA Nº3 DEL JUGADOR
add_executable(elusive_player src/elusive_player.cpp src/main_players.cpp)

# Link the player executables with JSONCPP
target_link_libraries(elusive_player jsoncpp)  

install(TARGETS
   elusive_player
   DESTINATION lib/${PROJECT_NAME})
   
ament_target_dependencies(elusive_player
                          rosgame_bridge
                          rosgame_msgs
                          rclcpp
                          std_msgs
                          geometry_msgs
                          sensor_msgs)        
                          

install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}
        )           

ament_package()

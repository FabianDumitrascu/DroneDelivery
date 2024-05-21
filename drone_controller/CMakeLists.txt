cmake_minimum_required(VERSION 3.0.2)
project(drone_controller)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


# Apollo1 Node
add_executable(apollo1_node src/apollo1.cpp)
target_link_libraries(apollo1_node ${catkin_LIBRARIES})

# Takeoff Node
add_executable(takeoff_node src/takeoff_2_drones.cpp)
target_link_libraries(takeoff_node ${catkin_LIBRARIES})

# Land Node
add_executable(land_node src/land_2_drones.cpp)
target_link_libraries(land_node ${catkin_LIBRARIES})

# Command 2 drones Node
add_executable(command_node src/command_2_drones.cpp)
target_link_libraries(command_node ${catkin_LIBRARIES})

# Command 2 drones position controller Node
add_executable(Position_move_node src/PositionController_move.cpp)
target_link_libraries(Position_move_node ${catkin_LIBRARIES})

# Command 2 drones velocity controller Node
add_executable(command_node_2_pos src/VelocityController_2_drones.cpp)
target_link_libraries(command_node_2_pos ${catkin_LIBRARIES})

# Command 2 drones position controller Turn Node
add_executable(Position_turn_node src/PositionController_turn.cpp)
target_link_libraries(Position_turn_node ${catkin_LIBRARIES})

# Command 2 drones position controller Turn and Move Node
add_executable(Position_turn_and_move_node src/PositionController_turn_and_move.cpp)
target_link_libraries(Position_turn_and_move_node ${catkin_LIBRARIES})

# Command 2 drones position controller bar Turn and Move Node
add_executable(Position_bar_turn_and_move_node src/PositionController_bar_turn_and_move.cpp)
target_link_libraries(Position_bar_turn_and_move_node ${catkin_LIBRARIES})

# Command 2 drones Velocity controller Turn and Move Node
add_executable(Velocity_turn_and_move_node src/VelocityController_turn_and_move.cpp)
target_link_libraries(Velocity_turn_and_move_node ${catkin_LIBRARIES})

# Command 2 drones Velocity controller bar Turn and Move Node
add_executable(Velocity_bar_turn_and_move_node src/VelocityController_bar_turn_and_move.cpp)
target_link_libraries(Velocity_bar_turn_and_move_node ${catkin_LIBRARIES})

# Command 2 drones Velocity controller 2 masses Turn and Move Node
add_executable(Velocity_2_masses_turn_and_move_node src/VelocityController_2_masses_turn_and_move.cpp)
target_link_libraries(Velocity_2_masses_turn_and_move_node ${catkin_LIBRARIES})
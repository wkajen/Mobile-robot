cmake_minimum_required(VERSION 3.8)
project(motor_interface)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

include_directories(include)

add_executable(motor_interface_node src/motor_interface_node.cpp)
ament_target_dependencies(
    motor_interface_node
    rclcpp
    geometry_msgs
    sensor_msgs
)

target_include_directories(motor_interface_node
    PRIVATE
        ${Boost_INCLUDE_DIRS}
)

target_link_libraries(motor_interface_node 
    ${Boost_LIBRARIES}
)

install(TARGETS
    motor_interface_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
    launch
    config
    DESTINATION share/${PROJECT_NAME}/
)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(motor_interface)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(xacro REQUIRED)

include_directories(include)

add_executable(motor_interface_node src/motor_interface_node.cpp)
ament_target_dependencies(
    motor_interface_node
    rclcpp
    geometry_msgs
    sensor_msgs
)

add_executable(odometry_node src/odometry_node.cpp)
ament_target_dependencies(
    odometry_node 
    rclcpp 
    tf2_ros 
    nav_msgs 
    geometry_msgs 
    sensor_msgs
)

add_executable(initial_pose_publisher src/initial_pose_publisher.cpp)
ament_target_dependencies(
    initial_pose_publisher
    rclcpp
    geometry_msgs
)

add_executable(aht10_node src/aht10_node.cpp)
ament_target_dependencies(
    aht10_node
    rclcpp
    sensor_msgs
)

target_include_directories(motor_interface_node
    PRIVATE
        ${Boost_INCLUDE_DIRS}
)

target_link_libraries(motor_interface_node 
    ${Boost_LIBRARIES}
)

install(TARGETS odometry_node 
    DESTINATION lib/${PROJECT_NAME})

install(TARGETS motor_interface_node
    DESTINATION lib/${PROJECT_NAME})

install(TARGETS initial_pose_publisher
    DESTINATION lib/${PROJECT_NAME})

install(TARGETS aht10_node
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
    launch
    config
    urdf
    DESTINATION share/${PROJECT_NAME}/
)

# install(DIRECTORY
#     urdf
#     DESTINATION share/${PROJECT_NAME}
# )


ament_package()

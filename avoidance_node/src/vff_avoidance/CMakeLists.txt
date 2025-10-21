# CMake version and project name (nothing special)
cmake_minimum_required(VERSION 3.8)
project(vff_avoidance)

# Finding the corresponding dependencies that will be used throughout the project
# Mainly we use the find_package() to set the appropriate packages that will be used in the code while running nodes
# This helps the nodes acknowledge if they are using one of the packages found here
find_package(ament_cmake REQUIRED)
find_package(rclpcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# This "function" (let us name it like that for now) is used to reference/include directories by listing the directory names
include_directories(include)

# This creates an executable for the files listed in the add_executable function where the first parameter is the exectuable name that
# will be in the run command while testing. Example: ros2 launch/run <package_name> <exectuable_name>
# In this case the executable name is avoidance_vff_node
# In our case the package name is vff_avoidance
add_executable(avoidance_vff_node 
               src/avoidance_vff_main.cpp
               src/vff_avoidance/AvoidanceNode.cpp)

# Links the dependencies previously defined
ament_target_dependencies(avoidance_vff_node
                          rclpcpp
                          sensor_msgs
                          geometry_msgs
                          visualization_msgs
                          tf2_geometry_msgs
                          tf2
                          tf2_ros)

install(TARGETS avoidance_vff_node
        DESTINATION lib/${PROJECT_NAME})

# Installs the directories launc/ and config/ to the share directory
install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/
        DESTINATION share/${PROJECT_NAME}/config)

ament_package()
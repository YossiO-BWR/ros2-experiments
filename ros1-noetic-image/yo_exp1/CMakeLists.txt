cmake_minimum_required(VERSION 3.0.2)
project(yo_exp1)
add_compile_options(-std=c++17)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
)
catkin_package()
include_directories(
        ${catkin_INCLUDE_DIRS}
)
add_executable(node_1 node_1.cpp)
# set_target_properties(node_1 PROPERTIES OUTPUT_NAME node_1 PREFIX "")
add_dependencies(node_1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(node_1
        ${catkin_LIBRARIES}
)

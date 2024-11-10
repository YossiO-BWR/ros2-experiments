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
add_executable(node node.cpp)
set_target_properties(node PROPERTIES OUTPUT_NAME node PREFIX "")
add_dependencies(node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(node
        ${catkin_LIBRARIES}
)

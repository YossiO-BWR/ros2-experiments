cmake_minimum_required(VERSION 3.5)
project(yo_exp2)

# Default to C99
if (NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

add_library(composable_pub SHARED
        src/ComposablePub.cpp
)
ament_target_dependencies(composable_pub
        rclcpp
        rclcpp_components
        std_msgs
)

add_library(composable_sub SHARED
        src/ComposableSub.cpp
)
ament_target_dependencies(composable_sub
        rclcpp
        rclcpp_components
        std_msgs
)


install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}
)

install(TARGETS composable_pub
        EXPORT export_composable_pub
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(TARGETS composable_sub
        EXPORT export_composable_sub
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

rclcpp_components_register_node(
        composable_pub
        PLUGIN "yo_exp2::ComposablePub"
        EXECUTABLE comp_pub_node
)

rclcpp_components_register_node(
        composable_sub
        PLUGIN "yo_exp2::ComposableSub"
        EXECUTABLE comp_sub_node
)

ament_package()

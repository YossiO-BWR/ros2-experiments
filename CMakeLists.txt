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

add_executable(node_2 src/node_2.cpp)
ament_target_dependencies(node_2
        rclcpp
        std_msgs
)

install(TARGETS node_2
        DESTINATION lib/${PROJECT_NAME})

ament_package()

install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}
)


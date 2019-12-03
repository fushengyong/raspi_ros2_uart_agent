cmake_minimum_required(VERSION 3.5)
project(ros2_uart_agent)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-psabi)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ros2_control_interfaces REQUIRED)
find_library(wiringPi_LIB wiringPi)

add_library(helper_lib src/helpers.cpp)
ament_target_dependencies(helper_lib sensor_msgs)
target_include_directories(helper_lib PUBLIC include)
install(
  TARGETS helper_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_executable(raspi_pub src/main.cpp)
target_link_libraries(raspi_pub ${wiringPi_LIB} helper_lib)
ament_target_dependencies(
        raspi_pub
        rclcpp
        std_msgs
        sensor_msgs
        ros2_control_interfaces
)
install(TARGETS
        raspi_pub
        DESTINATION lib/${PROJECT_NAME})

ament_package()

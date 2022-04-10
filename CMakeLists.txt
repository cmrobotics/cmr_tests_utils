cmake_minimum_required(VERSION 3.5)
project(cmr_tests_utils)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)


include_directories(include
  ${rclcpp_lifecycle_INCLUDE_DIRS}
  ${lifecycle_msgs_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${rclcpp_action_INCLUDE_DIRS}
)

add_library(cmr_tests_utils_lib SHARED
  src/basic_node_test.cpp
  src/basic_action_client_test.cpp
  src/basic_service_client_test.cpp
  src/basic_subscriber_node_test.cpp
  src/basic_publisher_node_test.cpp
)

set(dependencies
  "rclcpp_lifecycle"
  "lifecycle_msgs"
  "rclcpp"
  "rclcpp_action"
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_libraries(cmr_tests_utils_lib)
ament_export_dependencies(${dependencies})

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NA}
)

ament_package()

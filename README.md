<div align="center">

# CMR Tests Utils

A header only library to facilitate writing `gtest` unit tests for ROS2 nodes.

</div>

<div align="center">

[![Percentage of issues still open](http://isitmaintained.com/badge/open/cmrobotics/cmr_tests_utils.svg)](http://isitmaintained.com/project/cmrobotics/cmr_tests_utils "Percentage of issues still open")
[![GitHub license](https://img.shields.io/github/license/cmrobotics/cmr_tests_utils.svg)](https://github.com/cmrobotics/cmr_tests_utils/blob/galactic-devel/LICENSE)
[![GitHub contributors](https://img.shields.io/github/contributors/cmrobotics/cmr_tests_utils.svg)](https://GitHub.com/cmrobotics/cmr_tests_utils/graphs/contributors/)
[![Maintenance](https://img.shields.io/badge/Maintained%3F-yes-green.svg)](https://GitHub.com/Naereen/StrapDown.js/graphs/commit-activity)
[![GitHub pull-requests](https://img.shields.io/github/issues-pr/Naereen/StrapDown.js.svg)](https://GitHub.com/Naereen/StrapDown.js/pull/)

</div>


## Table of Contents
* [General Info](#general-information)
* [Features](#features)
* [Setup](#setup)
* [Usage](#usage)
* [Room for Improvement](#room-for-improvement)
* [Contact](#contact)
* [License](#license)


## General Information

The library provides an easy templated API for testing basic communication nodes in the ROS2 ecosystem. It also provide spinner classes to implement a threaded `spin_some` call in a tight, safe loop checking `rclcpp::ok()`. The project intents to help checking communication between nodes that implements `services`, `actions`, `publishers` and `subscribers`. `Spinners` can be use to emmulate running all nodes in different threads, cancelling the spin of one or all nodes registered etc... 


## Features
List the ready features here:
- `Basic Subscriber`
- `Basic Publisher`
- `Basic Service Client`
- `Basic Action Client`
- `Basic Action Server`
- `Basic Service Server`
- `Single Threaded Spinner` - calls `spin_some` for all nodes registered in one different thread.
- `Multi Threaded Spinner` - calls `spin_some` for groups. Implements multiple instances of `SingleThreadSpinner`


## Setup

The project requires `set(CMAKE_CXX_STANDARD 20)` in your `CMakeLists.txt` to be built. To setup the library in your ROS2 project:

- Create your `test` folder in your project `root`.
- Add the following lines to your project `CMakeLists.txt`:
```cmake
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  add_subdirectory(test)
endif()
```
- Create your `gtest` cpp files in your test folder.
- Create a `CMakeLists.txt` in your test folder as following:
```cmake
set(dependencies
  ${dependencies}
  ${cmr_tests_utils}
)

find_package(cmr_tests_utils REQUIRED)

ament_add_gtest(name_of_your_test name_of_your_test.cpp)
target_link_libraries(name_of_your_test
  name_of_the_libs_you_want_to_link
)
ament_target_dependencies(name_of_your_test ${dependencies})
target_include_directories(name_of_your_test PRIVATE "../include" "include" ${cmr_tests_utils_INCLUDE_DIRS})
```

## Usage

### A Simple Pub/Sub Test
```cpp
rclcpp::init(0, nullptr);
auto spinner = cmr_tests_utils::SingleThreadSpinner();
auto sub_node = std::make_shared<cmr_tests_utils::BasicSubscriberNodeTest<std_msgs::msg::Int32>>("sub_test_node", "test_topic");
auto pub_node = std::make_shared<cmr_tests_utils::BasicPublisherNodeTest<std_msgs::msg::Int32>>("pub_test_node", "test_topic", false, 100);

EXPECT_FALSE(sub_node->has_data_been_received());

spinner.add_node(sub_node->get_node_base_interface());
spinner.add_node(pub_node->get_node_base_interface());

// Publish Data without Spinning
std_msgs::msg::Int32 msg;
msg.data = 42;
pub_node->publish(msg);

// Subscriber shouldn't have received anything
EXPECT_FALSE(sub_node->has_data_been_received());

// Spin both nodes
spinner.spin_some_all_nodes();
// Overhead for queued data population
std::this_thread::sleep_for(std::chrono::milliseconds(30));

EXPECT_TRUE(sub_node->has_data_been_received());
EXPECT_EQ(sub_node->get_received_msg().data, 42);

// Publish a message when both nodes are spinning
msg.data = 1337;
pub_node->publish(msg);
// Overhead for queued data population
std::this_thread::sleep_for(std::chrono::milliseconds(30));

EXPECT_EQ(sub_node->get_received_msg().data, 1337);

rclcpp::shutdown();
```

## Room for Improvement

Room for improvement:
- Integrate a Setup/Teardown Fixture for `rclcpp::init` and `rclcpp::shutdown`

To do:
- `CodeCov` and `CodeFactor` integration.


## License
This project is open source and available under the [Apache License, Version 2.0](https://www.apache.org/licenses/LICENSE-2.0).

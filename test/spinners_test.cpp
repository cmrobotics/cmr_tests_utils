#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "cmr_tests_utils/basic_publisher_node_test.hpp"
#include "cmr_tests_utils/basic_subscriber_node_test.hpp"
#include "std_msgs/msg/int32.hpp"
#include "cmr_tests_utils/single_thread_spinner.hpp"
#include <chrono>
#include <thread>

TEST(NodeCommunicationTest, simple_single_thread_spinning_check)
{
  rclcpp::init(0, nullptr);

  auto basic_node = std::make_shared<rclcpp::Node>("test_node");
  auto sts = cmr_tests_utils::SingleThreadSpinner();
  
  EXPECT_TRUE(sts.add_node(basic_node));
  EXPECT_FALSE(sts.add_node(basic_node));
  EXPECT_TRUE(sts.remove_node(basic_node));
  EXPECT_FALSE(sts.remove_node(basic_node));
  EXPECT_TRUE(sts.add_node(basic_node));
  EXPECT_FALSE(sts.are_all_nodes_spinning());

  EXPECT_TRUE(sts.spin_some_all_nodes());
  EXPECT_FALSE(sts.spin_some_all_nodes());

  EXPECT_TRUE(sts.are_all_nodes_spinning());

  EXPECT_TRUE(sts.cancel_all_spin());
  EXPECT_FALSE(sts.are_all_nodes_spinning());

  EXPECT_TRUE(sts.spin_some_all_nodes());
  EXPECT_TRUE(sts.are_all_nodes_spinning());

  EXPECT_TRUE(sts.cancel_all_spin());

  rclcpp::shutdown();

  EXPECT_FALSE(sts.are_all_nodes_spinning());
}

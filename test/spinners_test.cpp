#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "cmr_tests_utils/basic_publisher_node_test.hpp"
#include "cmr_tests_utils/basic_subscriber_node_test.hpp"
#include "std_msgs/msg/int32.hpp"
#include "cmr_tests_utils/single_thread_spinner.hpp"
#include "cmr_tests_utils/multi_thread_spinner.hpp"
#include <chrono>
#include <thread>

TEST(SpinnersTest, simple_single_thread_spinning_check)
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

TEST(SpinnersTest, simple_multi_thread_spinning_check)
{
  rclcpp::init(0, nullptr);

  auto basic_node1 = std::make_shared<rclcpp::Node>("test_node1");
  auto basic_node2 = std::make_shared<rclcpp::Node>("test_node2");
  auto basic_node3 = std::make_shared<rclcpp::Node>("test_node3");
  auto mts = cmr_tests_utils::MultiThreadSpinner();
  
  EXPECT_FALSE(mts.remove_node(basic_node1));
  EXPECT_FALSE(mts.is_node_spinning(basic_node2));
  EXPECT_FALSE(mts.spin_some_node(basic_node3));
  EXPECT_FALSE(mts.cancel_spin_node(basic_node1));

  EXPECT_TRUE(mts.add_node(basic_node1));
  EXPECT_FALSE(mts.is_node_spinning(basic_node1));
  EXPECT_FALSE(mts.cancel_spin_node(basic_node1));
  EXPECT_TRUE(mts.remove_node(basic_node1));
  EXPECT_TRUE(mts.add_node(basic_node1));
  EXPECT_TRUE(mts.spin_some_node(basic_node1));
  EXPECT_FALSE(mts.cancel_spin_node(basic_node2));
  EXPECT_TRUE(mts.cancel_spin_node(basic_node1));

  EXPECT_TRUE(mts.add_node(basic_node2));
  EXPECT_TRUE(mts.add_node(basic_node3));
  EXPECT_TRUE(mts.spin_some_node(basic_node1));
  EXPECT_TRUE(mts.spin_some_node(basic_node2));
  EXPECT_FALSE(mts.are_all_nodes_spinning());
  EXPECT_TRUE(mts.spin_some_node(basic_node3));
  EXPECT_TRUE(mts.are_all_nodes_spinning());
  EXPECT_TRUE(mts.cancel_all_spins());
  EXPECT_FALSE(mts.cancel_all_spins());
  EXPECT_TRUE(mts.spin_all_nodes());
  EXPECT_FALSE(mts.spin_all_nodes());

  rclcpp::shutdown();
}

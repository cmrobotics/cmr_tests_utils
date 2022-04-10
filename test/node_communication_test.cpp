#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "cmr_tests_utils/basic_node_test.hpp"
#include "cmr_tests_utils/basic_publisher_node_test.hpp"
#include "cmr_tests_utils/basic_subscriber_node_test.hpp"
#include "std_msgs/msg/int32.hpp"
#include <chrono>
#include <thread>

TEST(NodeCommunicationTest, simple_spinning_check)
{
  rclcpp::init(0, nullptr);

  auto basic_node = cmr_tests_utils::BasicNodeTest("test_node");
  EXPECT_FALSE(basic_node.get_is_spinning());

  basic_node.spin_in_new_thread();
  EXPECT_TRUE(basic_node.get_is_spinning());

  rclcpp::shutdown();
  
  EXPECT_FALSE(basic_node.get_is_spinning());
}

TEST(NodeCommunicationTest, pub_sub_communication)
{
  rclcpp::init(0, nullptr);

  auto sub_node = cmr_tests_utils::BasicSubscriberNodeTest<std_msgs::msg::Int32>("sub_test_node", "test_topic");
  auto pub_node = cmr_tests_utils::BasicPublisherNodeTest<std_msgs::msg::Int32>("pub_test_node", "test_topic", false, 100);
  EXPECT_FALSE(sub_node.get_is_spinning());
  EXPECT_FALSE(pub_node.get_is_spinning());
  EXPECT_FALSE(sub_node.has_data_been_received());

  // Publish Data without Spinning
  std_msgs::msg::Int32 msg;
  msg.data = 42;
  pub_node.publish(msg);

  // Subscriber shouldn't have received anything
  EXPECT_FALSE(sub_node.has_data_been_received());

  // Spin both nodes
  sub_node.spin_in_new_thread();
  pub_node.spin_in_new_thread();

  // Overhead for queued data population
  std::this_thread::sleep_for(std::chrono::milliseconds(30));

  // Check that node is spinning
  EXPECT_TRUE(pub_node.get_is_spinning());
  EXPECT_TRUE(sub_node.get_is_spinning());
  
  EXPECT_TRUE(sub_node.has_data_been_received());
  EXPECT_EQ(sub_node.get_received_msg().data, 42);
  
  // Publish a message when both nodes are spinning
  msg.data = 1337;
  pub_node.publish(msg);

  // Overhead for queued data population
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  
  EXPECT_EQ(sub_node.get_received_msg().data, 1337);
  
  rclcpp::shutdown();

  EXPECT_FALSE(sub_node.get_is_spinning());
  EXPECT_FALSE(pub_node.get_is_spinning());
}

TEST(NodeCommunicationTest, mismatched_topic_names)
{
  rclcpp::init(0, nullptr);

  auto sub_node = cmr_tests_utils::BasicSubscriberNodeTest<std_msgs::msg::Int32>("sub_test_node", "test_topics");
  auto pub_node = cmr_tests_utils::BasicPublisherNodeTest<std_msgs::msg::Int32>("pub_test_node", "test_topic", false, 100);
  EXPECT_FALSE(sub_node.get_is_spinning());
  EXPECT_FALSE(pub_node.get_is_spinning());
  EXPECT_FALSE(sub_node.has_data_been_received());

  // Spin both nodes
  sub_node.spin_in_new_thread();
  pub_node.spin_in_new_thread();

  // Check that node is spinning
  EXPECT_TRUE(pub_node.get_is_spinning());
  EXPECT_TRUE(sub_node.get_is_spinning());
  
  // Publish a message when both nodes are spinning
  std_msgs::msg::Int32 pub_msg;
  pub_msg.data = 1337;
  pub_node.publish(pub_msg);

  // Overhead for queued data population
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  
  auto msg = sub_node.get_received_msg();
  EXPECT_FALSE(sub_node.has_data_been_received());
  EXPECT_EQ(msg.data, 0);
  
  rclcpp::shutdown();

  EXPECT_FALSE(sub_node.get_is_spinning());
  EXPECT_FALSE(pub_node.get_is_spinning());
}

TEST(NodeCommunicationTest, publisher_with_timer)
{
  rclcpp::init(0, nullptr);

  auto sub_node = cmr_tests_utils::BasicSubscriberNodeTest<std_msgs::msg::Int32>("sub_test_node", "test_topic");
  auto pub_node = cmr_tests_utils::BasicPublisherNodeTest<std_msgs::msg::Int32>("pub_test_node", "test_topic", true, 100);
  EXPECT_FALSE(sub_node.get_is_spinning());
  EXPECT_FALSE(pub_node.get_is_spinning());
  EXPECT_FALSE(sub_node.has_data_been_received());

  // Spin both nodes
  sub_node.spin_in_new_thread();
  pub_node.spin_in_new_thread();

  // Check that node is spinning
  EXPECT_TRUE(pub_node.get_is_spinning());
  EXPECT_TRUE(sub_node.get_is_spinning());

  // Overhead for queued data population
  std::this_thread::sleep_for(std::chrono::milliseconds(250));

  EXPECT_FALSE(sub_node.has_data_been_received());

  // Set timer handled published msg
  std_msgs::msg::Int32 msg;
  msg.data = 11;
  pub_node.set_published_msg(msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  auto rec_msg = sub_node.get_received_msg();
  EXPECT_EQ(rec_msg.data, msg.data);

  msg.data = 23;
  pub_node.set_published_msg(msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  rec_msg = sub_node.get_received_msg();
  EXPECT_EQ(rec_msg.data, msg.data);
  
  rclcpp::shutdown();

  EXPECT_FALSE(sub_node.get_is_spinning());
  EXPECT_FALSE(pub_node.get_is_spinning());
}


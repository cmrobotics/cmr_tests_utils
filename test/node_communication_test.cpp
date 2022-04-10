#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "cmr_tests_utils/basic_node_test.hpp"
#include "cmr_tests_utils/basic_publisher_node_test.hpp"
#include "cmr_tests_utils/basic_subscriber_node_test.hpp"
#include "std_msgs/msg/int32.hpp"

TEST(NodeCommunicationTest, simple_spinning_check)
{
  rclcpp::init(0, nullptr);

  auto basic_node = cmr_tests_utils::BasicNodeTest("test_node");
  EXPECT_FALSE(basic_node.get_is_spinning());
  basic_node.spin_in_new_thread();
  EXPECT_TRUE(basic_node.get_is_spinning());

  rclcpp::shutdown();
  basic_node.cancel_spin();
}

TEST(NodeCommunicationTest, pub_sub_communication)
{
  rclcpp::init(0, nullptr);

  auto sub_node = cmr_tests_utils::BasicSubscriberNodeTest<std_msgs::msg::Int32>("sub_test_node", "test_topic");
  auto pub_node = cmr_tests_utils::BasicPublisherNodeTest<std_msgs::msg::Int32>("pub_test_node", "test_topic", false, 100);
  EXPECT_FALSE(sub_node.get_is_spinning());
  EXPECT_FALSE(pub_node.get_is_spinning());
  EXPECT_EQ(sub_node.get_received_msg(), nullptr);

  // Publish Data without Spinning
  std_msgs::msg::Int32 msg;
  msg.data = 42;
  pub_node.publish(msg);
  sleep(2);

  // Subscriber shouldn't have received anything
  EXPECT_TRUE(!sub_node.get_received_msg());

  // Spin both nodes
  sub_node.spin_in_new_thread();
  pub_node.spin_in_new_thread();

  // We expect the data to be received now
  EXPECT_TRUE(pub_node.get_is_spinning());
  EXPECT_TRUE(sub_node.get_is_spinning());
  EXPECT_FALSE(!sub_node.get_received_msg());
  EXPECT_EQ(sub_node.get_received_msg()->data, 42);
  
  // Publish a message when both nodes are spinning
  msg.data = 1337;
  pub_node.publish(msg);
  sleep(2);
  EXPECT_FALSE(!sub_node.get_received_msg());
  EXPECT_EQ(sub_node.get_received_msg()->data, 1337);

  rclcpp::shutdown();
  sub_node.cancel_spin();
  pub_node.cancel_spin();
}

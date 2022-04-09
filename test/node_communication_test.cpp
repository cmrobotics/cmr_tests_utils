#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "cmr_tests_utils/basic_node_test.hpp"
#include "cmr_tests_utils/basic_publisher_node_test.hpp"
#include "cmr_tests_utils/basic_subscriber_node_test.hpp"
#include "std_msgs/msg/int32.hpp"

TEST(NodeCommunicationTest, spinning_test)
{
  rclcpp::init(0, nullptr);

  auto basic_node = cmr_tests_utils::BasicNodeTest("test_node");
  EXPECT_FALSE(basic_node.get_is_spinning());
  basic_node.spin_some();
  EXPECT_TRUE(basic_node.get_is_spinning());

  rclcpp::shutdown();
  basic_node.cancel_spin();
}

TEST(NodeCommunicationTest, is_node_spinning)
{
  rclcpp::init(0, nullptr);

  auto sub_node = cmr_tests_utils::BasicSubscriberNodeTest<std_msgs::msg::Int32>("sub_test_node", "test_topic");
  auto pub_node = cmr_tests_utils::BasicPublisherNodeTest<std_msgs::msg::Int32>("pub_test_node", "test_topic", false, 100);
  sub_node.spin_some();
  pub_node.spin_some();
  EXPECT_FALSE(sub_node.has_data_been_received());

  std_msgs::msg::Int32 msg;
  msg.data = 42;
  pub_node.publish(msg);
  sleep(2);
  EXPECT_TRUE(sub_node.has_data_been_received());
  EXPECT_EQ(sub_node.get_received_msg().data, 42);

  msg.data = 1337;
  pub_node.publish(msg);
  sleep(2);
  EXPECT_TRUE(sub_node.has_data_been_received());
  EXPECT_EQ(sub_node.get_received_msg().data, 1337);

  rclcpp::shutdown();
  sub_node.cancel_spin();
  pub_node.cancel_spin();
}

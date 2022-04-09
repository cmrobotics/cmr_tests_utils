#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "cmr_tests_utils/basic_node_test.hpp"
#include "cmr_tests_utils/basic_publisher_node_test.hpp"
#include "cmr_tests_utils/basic_subscriber_node_test.hpp"
#include "std_msgs/Int32.hpp"

TEST(NodeCommunicationTest, spinning_test)
{
  rclcpp::init(0, nullptr);

  auto basic_node = BasicNodeTest("test_node");
  EXPECT_FALSE(basic_node.is_spinning);
  basic_node.spin_some();
  EXPECT_TRUE(basic_node.is_spinning);

  rclcpp::shutdown();
  basic_node.cancel_spin();
}

TEST(NodeCommunicationTest, is_node_spinning)
{
  rclcpp::init(0, nullptr);

  auto sub_node<std_msgs::msg::Int32> = BasicSubscriberNodeTest("sub_test_node", "test_topic");
  auto pub_node<std_msgs::msg::Int32> = BasicPublisherNodeTest("pub_test_node", "test_topic", false, 100ms);
  sub_node.spin_some();
  pub_node.spin_some();
  EXPECT_FALSE(sub_node.has_data_been_received());

  std_msgs::Int32 msg;
  msg.data = 42;
  pub_node.publish(msg);
  std::sleep(2);
  EXPECT_TRUE(sub_node.has_data_been_received());
  EXPECT_EQ(sub_node.get_received_msg().data, 42);

  msg.data = 1337;
  pub_node.publish(msg);
  std::sleep(2);
  EXPECT_TRUE(sub_node.has_data_been_received());
  EXPECT_EQ(sub_node.get_received_msg().data, 1337);

  rclcpp::shutdown();
  sub_node.cancel_spin();
  pub_node.cancel_spin();
}

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "cmr_tests_utils/basic_publisher_node_test.hpp"
#include "cmr_tests_utils/basic_subscriber_node_test.hpp"
#include "cmr_tests_utils/single_thread_spinner.hpp"
#include "std_msgs/msg/int32.hpp"
#include <chrono>
#include <thread>

TEST(PubSubCommunication, pub_sub_communication)
{
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
}

TEST(PubSubCommunication, mismatched_topic_names)
{
  rclcpp::init(0, nullptr);

  auto spinner_sub = cmr_tests_utils::SingleThreadSpinner();
  auto spinner_pub = cmr_tests_utils::SingleThreadSpinner();
  auto sub_node = std::make_shared<cmr_tests_utils::BasicSubscriberNodeTest<std_msgs::msg::Int32>>("sub_test_node", "test_topics");
  auto pub_node = std::make_shared<cmr_tests_utils::BasicPublisherNodeTest<std_msgs::msg::Int32>>("pub_test_node", "test_topic", false, 100);
  
  spinner_sub.add_node(sub_node->get_node_base_interface());
  spinner_pub.add_node(pub_node->get_node_base_interface());
  
  EXPECT_FALSE(sub_node->has_data_been_received());

  // Spin both nodes
  spinner_sub.spin_some_all_nodes();
  spinner_pub.spin_some_all_nodes();
  
  // Publish a message when both nodes are spinning
  std_msgs::msg::Int32 pub_msg;
  pub_msg.data = 1337;
  pub_node->publish(pub_msg);

  // Overhead for queued data population
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  
  auto msg = sub_node->get_received_msg();
  EXPECT_FALSE(sub_node->has_data_been_received());
  EXPECT_EQ(msg.data, 0);
  
  rclcpp::shutdown();
}

TEST(PubSubCommunication, publisher_with_timer)
{
  rclcpp::init(0, nullptr);
  std::this_thread::sleep_for(std::chrono::milliseconds(30));

  auto spinner = cmr_tests_utils::SingleThreadSpinner();
  auto sub_node = std::make_shared<cmr_tests_utils::BasicSubscriberNodeTest<std_msgs::msg::Int32>>("sub_test_node", "test_topic");
  auto pub_node = std::make_shared<cmr_tests_utils::BasicPublisherNodeTest<std_msgs::msg::Int32>>("pub_test_node", "test_topic", true, 100);
  
  
  EXPECT_FALSE(sub_node->has_data_been_received());

  // Spin both nodes
  spinner.add_node(pub_node->get_node_base_interface());
  spinner.add_node(sub_node->get_node_base_interface());

  spinner.spin_some_all_nodes();

  // Overhead for queued data population
  std::this_thread::sleep_for(std::chrono::milliseconds(250));

  EXPECT_FALSE(sub_node->has_data_been_received());

  // Set timer handled published msg
  std_msgs::msg::Int32 msg;
  msg.data = 11;
  pub_node->set_published_msg(msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  auto rec_msg = sub_node->get_received_msg();
  EXPECT_EQ(rec_msg.data, msg.data);

  msg.data = 23;
  pub_node->set_published_msg(msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  rec_msg = sub_node->get_received_msg();
  EXPECT_EQ(rec_msg.data, msg.data);
  
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  rclcpp::shutdown();
}

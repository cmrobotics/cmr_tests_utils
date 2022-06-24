#include <chrono>
#include <thread>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include "cmr_tests_utils/single_thread_spinner.hpp"
#include "cmr_tests_utils/multi_thread_spinner.hpp"
#include "cmr_tests_utils/basic_tf_listener_node_test.hpp"
#include "cmr_tests_utils/basic_tf_broadcaster_node_test.hpp"

TEST(TransformsTest, simple_tf_broadcaster_instantiation)
{
  rclcpp::init(0, nullptr);

  auto spinner = cmr_tests_utils::SingleThreadSpinner();

  geometry_msgs::msg::TransformStamped tf;
  auto tf_broadcaster_node = std::make_shared<cmr_tests_utils::BasicTfBroadcasterNodeTest>("tf_broadcaster_node", tf, 100);
  spinner.add_node(tf_broadcaster_node->get_node_base_interface());
  
  spinner.spin_some_all_nodes();

  geometry_msgs::msg::TransformStamped tf_empty;
  ASSERT_EQ(tf_broadcaster_node->get_transform(), tf_empty);
  ASSERT_EQ(tf_broadcaster_node->get_transform().header.frame_id, "");

  tf.header.frame_id = "world";
  tf_broadcaster_node->set_transform(tf);

  ASSERT_NE(tf_broadcaster_node->get_transform(), tf_empty);
  ASSERT_EQ(tf_broadcaster_node->get_transform().header.frame_id, "world");

  rclcpp::shutdown();
}


TEST(TransformsTest, simple_tf_listener_instantiation)
{
  rclcpp::init(0, nullptr);

  auto spinner = cmr_tests_utils::SingleThreadSpinner();
  auto tf_listener_node = std::make_shared<cmr_tests_utils::BasicTfListenerNodeTest>("tf_listener_node", 1.0);
  spinner.add_node(tf_listener_node->get_node_base_interface());
  spinner.spin_some_all_nodes();

  geometry_msgs::msg::TransformStamped tf;
  ASSERT_FALSE(tf_listener_node->lookup_transform(tf, "rdn", "ndr"));

  geometry_msgs::msg::TransformStamped tf_empty;
  ASSERT_EQ(tf, tf_empty);

  geometry_msgs::msg::PoseStamped source, target;
  source.header.frame_id = "ndr";
  ASSERT_FALSE(tf_listener_node->transform_pose("rdn", source, target));

  ASSERT_NE(target.header.frame_id, "rdn");
  ASSERT_EQ(target.header.frame_id, "");

  rclcpp::shutdown();
}

TEST(TransformsTest, listen_to_broadcasted_frames)
{
  rclcpp::init(0, nullptr);

  auto spinner = cmr_tests_utils::MultiThreadSpinner();

  // Setup Transform from World to Base
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = rclcpp::Time();
  tf.header.frame_id = "world";
  tf.child_frame_id = "base";
  tf.transform.translation.x = 5;
  
  auto tf_listener_node = std::make_shared<cmr_tests_utils::BasicTfListenerNodeTest>("tf_listener_node", 2.0);
  auto tf_broadcaster_node = std::make_shared<cmr_tests_utils::BasicTfBroadcasterNodeTest>("tf_broadcaster_node", tf, 20);
  spinner.add_node(tf_broadcaster_node->get_node_base_interface());
  spinner.add_node(tf_listener_node->get_node_base_interface());
  spinner.spin_all_nodes();

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  geometry_msgs::msg::TransformStamped test_tf;
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "world";
  pose.pose.position.x = 5;
  ASSERT_TRUE(tf_listener_node->lookup_transform(test_tf, "base", "world"));
  
  ASSERT_EQ(test_tf.transform.translation.x, -5);

  geometry_msgs::msg::PoseStamped source, target;
  source.header.frame_id = "world";
  ASSERT_TRUE(tf_listener_node->transform_pose("base", source, target));

  ASSERT_EQ(target.pose.position.x, -5);

  rclcpp::shutdown();
}

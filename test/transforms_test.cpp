#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "cmr_tests_utils/basic_tf_listener_node_test.hpp"
#include "cmr_tests_utils/basic_tf_broadcaster_node_test.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "cmr_tests_utils/single_thread_spinner.hpp"
#include <chrono>
#include <thread>

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
  auto tf_listener_node = std::make_shared<cmr_tests_utils::BasicTfListenerNodeTest>("tf_listener_node", 0.1);
  spinner.add_node(tf_listener_node->get_node_base_interface());
  spinner.spin_some_all_nodes();

  geometry_msgs::msg::TransformStamped tf;
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "ndr";
  ASSERT_FALSE(tf_listener_node->lookup_transform(tf, "rdn", pose));

  geometry_msgs::msg::TransformStamped tf_empty;
  ASSERT_EQ(tf, tf_empty);

  geometry_msgs::msg::PoseStamped source, target, pose_empty;
  ASSERT_FALSE(tf_listener_node->transform_pose("rdn", source, target));

  RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Help");

  ASSERT_EQ(pose_empty, target);
  ASSERT_EQ(pose_empty, source);

  rclcpp::shutdown();
}

TEST(TransformsTest, listen_to_broadcasted_frames)
{
  rclcpp::init(0, nullptr);

  auto spinner = cmr_tests_utils::SingleThreadSpinner();

  // Setup Transform from World to Base
  geometry_msgs::msg::TransformStamped tf;
  geometry_msgs::msg::Quaternion quat_msg;
  tf2::Quaternion tf_quat;
  quat_msg = tf2::toMsg(tf2_quat);
  tf.header.stamp = rclcpp::Time();
  tf.header.frame_id = "world";
  tf.child_frame_id = "base";
  tf.transform.translation.x = 5;
  
  auto tf_listener_node = std::make_shared<cmr_tests_utils::BasicTfListenerNodeTest>("tf_listener_node", 0.1);
  auto tf_broadcaster_node = std::make_shared<cmr_tests_utils::BasicTfBroadcasterNodeTest>("tf_broadcaster_node", tf, 100);
  spinner.add_node(tf_broadcaster_node->get_node_base_interface());
  spinner.add_node(tf_listener_node->get_node_base_interface());
  spinner.spin_some_all_nodes();

  geometry_msgs::msg::TransformStamped test_tf;
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "world";
  pose.pose.position.x = 5;
  ASSERT_FALSE(tf_listener_node->lookup_transform(test_tf, "base", pose));
  
  ASSERT_EQ(test_tf.transform.translation.x, 5);

  geometry_msgs::msg::PoseStamped source, target, pose_empty;
  ASSERT_FALSE(tf_listener_node->transform_pose("rdn", source, target));

  RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Help");

  ASSERT_EQ(pose_empty, target);
  ASSERT_EQ(pose_empty, source);

  rclcpp::shutdown();
}

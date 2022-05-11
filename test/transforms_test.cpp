#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "cmr_tests_utils/basic_tf_listener_node_test.hpp"
#include "cmr_tests_utils/basic_tf_broadcaster_node_test.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "cmr_tests_utils/single_thread_spinner.hpp"
#include <chrono>
#include <thread>

TEST(TransformsTest, simple_tf_listener_instantiation)
{
  rclcpp::init(0, nullptr);

  auto spinner = cmr_tests_utils::SingleThreadSpinner();
  auto tf_listener_node = std::make_shared<cmr_tests_utils::BasicTfListenerNodeTest>("tf_listener_node", 0.1);
  spinner.add_node(tf_listener_node->get_node_base_interface());
  spinner.spin_some_all_nodes();

  geometry_msgs::msg::TransformStamped tf;
  ASSERT_FALSE(tf_listener_node->lookup_transform(tf, "rdn", "ndr"));

  geometry_msgs::msg::TransformStamped tf_empty;
  ASSERT_EQ(tf, tf_empty);

  geometry_msgs::msg::PoseStamped source, target, pose_empty;
  ASSERT_FALSE(tf_listener_node->transform_pose("rdn", source, target));
  ASSERT_EQ(pose_empty, target);
  ASSERT_EQ(pose_empty, source);

  rclcpp::shutdown();
}

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

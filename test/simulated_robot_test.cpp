#include <chrono>
#include <thread>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>


#include "cmr_tests_utils/basic_publisher_node_test.hpp"
#include "cmr_tests_utils/simulated_differential_robot.hpp"
#include "cmr_tests_utils/basic_tf_listener_node_test.hpp"
#include "cmr_tests_utils/multi_thread_spinner.hpp"

TEST(SimulatedRobot, robot_has_not_moved)
{
  rclcpp::init(0, nullptr);

  auto spinner = cmr_tests_utils::MultiThreadSpinner();
  auto tf_listener = std::make_shared<cmr_tests_utils::BasicTfListenerNodeTest>("tf_listener_node", 1.0);
  auto robot = std::make_shared<cmr_tests_utils::SimulatedDifferentialRobot>("robot_node", "odom", "base_footprint", "cmd_vel");

  spinner.add_node(tf_listener->get_node_base_interface());
  spinner.add_node(robot->get_node_base_interface());
  
  ASSERT_TRUE(spinner.spin_all_nodes());

  // Test if robot has been constructed correctly
  ASSERT_FALSE(robot->has_vel_been_received());
  ASSERT_EQ(robot->get_transform().header.frame_id, "odom");
  ASSERT_EQ(robot->get_transform().child_frame_id, "base_footprint");

  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  ASSERT_TRUE(robot->is_broadcasting());

  // Test if external nodes can access the Robot's state
  geometry_msgs::msg::TransformStamped test_tf;
  ASSERT_TRUE(tf_listener->lookup_transform(test_tf, "odom", "base_footprint"));
  ASSERT_EQ(test_tf.transform.translation.x, 0);
  geometry_msgs::msg::PoseStamped source, target, pose;
  source.header.frame_id = "odom";
  source.pose.position.x = 5;
  ASSERT_TRUE(tf_listener->transform_pose("base_footprint", source, target));
  ASSERT_EQ(target.pose.position.x, 5);
  
  rclcpp::shutdown();
}

TEST(SimulatedRobot, robot_receives_exactly_one_command)
{
  rclcpp::init(0, nullptr);

  auto spinner = cmr_tests_utils::MultiThreadSpinner();
  auto robot_spinner = cmr_tests_utils::SingleThreadSpinner();
  auto velocity_pub = std::make_shared<cmr_tests_utils::BasicPublisherNodeTest<geometry_msgs::msg::Twist>>("velocity_commander_node", "cmd_vel", false);
  auto tf_listener = std::make_shared<cmr_tests_utils::BasicTfListenerNodeTest>("tf_listener_node", 1.0);
  auto robot = std::make_shared<cmr_tests_utils::SimulatedDifferentialRobot>("robot_node", "odom", "base_footprint", "cmd_vel");

  spinner.add_node(velocity_pub->get_node_base_interface());
  spinner.add_node(tf_listener->get_node_base_interface());
  spinner.add_node(robot->get_node_base_interface());
  
  ASSERT_TRUE(spinner.spin_all_nodes());

  // Test if robot has been constructed correctly
  ASSERT_FALSE(robot->has_vel_been_received());
  ASSERT_EQ(robot->get_transform().header.frame_id, "odom");
  ASSERT_EQ(robot->get_transform().child_frame_id, "base_footprint");

  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  ASSERT_TRUE(robot->is_broadcasting());

  // Test if external nodes can access the Robot's state
  geometry_msgs::msg::TransformStamped test_tf;
  ASSERT_TRUE(tf_listener->lookup_transform(test_tf, "odom", "base_footprint"));
  ASSERT_EQ(test_tf.transform.translation.x, 0);
  geometry_msgs::msg::PoseStamped source, target, pose;
  source.header.frame_id = "odom";
  source.pose.position.x = 5;
  ASSERT_TRUE(tf_listener->transform_pose("base_footprint", source, target));
  ASSERT_EQ(target.pose.position.x, 5);

  // Publish exactly one velocity
  geometry_msgs::msg::Twist twist;
  twist.linear.x = 1.0;
  velocity_pub->publish(twist);

  // Wait for the robot to move
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  
  // The robot should've moved at 1.0 m/s for 0.5s, it should be 0.5m forward
  ASSERT_TRUE(robot->has_vel_been_received());
  ASSERT_EQ(robot->get_transform().header.frame_id, "odom");
  ASSERT_EQ(robot->get_transform().child_frame_id, "base_footprint");
  ASSERT_NEAR(robot->get_transform().transform.translation.x, 0.5, 0.01);
  ASSERT_NEAR(robot->get_transform().transform.translation.y, 0.0, 0.01);
  ASSERT_NEAR(robot->get_transform().transform.rotation.x, 0.0, 0.01);
  ASSERT_NEAR(robot->get_transform().transform.rotation.y, 0.0, 0.01);
  ASSERT_NEAR(robot->get_transform().transform.rotation.z, 0.0, 0.01);
  ASSERT_NEAR(robot->get_transform().transform.rotation.w, 1.0, 0.01);

  // Send new command
  twist.linear.x = 0;
  twist.angular.z = 3.14;
  velocity_pub->publish(twist);

  // Wait for the robot to move
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  ASSERT_TRUE(robot->has_vel_been_received());
  ASSERT_EQ(robot->get_transform().header.frame_id, "odom");
  ASSERT_EQ(robot->get_transform().child_frame_id, "base_footprint");
  ASSERT_NEAR(robot->get_transform().transform.translation.x, 0.5, 0.01);
  ASSERT_NEAR(robot->get_transform().transform.translation.y, 0.0, 0.01);
  ASSERT_NEAR(robot->get_transform().transform.rotation.x, 0.0, 0.01);
  ASSERT_NEAR(robot->get_transform().transform.rotation.y, 0.0, 0.01);
  ASSERT_NEAR(robot->get_transform().transform.rotation.z, 0.707, 0.01);
  ASSERT_NEAR(robot->get_transform().transform.rotation.w, 0.707, 0.01);
  
  rclcpp::shutdown();
}

#ifndef BASIC_TF_BROADCASTER_NODE_TEST_HPP
#define BASIC_TF_BROADCASTER_NODE_TEST_HPP
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>

namespace cmr_tests_utils
{

class BasicTfBroadcasterNodeTest: public rclcpp::Node {

  public:

  BasicTfBroadcasterNodeTest(std::string node_name, geometry_msgs::msg::TransformStamped initial_transform, 
                             unsigned int publish_period_ms = 100, bool use_timer = true): rclcpp::Node(node_name)
  {
    clock_ = get_clock();
    buffer_ = std::make_shared<tf2_ros::Buffer>(clock_);
    listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
    broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    transform_ = initial_transform;
    if (use_timer) timer_ = this->create_wall_timer(
      std::chrono::milliseconds(publish_period_ms), 
      std::bind(&BasicTfBroadcasterNodeTest::broadcast_transform, this)
    );
  }

  const geometry_msgs::msg::TransformStamped& get_transform () const
  {
    std::lock_guard<std::mutex> lock(tf_mutex_);
    return transform_;
  }

  void set_transform(const geometry_msgs::msg::TransformStamped & transform) 
  {
    std::lock_guard<std::mutex> lock(tf_mutex_);
    transform_ = transform;
  }

  void broadcast_transform() 
  {
    std::lock_guard<std::mutex> lock(tf_mutex_);
    transform_.header.stamp = clock_->now();
    broadcaster_->sendTransform(transform_);
  }

  private:

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Duration transform_tolerance_ {0, 0};
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  geometry_msgs::msg::TransformStamped transform_;
  rclcpp::Clock::SharedPtr clock_;
  mutable std::mutex tf_mutex_;
};

}

#endif
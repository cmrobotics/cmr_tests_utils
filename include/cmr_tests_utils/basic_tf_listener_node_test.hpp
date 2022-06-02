#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>

namespace cmr_tests_utils
{

class BasicTfListenerNodeTest: public rclcpp::Node {

  public:

  BasicTfListenerNodeTest(std::string node_name, double transform_tolerance): rclcpp::Node(node_name)
  {
    clock_ = get_clock();
    buffer_ = std::make_shared<tf2_ros::Buffer>(clock_);
    listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
    transform_tolerance_ = rclcpp::Duration::from_seconds(transform_tolerance);
  }

  bool transform_pose(const std::string target_frame,
    const geometry_msgs::msg::PoseStamped & in_pose, geometry_msgs::msg::PoseStamped & out_pose) const
  {
    if (in_pose.header.frame_id == target_frame) {
      out_pose = in_pose;
      return true;
    }

    try {
      if (!buffer_->canTransform(target_frame, in_pose.header.frame_id, rclcpp::Time(), transform_tolerance_)) {
        RCLCPP_ERROR(get_logger(), "Transform between %s and %s was not available", in_pose.header.frame_id.c_str(), target_frame.c_str());
        return false;
      } 
      // Transform is a blocking call that will hang forever if frames don't exist, so we call canTransform first
      buffer_->transform(in_pose, out_pose, target_frame);
      return true;
    } catch (tf2::ExtrapolationException & ex) {
      geometry_msgs::msg::TransformStamped transform;

      if (this->lookup_transform(transform, target_frame, in_pose.header.frame_id)) 
      {
        tf2::doTransform(in_pose, out_pose, transform);
        return true;
      }
    }

    return false;
  }

  bool lookup_transform(geometry_msgs::msg::TransformStamped & transform, std::string target_frame, std::string source_frame) const
  {
    try {
      if (!buffer_->canTransform(target_frame, source_frame, rclcpp::Time(), transform_tolerance_)) {
        RCLCPP_ERROR(get_logger(), "Transform between %s and %s was not available", source_frame.c_str(), target_frame.c_str());
        return false;
      } 
      
      // Lookup Transform is a blocking call that will hang forever if frames don't exist, so we call canTransform first
      transform = buffer_->lookupTransform(target_frame, source_frame, rclcpp::Time(), transform_tolerance_);

    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(
        rclcpp::get_logger("tf_help"),
        "Exception in transform_pose: %s",
        ex.what()
      );
      return false;
    }
    
    return true;
  }

  private:

  // ROS
  rclcpp::Duration transform_tolerance_ {0, 0};
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::Clock::SharedPtr clock_;
};

}

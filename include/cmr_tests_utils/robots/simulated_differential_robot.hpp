#ifndef SIMULATED_DIFFERENTIAL_ROBOT
#define SIMULATED_DIFFERENTIAL_ROBOT

#include <chrono>
#include "cmr_tests_utils/synchronous_nodes/basic_tf_broadcaster_node_test.hpp"
#include "cmr_tests_utils/synchronous_nodes/basic_subscriber_node_test.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace cmr_tests_utils {

class SimulatedDifferentialRobot: public rclcpp::Node 
{
  public:  

  SimulatedDifferentialRobot(std::string node_name, std::string global_frame, std::string base_frame, std::string velocity_topic,
                          double velocity_timeout_sec = 0.5,
                          rclcpp::QoS velocity_topic_qos = rclcpp::SystemDefaultsQoS()): rclcpp::Node(node_name)
  {
    clock_ = get_clock();
    buffer_ = std::make_shared<tf2_ros::Buffer>(clock_);
    listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
    broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    global_frame_ = global_frame;
    base_frame_ = base_frame;  
    velocity_timeout_sec_ = velocity_timeout_sec;

    transform_.header.frame_id = global_frame;
    transform_.child_frame_id = base_frame;

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(int(broadcast_period_sec_*1000)), 
      std::bind(&SimulatedDifferentialRobot::broadcast_transform_, this)
    );  

    vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist> (
              velocity_topic, velocity_topic_qos,
              std::bind (&SimulatedDifferentialRobot::vel_callback_, this, std::placeholders::_1));
  }  

  bool has_vel_been_received() const 
  {
    return (last_received_vel_ != nullptr);
  }  

  const bool& is_broadcasting() const
  {
    return is_broadcasting_;
  }

  const geometry_msgs::msg::TransformStamped& get_transform () const
  {
    return transform_;
  }

  geometry_msgs::msg::Twist get_last_velocity_msg() const
  {
    geometry_msgs::msg::Twist msg;
    if (!last_received_vel_) 
    {
      RCLCPP_WARN(get_logger(), "Tried to get last received velocity from subscription but nothing was published yet.");
      return msg;
    }
    return *last_received_vel_;
  }  


  private:  

  void broadcast_transform_() 
  {
    is_broadcasting_ = true;

    if (last_received_vel_)
    {
      std::chrono::duration<double> elapsed_time = std::chrono::system_clock::now() - last_velocity_timestamp_;

      // Last received command was too long ago, the robot stops
      if (elapsed_time.count() > velocity_timeout_sec_)
      {
        transform_.header.stamp = clock_->now();
        broadcaster_->sendTransform(transform_);
        return;
      }
      
      // Estimate pose using last command and dt since 
      transform_.transform.translation.x += broadcast_period_sec_ * last_received_vel_->linear.x * cos(last_yaw_);
      transform_.transform.translation.y += broadcast_period_sec_ * last_received_vel_->linear.y * sin(last_yaw_);
      last_yaw_ += broadcast_period_sec_ * last_received_vel_->angular.z;  
      tf2::Quaternion quat;
      quat.setRPY(0, 0, last_yaw_);
      transform_.transform.rotation = tf2::toMsg(quat);
    }

    transform_.header.stamp = clock_->now();
    broadcaster_->sendTransform(transform_);
  }  

  void vel_callback_(const std::shared_ptr<geometry_msgs::msg::Twist> msg) 
  {
    last_received_vel_ = msg;
    last_velocity_timestamp_ = std::chrono::system_clock::now();
  }

  // Transforms Utils
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::TransformStamped transform_;
  rclcpp::Duration transform_tolerance_ {0, 0};
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::Clock::SharedPtr clock_;  

  // Subscription Utils
  typename rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
  std::shared_ptr<geometry_msgs::msg::Twist> last_received_vel_;
  double last_yaw_;  

  // Internal Attributes
  std::string base_frame_;
  std::string global_frame_;
  std::chrono::system_clock::time_point last_velocity_timestamp_; // As long as we don't have TwistStamped for velocities, it's the best we can do
  bool is_broadcasting_;
  double velocity_timeout_sec_;
  double broadcast_period_sec_ = 0.01; // 100 Hz
};

}

#endif
#pragma once

#include <mutex>
#include <chrono>
#include <thread>

#include <rclcpp/rclcpp.hpp>

namespace cmr_tests_utils {

template<class MessageT>
class BasicPublisherNodeTest: public rclcpp::Node {

  public:
  
  BasicPublisherNodeTest(std::string node_name, std::string topic_name, bool use_timer,
                          unsigned int publish_period_ms = 100,
                          rclcpp::QoS qos = rclcpp::SystemDefaultsQoS())
    : rclcpp::Node(node_name)
  {
    topic_pub_ = this->create_publisher<MessageT> (topic_name, qos);
    if (use_timer) timer_ = this->create_wall_timer(
                              std::chrono::milliseconds(publish_period_ms), 
                              std::bind(&BasicPublisherNodeTest::publish_, this)
                            );
  }

  void set_published_msg(const MessageT& msg)
  {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    if (!published_msg_) published_msg_ = std::make_shared<MessageT>();
    *published_msg_ = msg;
  }

  void publish(const MessageT& msg)
  {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    topic_pub_->publish(msg);
  }

  private:

  void publish_()
  {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    if (!published_msg_)
    {
      RCLCPP_ERROR(get_logger(), "Failed to publish message, attribute was not set!");
      return;
    }
    topic_pub_->publish(*published_msg_);
  }

  typename rclcpp::Publisher<MessageT>::SharedPtr topic_pub_;
  std::shared_ptr<MessageT> published_msg_;
  rclcpp::TimerBase::SharedPtr timer_;
  mutable std::mutex msg_mutex_;
};

}

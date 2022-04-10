#ifndef BASIC_PUBLISHER_NODE_TEST_HPP
#define BASIC_PUBLISHER_NODE_TEST_HPP

#include "rclcpp/rclcpp.hpp"
#include "cmr_tests_utils/basic_node_test.hpp"
#include <mutex>
#include <chrono>
#include <thread>

namespace cmr_tests_utils {

template<class MessageT>
class BasicPublisherNodeTest: public BasicNodeTest {

  public:
  
  BasicPublisherNodeTest(std::string node_name, std::string topic_name, bool use_timer,
                          unsigned int publish_period_ms = 100,
                          rclcpp::QoS qos = rclcpp::SystemDefaultsQoS())
    : BasicNodeTest(node_name)
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
    published_msg_ = msg;
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
    topic_pub_->publish(published_msg_);
  }

  typename rclcpp::Publisher<MessageT>::SharedPtr topic_pub_;
  MessageT published_msg_;
  rclcpp::TimerBase::SharedPtr timer_;
  mutable std::mutex msg_mutex_;
};

}

#endif
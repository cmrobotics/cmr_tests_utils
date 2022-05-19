#ifndef BASIC_SUBSCRIBER_NODE_TEST_HPP
#define BASIC_SUBSCRIBER_NODE_TEST_HPP

#include <mutex>

#include <rclcpp/rclcpp.hpp>

namespace cmr_tests_utils
{

template<class MessageT>
class BasicSubscriberNodeTest: public rclcpp::Node {

  public:

  BasicSubscriberNodeTest(std::string node_name, std::string topic_name, 
                          rclcpp::QoS qos = rclcpp::SystemDefaultsQoS()): rclcpp::Node(node_name)
  {
    topic_sub_ = this->create_subscription<MessageT> (
                topic_name, qos,
                std::bind (&BasicSubscriberNodeTest::topic_callback, this, std::placeholders::_1));
  }

  bool has_data_been_received() const 
  {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    return (received_msg_ != nullptr);
  }

  MessageT get_received_msg() const
  {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    MessageT msg;
    if (!received_msg_) 
    {
      RCLCPP_WARN(get_logger(), "Tried to get received message from subscription but nothing was published yet.");
      return msg;
    }
    return *received_msg_;
  }

  private:

  void topic_callback(const std::shared_ptr<MessageT> msg) 
  {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    received_msg_ = msg;
  }

  typename rclcpp::Subscription<MessageT>::SharedPtr topic_sub_;
  std::shared_ptr<MessageT> received_msg_;
  mutable std::mutex msg_mutex_;
};

}

#endif
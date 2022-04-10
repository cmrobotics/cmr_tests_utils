#ifndef BASIC_SUBSCRIBER_NODE_TEST_HPP
#define BASIC_SUBSCRIBER_NODE_TEST_HPP

#include "rclcpp/rclcpp.hpp"
#include "basic_node_test.hpp" 
#include <mutex>

namespace cmr_tests_utils
{

template<class MessageT>
class BasicSubscriberNodeTest: public BasicNodeTest {

  public:

  BasicSubscriberNodeTest(std::string node_name, std::string topic_name, 
                          rclcpp::QoS qos = rclcpp::SystemDefaultsQoS());
  const std::shared_ptr<MessageT> get_received_msg() const;

  private:

  void topic_callback(const std::shared_ptr<MessageT> msg);

  typename rclcpp::Subscription<MessageT>::SharedPtr topic_sub_;
  std::shared_ptr<MessageT> received_msg_;
  mutable std::mutex msg_mutex_;
};

}

#endif
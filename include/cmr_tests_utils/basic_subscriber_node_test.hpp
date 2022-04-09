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
                          rclcpp::QoS qos = rclcpp::SystemDefaultsQoS()): BasicNodeTest(node_name)
  {
    topic_sub_ = this->create_subscription<MessageT> (
                 footprint_topic, qos,
                 std::bind (&BasicSubscriberNodeTest::topic_callback, this, std::placeholders::_1));
  }

  bool has_data_been_received() const 
  {
    return data_has_been_received_;
  }

  MessageT get_received_msg() const 
  {
    return received_msg_;
  }

  private:

  void topic_callback(const MessageT::SharedPtr msg) 
  {
    std::call_once(init_flag_, [=] {
      data_has_been_received_ = true;
    });
    received_msg_ = *msg;
  }

  std::shared_ptr<rclcpp::Subscription<MessageT>> topic_sub_;
  MessageT received_msg_;
  std::once_flag init_flag_;
  bool data_has_been_received_;
}

}

#endif
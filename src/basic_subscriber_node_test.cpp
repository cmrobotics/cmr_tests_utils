#include "cmr_tests_utils/basic_subscriber_node_test.hpp"

namespace cmr_tests_utils {

template<class MessageT>
BasicSubscriberNodeTest<MessageT>::BasicSubscriberNodeTest(std::string node_name, std::string topic_name, rclcpp::QoS qos)
  : BasicNodeTest(node_name)
{
  topic_sub_ = this->create_subscription<MessageT> (
               topic_name, qos,
               std::bind (&BasicSubscriberNodeTest::topic_callback, this, std::placeholders::_1));
}

template<class MessageT>
const std::shared_ptr<MessageT> BasicSubscriberNodeTest<MessageT>::get_received_msg() const 
{
  std::lock_guard<std::mutex> lock(msg_mutex_);
  auto msg = received_msg_;
  if (!msg) RCLCPP_WARN(get_logger(), "Tried to get received message from subscription but nothing was published yet.");
  return msg;
}

template<class MessageT>
void BasicSubscriberNodeTest<MessageT>::topic_callback(const std::shared_ptr<MessageT> msg) 
{
  std::lock_guard<std::mutex> lock(msg_mutex_);
  received_msg_ = msg;
}

}

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
const bool& BasicSubscriberNodeTest<MessageT>::has_data_been_received() const 
{
  return data_has_been_received_;
}

template<class MessageT>
const MessageT& BasicSubscriberNodeTest<MessageT>::get_received_msg() const 
{
  return received_msg_;
}

template<class MessageT>
void BasicSubscriberNodeTest<MessageT>::topic_callback(const std::shared_ptr<MessageT> msg) 
{
  std::call_once(init_flag_, [=] {
    data_has_been_received_ = true;
  });
  received_msg_ = *msg;
}

}

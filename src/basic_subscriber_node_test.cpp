#include "cmr_tests_utils/basic_subscriber_node_test.hpp"

namespace cmr_tests_utils {

BasicSubscriberNodeTest::BasicSubscriberNodeTest(std::string node_name, std::string topic_name, rclcpp::QoS qos)
  : BasicNodeTest(node_name)
{
  topic_sub_ = this->create_subscription<MessageT> (
               footprint_topic, qos,
               std::bind (&BasicSubscriberNodeTest::topic_callback, this, std::placeholders::_1));
}

const bool& BasicSubscriberNodeTest::has_data_been_received() const 
{
  return data_has_been_received_;
}

const MessageT& BasicSubscriberNodeTest::get_received_msg() const 
{
  return received_msg_;
}

void BasicSubscriberNodeTest::topic_callback(const std::shared_ptr<MessageT> msg) 
{
  std::call_once(init_flag_, [=] {
    data_has_been_received_ = true;
  });
  received_msg_ = *msg;
}

}

#include "cmr_tests_utils/basic_publisher_node_test.hpp"

namespace cmr_tests_utils {

template<class MessageT>
BasicPublisherNodeTest<MessageT>::BasicPublisherNodeTest(std::string node_name, std::string topic_name, bool use_timer,
                                               unsigned int publish_period_ms,
                                               rclcpp::QoS qos)
  : BasicNodeTest(node_name)
{
  topic_pub_ = this->create_publisher<MessageT> (topic_name, qos);
  if (use_timer) timer_ = this->create_wall_timer(
                            std::chrono::milliseconds(publish_period_ms), 
                            std::bind(&BasicPublisherNodeTest::publish, this)
                          );
}

template<class MessageT>
void BasicPublisherNodeTest<MessageT>::set_published_msg(const MessageT& msg)
{
  published_msg_ = msg;
}

template<class MessageT>
void BasicPublisherNodeTest<MessageT>::publish()
{
  topic_pub_->publish(published_msg_);
}

template<class MessageT>
void BasicPublisherNodeTest<MessageT>::publish(const MessageT& msg) 
{
  topic_pub_->publish(msg);
}

}

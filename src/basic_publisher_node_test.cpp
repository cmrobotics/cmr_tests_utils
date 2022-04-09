#include "cmr_tests_utils/basic_publisher_node_test.hpp"

namespace cmr_tests_utils {

BasicPublisherNodeTest::BasicPublisherNodeTest(std::string node_name, std::string topic_name, bool use_timer,
                                               std::chrono::milliseconds publish_period,
                                               rclcpp::QoS qos)
  : BasicNodeTest(node_name)
{
  topic_sub_ = this->create_publisher<MessageT> (footprint_topic, qos);
  if (use_timer) timer_ = node_->create_wall_timer(
                            publish_period, 
                            std::bind(&BasicPublisherNodeTest::publish, this)
                          );
}

void BasicPublisherNodeTest::set_published_msg(const MessageT& msg)
{
  published_msg_ = msg;
}

void BasicPublisherNodeTest::publish()
{
  topic_pub_->publish(published_msg_);
}

void BasicPublisherNodeTest::publish(const MessageT& msg) 
{
  topic_pub_->publish(msg);
}

}

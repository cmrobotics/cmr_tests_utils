#ifndef BASIC_PUBLISHER_NODE_TEST_HPP
#define BASIC_PUBLISHER_NODE_TEST_HPP

#include "rclcpp/rclcpp.hpp"
#include "basic_node_test.hpp"

namespace cmr_tests_utils
{

template<class MessageT>
class BasicPublisherNodeTest: public BasicNodeTest {

  public:
  BasicPublisherNodeTest(std::string node_name, std::string topic_name, bool use_timer,
                          std::chrono::duration publish_period,
                          rclcpp::QoS qos = rclcpp::SystemDefaultsQoS()): BasicNodeTest(node_name)
  {
    topic_sub_ = this->create_publisher<MessageT> (footprint_topic, qos);

    if (use_timer) timer_ = node_->create_wall_timer(
                              publish_period, 
                              std::bind(&BasicPublisherNodeTest::publish, this)
                            );
  }

  void set_published_msg(const MessageT msg)
  {
    published_msg_ = msg;
  }

  void publish()
  {
    topic_pub_->publish(published_msg_);
  }

  void publish(const MessageT msg) 
  {
    topic_pub_->publish(msg);
  }

  private:

  std::shared_ptr<rclcpp::Publisher<MessageT>> topic_pub_;
  MessageT published_msg_;
  rclcpp::TimerBase::SharedPtr timer_;
}

}

#endif
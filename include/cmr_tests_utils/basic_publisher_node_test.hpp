#ifndef BASIC_PUBLISHER_NODE_TEST_HPP
#define BASIC_PUBLISHER_NODE_TEST_HPP

#include "rclcpp/rclcpp.hpp"
#include "cmr_tests_utils/basic_node_test.hpp"
#include <mutex>

namespace cmr_tests_utils {

template<class MessageT>
class BasicPublisherNodeTest: public BasicNodeTest {

  public:
  
  BasicPublisherNodeTest(std::string node_name, std::string topic_name, bool use_timer,
                          unsigned int publish_period = 100,
                          rclcpp::QoS qos = rclcpp::SystemDefaultsQoS());
  void set_published_msg(const MessageT& msg);
  void publish();
  void publish(const MessageT& msg);

  private:

  typename rclcpp::Publisher<MessageT> topic_pub_;
  MessageT published_msg_;
  rclcpp::TimerBase::SharedPtr timer_;
  mutable std::mutex msg_mutex_;
};

}

#endif
#ifndef BASIC_NODE_TEST_HPP
#define BASIC_NODE_TEST_HPP

#include "rclcpp/rclcpp.hpp"

namespace cmr_tests_utils {

class BasicNodeTest: public rclcpp::Node
{
  public:

  BasicNodeTest(std::string node_name): Node(node_name) {}
  bool is_spinning = false;

  void spin_some()
  {
    spin_thread_ = std::make_shared<std::thread>(&BasicNodeTest::spin_, this);
    is_spinning = true;
  }

  void cancel_spin()
  {
    spin_thread_->join();
    spin_thread_.reset();
    is_spinning = false;
  }

  private:

  std::shared_ptr<std::thread> spin_thread_;

  void spin_()
  {
    rclcpp::spin(node_);
  }
};

}

#endif
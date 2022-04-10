#ifndef BASIC_NODE_TEST_HPP
#define BASIC_NODE_TEST_HPP

#include <atomic>
#include "rclcpp/rclcpp.hpp"

namespace cmr_tests_utils {

class BasicNodeTest: public rclcpp::Node
{
  public:

  BasicNodeTest(std::string node_name): Node(node_name) {}

  void spin_in_new_thread()
  {
    if (spin_thread_)
    {
      RCLCPP_WARN(get_logger(), "Tried to spin in new thread but node is already spinning, ignoring...");
      return;
    }
    spin_thread_ = std::make_shared<std::thread>(&BasicNodeTest::spin_, this);
    is_spinning_.store(true);
  }

  void cancel_spin()
  {
    spin_thread_->join();
    spin_thread_.reset();
    is_spinning_.store(false);
  }

  bool get_is_spinning() const
  {
    return is_spinning_.load();
  }

  private:

  void spin_()
  {
    rclcpp::spin(get_node_base_interface());
  }

  std::shared_ptr<std::thread> spin_thread_;
  std::atomic<bool> is_spinning_;
  
};

}

#endif
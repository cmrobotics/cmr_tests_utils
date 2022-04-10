#ifndef BASIC_NODE_TEST_HPP
#define BASIC_NODE_TEST_HPP

#include <atomic>
#include "rclcpp/rclcpp.hpp"

namespace cmr_tests_utils {

class BasicNodeTest: public rclcpp::Node
{
  public:

  BasicNodeTest(std::string node_name): Node(node_name) {}
  ~BasicNodeTest()
  {
    if (spinner_) cancel_spin();
  }

  void spin_in_new_thread()
  {
    if (spinner_)
    {
      RCLCPP_WARN(get_logger(), "Tried to spin in new thread but node is already spinning, ignoring...");
      return;
    }
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "Failed to spin node, rclcpp is not healthy.");
      return;
    }
    spinner_ = std::make_shared<std::thread>(&BasicNodeTest::spin_, this);
    is_spinning_.store(true);
  }

  void cancel_spin()
  {
    spinner_->join();
    spinner_.reset();
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

  std::shared_ptr<std::thread> spinner_;
  std::atomic<bool> is_spinning_ = false;
  
};

}

#endif
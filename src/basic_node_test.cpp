#include "cmr_tests_utils/basic_node_test.hpp"

namespace cmr_tests_utils {

BasicNodeTest::BasicNodeTest(std::string node_name): Node(node_name) {}

void BasicNodeTest::spin_in_new_thread()
{
  if (spin_thread_)
  {
    RCLCPP_WARN(get_logger(), "Tried to spin in new thread but node is already spinning, ignoring...");
    return;
  }
  spin_thread_ = std::make_shared<std::thread>(&BasicNodeTest::spin_, this);
  is_spinning_.store(true);
}

void BasicNodeTest::cancel_spin()
{
  spin_thread_->join();
  spin_thread_.reset();
  is_spinning_.store(false);
}

void BasicNodeTest::spin_()
{
  rclcpp::spin(get_node_base_interface());
}

const bool& BasicNodeTest::get_is_spinning() const
{
  return is_spinning_.load();
}

}

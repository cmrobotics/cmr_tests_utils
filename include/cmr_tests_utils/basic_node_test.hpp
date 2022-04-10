#ifndef BASIC_NODE_TEST_HPP
#define BASIC_NODE_TEST_HPP

#include <atomic>
#include "rclcpp/rclcpp.hpp"

namespace cmr_tests_utils {

class BasicNodeTest: public rclcpp::Node
{
  public:

  BasicNodeTest(std::string node_name);
  void spin_in_new_thread();
  void cancel_spin();
  const bool& get_is_spinning() const;

  private:

  std::shared_ptr<std::thread> spin_thread_;
  std::atomic<bool> is_spinning_;
  void spin_();
};

}

#endif
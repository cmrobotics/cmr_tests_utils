#ifndef BASIC_NODE_TEST_HPP
#define BASIC_NODE_TEST_HPP

#include "rclcpp/rclcpp.hpp"

namespace cmr_tests_utils {

class BasicNodeTest: public rclcpp::Node
{
  public:

  BasicNodeTest(std::string node_name);
  void spin_some();
  void cancel_spin();
  const bool& get_is_spinning() const;

  private:

  std::shared_ptr<std::thread> spin_thread_;
  bool is_spinning_ = false;
  void spin_();
};

}

#endif
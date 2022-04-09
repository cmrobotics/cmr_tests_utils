#ifndef BASIC_ACTION_CLIENT_TEST_HPP
#define BASIC_ACTION_CLIENT_TEST_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <chrono>

namespace cmr_tests_utils {

template<class ActionT>
class BasicActionClientTest {
  
  public:

  BasicActionClientTest(std::string client_node_name, std::string action_name, 
                        std::chrono::milliseconds action_timeout);
  bool is_action_ready();
  int8_t send_goal(ActionT::Goal goal);

  private:

  typename rclcpp_action::Client<ActionT>::SharedPtr action_client_;
  rclcpp::Node::SharedPtr client_node_;
  rclcpp_action::ClientGoalHandle<ActionT> action_goal_handle_;
  std::string action_name_;
  std::chrono::milliseconds action_timeout_;
  std::string client_node_name_;
};

}

#endif
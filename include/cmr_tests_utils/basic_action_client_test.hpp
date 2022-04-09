#ifndef BASIC_ACTION_CLIENT_TEST_HPP
#define BASIC_ACTION_CLIENT_TEST_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/rclcpp_action.hpp"
#include "basic_node_test.hpp"
#include <chrono>

namespace cmr_tests_utils {

template<class ActionT>
class BasicActionClientTest {
  
  public:
  BasicActionClientTest(std::string client_node_name, std::string action_name, 
                        std::chrono::duration action_timeout)
  {
    client_node_ = rclcpp::Node::make_shared(client_node_name);
    action_client_ = rclcpp_action::create_client<ActionT>(shared_from_this(), action_name);
    
    action_timeout_ = action_timeout;
    action_name_ = action_name;
    client_node_name_ = client_node_name;
  }

  bool is_action_ready() 
  {
    if !(this->is_spinning)
    {
      RCLCPP_ERROR(rclcpp::get_logger(client_node_name_), "Failed to check if action server is responding: node is not spinning.");
      return false;
    }
    return action_client_->wait_for_action_server(action_timeout_);
  }

  int8_t send_goal(ActionT::Goal goal) 
  {
    if (!is_action_ready())
    {
      RCLCPP_ERROR(rclcpp::get_logger(client_node_name_), "Action server did not respond in time: failed to send goal");
      return false;
    }

    auto send_goal_options = rclcpp_action::Client<ActionT>::SendGoalOptions();
    auto result_future = this->action_client_->async_send_goal(goal, send_goal_options);

    if (rclcpp::spin_until_future_complete(this->client_node_, result_future) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(rclcpp::get_logger(client_node_name_), "Successfully called action!");
    } else {
      RCLCPP_ERROR(rclcpp::get_logger(client_node_name_), "Failed to call action.");
      return false;
    }
    action_goal_handle_ = result_future.get();
    if (!action_goal_handle_) {
      RCLCPP_ERROR(rclcpp::get_logger(client_node_name_), "Goal was rejected by server");
      return false;
    }

    return action_goal_handle_.get_status();
  }

  private:

  rclcpp::Client<ActionT>::SharedPtr action_client_;
  rclcpp::Node::SharedPtr client_node_;
  rclcpp_action::ClientGoalHandle<ActionT> action_goal_handle_;
  std::string action_name_;
  std::chrono::duration action_timeout_;
  std::string client_node_name_;
}

}

#endif
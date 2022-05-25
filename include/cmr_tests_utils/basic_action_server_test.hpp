#ifndef BASIC_ACTION_SERVER_TEST_HPP
#define BASIC_ACTION_SERVER_TEST_HPP
#pragma once

#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace cmr_tests_utils {

template<class ActionT>
class BasicActionServerTest : public rclcpp::Node
{
  public:

  BasicActionServerTest(std::string node_name, std::string action_name)
    : Node(node_name)
  {
    using namespace std::placeholders;  // NOLINT

    this->action_server_ = rclcpp_action::create_server<ActionT>(
      this->get_node_base_interface(),
      this->get_node_clock_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      action_name,
      std::bind(&BasicActionServerTest::handle_goal, this, _1, _2),
      std::bind(&BasicActionServerTest::handle_cancel, this, _1),
      std::bind(&BasicActionServerTest::handle_accepted, this, _1));
  }

  std::shared_ptr<const typename ActionT::Goal> getLastGoal() const
  {
    return last_goal_;
  }

  protected:

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const typename ActionT::Goal> goal)
  {
    last_goal_ = goal;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const typename std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  virtual void execute(
    typename std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle)
  {
    auto result = std::make_shared<typename ActionT::Result>();
    if (goal_handle->is_canceling()) 
    {
      RCLCPP_INFO(get_logger(), "Cancelling the action goal.");
      goal_handle->canceled(result);
    }
    RCLCPP_INFO(get_logger(), "Successfully reached the goal");
    goal_handle->succeed(result);
  }

  void handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle)
  {
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&BasicActionServerTest::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  private:

  typename rclcpp_action::Server<ActionT>::SharedPtr action_server_;
  std::shared_ptr<const typename ActionT::Goal> last_goal_;
};

}

#endif  // TEST_ACTION_SERVER_HPP_

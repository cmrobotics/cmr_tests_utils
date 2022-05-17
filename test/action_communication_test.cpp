#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "cmr_tests_utils/synchronous_nodes/basic_action_server_test.hpp"
#include "cmr_tests_utils/synchronous_nodes/basic_action_client_test.hpp"
#include "cmr_tests_utils/spinners/single_thread_spinner.hpp"
#include "example_interfaces/action/fibonacci.hpp"
#include <chrono>
#include <thread>

TEST(ServiceCommunication, action_communication)
{
  rclcpp::init(0, nullptr);

  auto spinner = cmr_tests_utils::SingleThreadSpinner();
  auto action_client = std::make_shared<cmr_tests_utils::BasicActionClientTest<example_interfaces::action::Fibonacci>>(
                          "action_client_test", "action"
                        );
  EXPECT_FALSE(action_client->is_server_ready());

  auto action_server = std::make_shared<cmr_tests_utils::BasicActionServerTest<example_interfaces::action::Fibonacci>>(
                          "action_server_test", "action"
                        );
  EXPECT_TRUE(action_client->is_server_ready());                        
                        
  EXPECT_EQ(action_server->getLastGoal(), nullptr);

  spinner.add_node(action_server->get_node_base_interface());

  // Instantiate service request without spinning
  auto goal_msg = example_interfaces::action::Fibonacci::Goal();
  goal_msg.order = 3;

  spinner.spin_some_all_nodes();

  // Overhead for queued data population
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  
  EXPECT_TRUE(action_client->is_server_ready());
  
  // Send request
  auto status = action_client->send_goal(goal_msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  EXPECT_EQ(status, rclcpp_action::ResultCode::SUCCEEDED);

  auto last_goal = action_server->getLastGoal();
  EXPECT_EQ(last_goal->order, 3);
  
  rclcpp::shutdown();
}

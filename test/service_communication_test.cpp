#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "cmr_tests_utils/synchronous_nodes/basic_service_server_test.hpp"
#include "cmr_tests_utils/synchronous_nodes/basic_service_client_test.hpp"
#include "cmr_tests_utils/spinners/single_thread_spinner.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include <chrono>
#include <thread>

TEST(ServiceCommunication, service_communication)
{
  rclcpp::init(0, nullptr);

  auto spinner = cmr_tests_utils::SingleThreadSpinner();
  auto service_client = std::make_shared<cmr_tests_utils::BasicServiceClientTest<example_interfaces::srv::AddTwoInts>>(
                          "service_client_test", "service"
                        );
  EXPECT_FALSE(service_client->is_server_ready());

  auto service_server = std::make_shared<cmr_tests_utils::BasicServiceServerTest<example_interfaces::srv::AddTwoInts>>(
                          "service_server_test", "service"
                        );
  EXPECT_TRUE(service_client->is_server_ready());

  EXPECT_EQ(service_server->getLastRequest(), nullptr);

  spinner.add_node(service_server->get_node_base_interface());

  // Instantiate service request without spinning
  auto sum_req = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
  sum_req->a = 2;
  sum_req->b = 3;

  // Spin both nodes
  spinner.spin_some_all_nodes();

  // Overhead for queued data population
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  
  EXPECT_TRUE(service_client->is_server_ready());
  
  auto response = service_client->send_request(sum_req);
  EXPECT_TRUE(response);

  auto last_request = service_server->getLastRequest();
  EXPECT_TRUE(last_request);
  EXPECT_EQ(last_request->a, 2);
  EXPECT_EQ(last_request->b, 3);
  
  rclcpp::shutdown();
}

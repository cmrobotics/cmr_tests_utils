#include "cmr_tests_utils/basic_service_client_test.hpp"

namespace cmr_tests_utils {

BasicServiceClientTest::BasicServiceClientTest(std::string client_node_name, std::string service_name, 
                                               std::chrono::milliseconds service_timeout)
{
  client_node_ = rclcpp::Node::make_shared(client_node_name);
  service_client_ = client_node_->create_client<ServiceT>(service_name);
  
  service_timeout_ = service_timeout;
  service_name_ = service_name;
  client_node_name_ = client_node_name;
}

bool BasicServiceClientTest::is_service_ready() 
{
  if (!rclcpp::ok())
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to check if service server is responding: ROS isn't running");
    return false;
  }
  return service_client_->wait_for_service(service_timeout_);
}

bool BasicServiceClientTest::send_request(std::shared_ptr<ServiceT::Request> request, std::shared_ptr<ServiceT::Result> result) 
{
  auto res = service_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(client_node_, res) ==
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger(client_node_name_), "Successfully called service!");
    result = res.get();
    return true;
  }
  
  RCLCPP_ERROR(rclcpp::get_logger(client_node_name_), "Failed to call service.");
  return false;
}

}

#include "cmr_tests_utils/basic_service_client_test.hpp"

namespace cmr_tests_utils {

template<class ServiceT>
BasicServiceClientTest<ServiceT>::BasicServiceClientTest(std::string client_node_name, std::string service_name, 
                                               unsigned int service_timeout_ms)
{
  client_node_ = rclcpp::Node::make_shared(client_node_name);
  service_client_ = client_node_->create_client<ServiceT>(service_name);
  
  service_timeout_ = std::chrono::milliseconds(service_timeout_ms);
  service_name_ = service_name;
  client_node_name_ = client_node_name;
}

template<class ServiceT>
bool BasicServiceClientTest<ServiceT>::is_service_ready() 
{
  if (!rclcpp::ok())
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to check if service server is responding: ROS isn't running");
    return false;
  }
  return service_client_->wait_for_service(service_timeout_);
}

template<class ServiceT>
bool BasicServiceClientTest<ServiceT>::send_request(std::shared_ptr<typename ServiceT::Request> request, std::shared_ptr<typename ServiceT::Result> result) 
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

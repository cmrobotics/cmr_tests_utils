#ifndef BASIC_SERVICE_CLIENT_TEST_HPP
#define BASIC_SERVICE_CLIENT_TEST_HPP

#include "rclcpp/rclcpp.hpp"
#include <chrono>

namespace cmr_tests_utils {

template<class ServiceT>
class BasicServiceClientTest {
  
  public:

  BasicServiceClientTest(std::string client_node_name, std::string service_name, unsigned int service_timeout_ms)
  {
    client_node_ = rclcpp::Node::make_shared(client_node_name);
    service_client_ = client_node_->create_client<ServiceT>(service_name);
    
    service_timeout_ = std::chrono::milliseconds(service_timeout_ms);
    service_name_ = service_name;
    client_node_name_ = client_node_name;
  }

  bool is_service_ready()
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to check if service server is responding: ROS isn't running");
      return false;
    }
    return service_client_->wait_for_service(service_timeout_);
  }

  bool send_request(std::shared_ptr<typename ServiceT::Request> request, 
                    std::shared_ptr<typename ServiceT::Result> result)
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

  private:

  typename rclcpp::Client<ServiceT>::SharedPtr service_client_;
  rclcpp::Node::SharedPtr client_node_;
  std::string service_name_;
  std::chrono::milliseconds service_timeout_;
  std::string client_node_name_;
};

}

#endif
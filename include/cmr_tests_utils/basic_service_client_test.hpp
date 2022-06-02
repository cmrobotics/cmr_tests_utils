#pragma once

#include <chrono>

#include <rclcpp/rclcpp.hpp>

namespace cmr_tests_utils {

template<class ServiceT>
class BasicServiceClientTest {
  
  public:

  BasicServiceClientTest(std::string client_node_name, std::string service_name, unsigned int service_timeout_ms = 1000)
  {
    client_node_ = rclcpp::Node::make_shared(client_node_name);
    service_client_ = client_node_->create_client<ServiceT>(service_name);
    
    service_timeout_ = std::chrono::milliseconds(service_timeout_ms);
    service_name_ = service_name;
    client_node_name_ = client_node_name;
  }

  bool is_server_ready()
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to check if service server is responding: ROS isn't running");
      return false;
    }
    return service_client_->wait_for_service(service_timeout_);
  }

  std::shared_ptr<typename ServiceT::Response> send_request(std::shared_ptr<typename ServiceT::Request> request)
  {
    if (!is_server_ready()) return nullptr;

    auto res = service_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(client_node_, res) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(rclcpp::get_logger(client_node_name_), "Successfully called service!");
      return res.get();
    }
    
    RCLCPP_ERROR(rclcpp::get_logger(client_node_name_), "Failed to call service.");
    return nullptr;
  }

  private:

  typename rclcpp::Client<ServiceT>::SharedPtr service_client_;
  rclcpp::Node::SharedPtr client_node_;
  std::string service_name_;
  std::chrono::milliseconds service_timeout_;
  std::string client_node_name_;
};

}

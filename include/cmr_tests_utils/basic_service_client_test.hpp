#ifndef BASIC_SERVICE_CLIENT_TEST_HPP
#define BASIC_SERVICE_CLIENT_TEST_HPP

#include "rclcpp/rclcpp.hpp"
#include <chrono>

namespace cmr_tests_utils {

template<class ServiceT>
class BasicServiceClientTest {
  
  public:

  BasicServiceClientTest(std::string client_node_name, std::string service_name, std::chrono::milliseconds service_timeout);
  bool is_service_ready();
  bool send_request(std::shared_ptr<typename ServiceT::Request> request, 
                    std::shared_ptr<typename ServiceT::Result> result);

  private:

  typename rclcpp::Client<ServiceT>::SharedPtr service_client_;
  rclcpp::Node::SharedPtr client_node_;
  std::string service_name_;
  std::chrono::milliseconds service_timeout_;
  std::string client_node_name_;
};

}

#endif
#pragma once

#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>

namespace cmr_tests_utils {

template<class ServiceT>
class BasicServiceServerTest: public rclcpp::Node
{
  public:

  BasicServiceServerTest(std::string node_name, std::string service_name)
     : rclcpp::Node(node_name)
  {
    using namespace std::placeholders;  // NOLINT

    server_ = create_service<ServiceT>(
      service_name,
      std::bind(&BasicServiceServerTest::request_callback, this, _1, _2, _3));
  }

  std::shared_ptr<typename ServiceT::Request> getLastRequest() const
  {
    return last_request_;
  }

  protected:

  virtual void request_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<typename ServiceT::Request> request,
    std::shared_ptr<typename ServiceT::Response> response)
  {
    (void)request_header;
    (void)response;
    last_request_ = request;
  }

  std::shared_ptr<typename ServiceT::Request> last_request_;

  private:
  
  typename rclcpp::Service<ServiceT>::SharedPtr server_;
};

}


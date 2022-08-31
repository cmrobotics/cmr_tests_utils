#pragma once

#include <map>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "cmr_tests_utils/single_thread_spinner.hpp"

namespace cmr_tests_utils {

class MultiThreadSpinner
{
  public:

  MultiThreadSpinner() {}

  ~MultiThreadSpinner() 
  {
    for (auto spinner : spinners_)
    {
      spinner.second.reset();
    }
    this->spinners_.clear();
  }

  bool add_node(rclcpp_lifecycle::LifecycleNode::SharedPtr lc_node)
  {
    return add_node(lc_node->get_node_base_interface());
  }

  bool add_node(rclcpp::Node::SharedPtr node)
  {
    return add_node(node->get_node_base_interface()); 
  }

  bool remove_node(rclcpp_lifecycle::LifecycleNode::SharedPtr lc_node)
  {
    return remove_node(lc_node->get_node_base_interface());
  }

  bool remove_node(rclcpp::Node::SharedPtr node)
  {
    return remove_node(node->get_node_base_interface());
  }

  bool add_node(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node, std::shared_ptr<SingleThreadSpinner> spinner = std::make_shared<SingleThreadSpinner>())
  {
    if (this->spinners_.find(std::string(node->get_name())) != spinners_.end())
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Node is already registered in a spinner");
      return false;
    }

    bool res = spinner->add_node(node);
    
    if (!res) return false;
    this->nodes_.insert(std::pair<std::string, rclcpp::node_interfaces::NodeBaseInterface::SharedPtr>(std::string(node->get_name()), node));
    this->spinners_.insert(std::pair<std::string, std::shared_ptr<SingleThreadSpinner>>(std::string(node->get_name()), spinner)); 

    return true;
  }

  bool remove_node(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node)
  {
    if (this->spinners_.find(std::string(node->get_name())) == spinners_.end())
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Node was never registered in a spinner");
      return false;
    }

    auto spinner = this->spinners_.at(std::string(node->get_name()));

    bool res = spinner->remove_node(node);
    
    if (!res) return false;

    this->nodes_.erase(std::string(node->get_name()));
    this->spinners_.erase(std::string(node->get_name()));

    return true;
  }

  bool spin_some_node(rclcpp::Node::SharedPtr node)
  {
    return spin_some_node(node->get_node_base_interface()); 
  }

  bool spin_some_node(rclcpp_lifecycle::LifecycleNode::SharedPtr lc_node)
  {
    return spin_some_node(lc_node->get_node_base_interface()); 
  }

  bool spin_some_node(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node)
  {
    if (this->spinners_.find(std::string(node->get_name())) == spinners_.end())
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Node was never registered in a spinner, can't spin it");
      return false;
    }

    auto spinner = this->spinners_.at(std::string(node->get_name()));
    return spinner->spin_some_all_nodes();
  }

  bool cancel_spin_node(rclcpp::Node::SharedPtr node)
  {
    return cancel_spin_node(node->get_node_base_interface()); 
  }

  bool cancel_spin_node(rclcpp_lifecycle::LifecycleNode::SharedPtr lc_node)
  {
    return cancel_spin_node(lc_node->get_node_base_interface()); 
  }

  bool cancel_spin_node(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node)
  {
    if (this->spinners_.find(std::string(node->get_name())) == spinners_.end())
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Node was never registered in a spinner, can't spin it");
      return false;
    }

    auto spinner = this->spinners_.at(std::string(node->get_name()));
    return spinner->cancel_all_spin();
  }

  bool is_node_spinning(rclcpp::Node::SharedPtr node)
  {
    return is_node_spinning(node->get_node_base_interface()); 
  }

  bool is_node_spinning(rclcpp_lifecycle::LifecycleNode::SharedPtr lc_node)
  {
    return is_node_spinning(lc_node->get_node_base_interface()); 
  }

  bool is_node_spinning(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node)
  {
    if (this->spinners_.find(std::string(node->get_name())) == spinners_.end())
    {
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Node was never registered in a spinner!");
      return false;
    }

    auto spinner = this->spinners_.at(std::string(node->get_name()));
    return spinner->are_all_nodes_spinning();
  }

  bool cancel_all_spins()
  {
    bool res = true;
    for (auto spin : this->spinners_)
    {
      if (!spin.second->cancel_all_spin()) res = false;
    }
    return res;
  }

  bool spin_all_nodes()
  {
    bool res = true;
    for (auto spin : this->spinners_)
    {
      if (!spin.second->spin_some_all_nodes()) res = false;
    }
    return res;
  }

  bool are_all_nodes_spinning()
  {
    bool res = true;
    for (auto spin : this->spinners_)
    {
      if (!is_node_spinning(nodes_.at(spin.first))) res = false;
    }
    return res;
  }

  private:

  std::map<std::string, rclcpp::node_interfaces::NodeBaseInterface::SharedPtr> nodes_;
  std::map<std::string, std::shared_ptr<SingleThreadSpinner>> spinners_;
};

}

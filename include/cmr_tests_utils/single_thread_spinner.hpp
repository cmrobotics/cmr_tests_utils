#ifndef SINGLE_THREAD_SPINNER_HPP
#define SINGLE_THREAD_SPINNER_HPP
#pragma once

#include <thread>
#include <map>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace cmr_tests_utils {

class SingleThreadSpinner: private rclcpp::executors::SingleThreadedExecutor
{
  public:

  SingleThreadSpinner() {}

  ~SingleThreadSpinner()
  {
    if (spinner_thread_)
    {
      if (spinner_thread_->joinable()) spinner_thread_->join();
    }
    spinner_thread_.reset();
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

  bool add_node(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    try {
      // This block ensures no external executor has registered the node.
      rclcpp::executors::SingleThreadedExecutor::add_node(node);
    } catch (std::runtime_error & ex) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "%s", ex.what());
      return false;
    }

    return true;
  }

  bool remove_node(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    try {
      rclcpp::executors::SingleThreadedExecutor::remove_node(node);
    } catch (std::runtime_error & ex) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "%s", ex.what());
      return false;
    }

    return true;
  }

  bool cancel_all_spin()
  {
    if (!are_all_nodes_spinning_.load())
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Spinner is not spinning");
      return false;
    }

    cancel_spin_called_.store(true);  
    while(are_all_nodes_spinning_.load());
    std::lock_guard<std::mutex> lock(mutex_);
    if (spinner_thread_) 
    {
      if (spinner_thread_->joinable()) spinner_thread_->join();
      spinner_thread_.reset();
    }
    return true;
  }

  bool are_all_nodes_spinning() const
  {
    return are_all_nodes_spinning_.load();
  }

  bool spin_some_all_nodes()
  {
    if (are_all_nodes_spinning_.load())
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "All registered nodes are already spinning");
      return false;
    }
    
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (spinner_thread_)
      {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "All registered nodes are already spinning");
        return false;
      }

      spinner_thread_ = std::make_shared<std::thread>(&SingleThreadSpinner::spin_all_nodes_, this);
    }
    are_all_nodes_spinning_.store(true);
    return true;
  }

  private:

  void spin_all_nodes_()
  {
    try {
      while (rclcpp::ok()) 
      {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          rclcpp::executors::SingleThreadedExecutor::spin_some();
        }
        if (cancel_spin_called_.load()) break;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Single Thread Spinner was cancelled.");
    } catch (rclcpp::exceptions::RCLError & ex) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to spin nodes: %s", ex.what());
    }
    cancel_spin_called_.store(false);
    are_all_nodes_spinning_.store(false);
  }

  std::mutex mutex_;
  std::shared_ptr<std::thread> spinner_thread_;
  std::atomic<bool> are_all_nodes_spinning_ = false;
  std::atomic<bool> cancel_spin_called_ = false;
};

}

#endif
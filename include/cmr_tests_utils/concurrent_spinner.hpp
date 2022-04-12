#ifndef CONCURRENT_SPINNER_HPP
#define CONCURRENT_SPINNER_HPP

#include "rclcpp/rclcpp.hpp"
#include "cmr_tests_utils/basic_publisher_node_test.hpp"
#include "cmr_tests_utils/basic_subscriber_node_test.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <thread>
#include <unordered_map>

namespace cmr_tests_utils {

class ConcurrentSpinner
{
  public:

  explicit ConcurrentSpinner() {}

  ~ConcurrentSpinner()
  {
    for (auto thread : spinner_threads_)
    {
      if (thread) {
        if (thread->joinable()) thread->join();
        thread.reset();
      }
    }
    are_all_nodes_spinning_.store(false);
  }

  bool add_node(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return add_node_(node);
  }

  bool add_node(rclcpp_lifecycle::LifecycleNode::SharedPtr lc_node)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return add_node_(lc_node->get_node_base_interface());
  }

  bool add_node(rclcpp::Node::SharedPtr node)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return add_node_(node->get_node_base_interface());
  }

  bool add_node(std::shared_ptr<cmr_tests_utils::BasicSubscriberNodeTest> node)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return add_node_(node->get_node_base_interface());
  }

  bool add_node(std::shared_ptr<cmr_tests_utils::BasicPublisherNodeTest> node)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return add_node_(node->get_node_base_interface());
  }

  bool remove_node(rclcpp_lifecycle::LifecycleNode::SharedPtr lc_node)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return remove_node_(lc_node->get_node_base_interface());
  }

  bool remove_node(rclcpp::Node::SharedPtr node)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return remove_node_(node->get_node_base_interface());
  }

  bool remove_node(std::shared_ptr<cmr_tests_utils::BasicPublisherNodeTest> node)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return remove_node_(node->get_node_base_interface());
  }

  bool remove_node(std::shared_ptr<cmr_tests_utils::BasicSubscriberNodeTest> node)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return remove_node_(node->get_node_base_interface());
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
    for (auto t : spinner_threads_)
    {
      if (t) {
        if (t->joinable()) t->join();
        t.reset();
      }
    }
    return true;
  }

  bool are_all_nodes_spinning() const
  {
    return are_all_nodes_spinning_.load();
  }

  protected:

  std::map<std::string, std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface>> nodes_;
  std::mutex mutex_;
  std::vector<std::shared_ptr<std::thread>> spinner_threads_;
  std::atomic<bool> are_all_nodes_spinning_ = false;
  std::atomic<bool> cancel_spin_called_ = false;

  private:

  bool add_node_(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node)
  {
    try {
      // This block ensures no external executor has registered the node.
      rclcpp::executors::SingleThreadedExecutor exe;
      exe.add_node(node);
      exe.remove_node(node);
    } catch (std::runtime_error & ex) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "%s", ex.what());
      return false;
    }

    if (nodes_.contains(node->get_name()))
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to add node to concurrent spinner, the node is already registered in the spinner.");
      return false;
    }
    
    nodes_.insert(std::pair<std::string, rclcpp::node_interfaces::NodeBaseInterface::SharedPtr>(
                  std::string(node->get_name()), node)); 
    return true;
  }

  bool remove_node_(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node)
  {
    if (!nodes_.contains(node->get_name()))
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to remove node to concurrent spinner, the node was never registered in the spinner.");
      return false;
    }
    
    nodes_.erase(std::string(node->get_name()));
    return true;
  }
};

}

#endif
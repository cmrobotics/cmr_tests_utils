#ifndef SINGLE_THREAD_SPINNER_HPP
#define SINGLE_THREAD_SPINNER_HPP

#include "rclcpp/rclcpp.hpp"
#include <thread>
#include <map>
#include "cmr_tests_utils/concurrent_spinner.hpp"

namespace cmr_tests_utils {

class SingleThreadSpinner: public ConcurrentSpinner
{
  public:

  SingleThreadSpinner() {}

  bool spin_some_all_nodes()
  {
    std::lock_guard<std::mutex> lock(ConcurrentSpinner::mutex_);

    if (ConcurrentSpinner::are_all_nodes_spinning_.load())
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "All registered nodes are already spinning");
      return false;
    }

    ConcurrentSpinner::spinner_threads_.push_back(
      std::make_shared<std::thread>(&SingleThreadSpinner::spin_all_nodes_, this)
    );
    ConcurrentSpinner::are_all_nodes_spinning_.store(true);
    return true;
  }

  private:

  void spin_all_nodes_()
  {
    try {
      while (rclcpp::ok()) 
      {
        rclcpp::executors::SingleThreadedExecutor exe;
        for (auto n : ConcurrentSpinner::nodes_)
        {
          exe.add_node(n.second);
        }
        exe.spin_some();
        if (ConcurrentSpinner::cancel_spin_called_.load()) break;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Single Thread Spinner was cancelled.");
    } catch (rclcpp::exceptions::RCLError & ex) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to spin nodes: %s", ex.what());
    }
    ConcurrentSpinner::cancel_spin_called_.store(false);
    ConcurrentSpinner::are_all_nodes_spinning_.store(false);
  }
};

}

#endif
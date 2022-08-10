#pragma once

#include <mutex>

#include <rclcpp/rclcpp.hpp>

namespace cmr_tests_utils
{

typedef struct TopicInfo
{
  std::string topic_name;
  double frequency;
  bool is_alive;
} TopicInfo;

template<class MessageT>
class BasicSubscriberNodeTest: public rclcpp::Node {

  public:

  BasicSubscriberNodeTest(std::string node_name, std::string topic_name, 
                          rclcpp::QoS qos = rclcpp::SystemDefaultsQoS(), 
                          double liveness_timeout_sec = 2.0): rclcpp::Node(node_name)
  {
    topic_sub_ = this->create_subscription<MessageT> (
                topic_name, qos,
                std::bind (&BasicSubscriberNodeTest::topic_callback, this, std::placeholders::_1));
    liveness_timeout_sec_ = liveness_timeout_sec;
    this->topic_name = topic_name;

    topic_info_ = std::make_shared<TopicInfo>();
    topic_info_->topic_name = topic_name;

    timestamp_check_thread_ = std::thread(&BasicSubscriberNodeTest::update_liveness_, this);
  }

  ~BasicSubscriberNodeTest()
  {
    if (timestamp_check_thread_.joinable()) timestamp_check_thread_.join();
  }

  std::shared_ptr<TopicInfo> get_topic_info() const
  {
    std::lock_guard<std::mutex> lock(topic_info_mutex_);
    return topic_info_;
  }

  bool get_liveness() const 
  {
    return is_alive_.load();
  }

  double get_frequency() const
  {
    std::lock_guard<std::mutex> lock(frequency_mutex_);
    return frequency_;
  }

  bool has_data_been_received() const 
  {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    return (received_msg_ != nullptr);
  }

  MessageT get_received_msg() const
  {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    MessageT msg;
    if (!received_msg_) 
    {
      RCLCPP_WARN(get_logger(), "Tried to get received message from subscription but nothing was published yet.");
      return msg;
    }
    return *received_msg_;
  }

  std::string topic_name;

  private:

  void topic_callback(const std::shared_ptr<MessageT> msg) 
  {  
    is_alive_.store(true);

    { 
      std::lock_guard<std::mutex> lock(topic_info_mutex_);
      topic_info_->is_alive = true;
    }

    if (!last_message_timestamp_)
    {
      RCLCPP_INFO (get_logger(), "Discovered [%s] topic...", topic_name.c_str());
      last_message_timestamp_ = std::make_shared<std::chrono::time_point<std::chrono::system_clock>>();
      *last_message_timestamp_ = std::chrono::system_clock::now(); 
    } else {
      auto now = std::chrono::system_clock::now();
      std::chrono::duration<double> elapsed_seconds = now - *last_message_timestamp_;
      double elapsec_seconds_double = elapsed_seconds.count();
      { 
        std::lock_guard<std::mutex> lock(frequency_mutex_);
        frequency_ = 1 / elapsec_seconds_double;
      }
      
      {
        std::lock_guard<std::mutex> lock(topic_info_mutex_);
        topic_info_->frequency = 1 / elapsec_seconds_double;
      }
      *last_message_timestamp_ = now;
    }

    std::lock_guard<std::mutex> lock(msg_mutex_);
    received_msg_ = msg;
  }

  void update_liveness_() 
  {
    std::chrono::duration<double> elapsed_seconds;
    while (true)
    {
      {
        std::lock_guard<std::mutex> lock(timestamp_mutex_);
        if (!last_message_timestamp_) continue;

        auto now = std::chrono::system_clock::now();
        elapsed_seconds = now - *last_message_timestamp_;
      }

      if (elapsed_seconds.count() > liveness_timeout_sec_) 
      {
        is_alive_.store(false);
        {
          std::lock_guard<std::mutex> lock(topic_info_mutex_);
          topic_info_->is_alive = false;
          topic_info_->frequency = 0.0;
        }
      }
      std::this_thread::sleep_for(std::chrono::seconds(2));
    }
  }

  typename rclcpp::Subscription<MessageT>::SharedPtr topic_sub_;

  std::shared_ptr<MessageT> received_msg_;
  std::atomic<bool> is_alive_;
  std::shared_ptr<std::chrono::time_point<std::chrono::system_clock>> last_message_timestamp_;
  double frequency_;
  double liveness_timeout_sec_;
  std::shared_ptr<TopicInfo> topic_info_;

  std::thread timestamp_check_thread_;
  
  mutable std::mutex msg_mutex_;
  mutable std::mutex frequency_mutex_;
  mutable std::mutex topic_info_mutex_;
  mutable std::mutex timestamp_mutex_;
};

}

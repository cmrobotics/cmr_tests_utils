#pragma once

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

    this->create_wall_timer(std::chrono::milliseconds(2000), std::bind(&BasicSubscriberNodeTest::update_liveness_, this));
  }

  std::shared_ptr<TopicInfo> get_topic_info() const
  {
    return topic_info_;
  }

  bool get_liveness() const 
  {
    return is_alive_;
  }

  double get_frequency() const
  {
    return frequency_;
  }

  bool has_data_been_received() const 
  {
    return (received_msg_ != nullptr);
  }

  MessageT get_received_msg() const
  {
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
    is_alive_ = true;
    topic_info_->is_alive = true;
    

    if (!last_message_timestamp_)
    {
      RCLCPP_INFO (get_logger(), "Discovered [%s] topic...", topic_name.c_str());
      last_message_timestamp_ = std::make_shared<std::chrono::time_point<std::chrono::system_clock>>();
      *last_message_timestamp_ = std::chrono::system_clock::now(); 
    } else {
      auto now = std::chrono::system_clock::now();
      std::chrono::duration<double> elapsed_seconds = now - *last_message_timestamp_;
      double elapsec_seconds_double = elapsed_seconds.count();

      frequency_ = 1 / elapsec_seconds_double;
      
      topic_info_->frequency = 1 / elapsec_seconds_double;
      *last_message_timestamp_ = now;
    }

    received_msg_ = msg;
  }

  void update_liveness_() 
  {
    if (!last_message_timestamp_) return;

    auto elapsed_seconds = std::chrono::system_clock::now() - *last_message_timestamp_;
    
    if (elapsed_seconds.count() > liveness_timeout_sec_) 
    {
      is_alive_ = false;
      topic_info_->is_alive = false;
      topic_info_->frequency = 0.0;
    }
  }

  typename rclcpp::Subscription<MessageT>::SharedPtr topic_sub_;

  std::shared_ptr<MessageT> received_msg_;
  bool is_alive_;
  std::shared_ptr<std::chrono::time_point<std::chrono::system_clock>> last_message_timestamp_;
  double frequency_;
  double liveness_timeout_sec_;
  std::shared_ptr<TopicInfo> topic_info_;
};

}

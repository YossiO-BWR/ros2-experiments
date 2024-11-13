//
// Created by yossi on 11/12/24.
//

#include "ComposableSub.h"
std::string formatNumber(int number) {
  char buffer[4];
  snprintf(buffer, sizeof(buffer), "%03d", number);
  return std::string(buffer);
}

void yo_exp2::ComposableSub::declareParameters() {
  declare_parameter<int>("pubs_count", 1);
  declare_parameter<int>("queue_size", 1);
  declare_parameter<bool>("best_effort", false);
}
void yo_exp2::ComposableSub::createSubscribers() {
  int pubs_count = 2;
  get_parameter("pubs_count", pubs_count);
  int queue_size = 1;
  get_parameter("queue_size", queue_size);
  subscribers_.reserve(pubs_count);

  rclcpp::QoS qos(queue_size);
  bool best_effort = false;
  get_parameter<bool>("best_effort", best_effort);
  if (best_effort) {
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  }

  for (int i = 0; i < pubs_count; i++) {
    auto topic_name = "/topic_" + formatNumber(i);
    msgs_count_[i] = 0;
    msgs_size_[i] = 0;
    subscribers_.emplace_back(create_subscription<std_msgs::msg::UInt8MultiArray>(
        topic_name, qos, [this, i](std_msgs::msg::UInt8MultiArray::ConstSharedPtr msg) {
          {
            RCLCPP_DEBUG(get_logger(), "Received a message from %d with size: %lu = %f MB", i, msg->data.size(),
                         1.0 * msg->data.size() / 1024 / 1024);
            msgs_count_[i] += 1;
            msgs_size_[i] += msg->data.size();
          }
        }));
  }
  RCLCPP_INFO(get_logger(), "Created %lu  subscribers", subscribers_.size());
}
void yo_exp2::ComposableSub::createTimer() {
  timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
    for (const auto &[i, val] : msgs_count_) {
      RCLCPP_INFO(get_logger(), "Subscriber %d received %d msgs  with total size: %lu = %f", i, val, msgs_size_[i],
                  1.0 * msgs_size_[i] / 1024 / 1024);
      msgs_count_[i] = 0;
      msgs_size_[i] = 0;
    }
  });
}

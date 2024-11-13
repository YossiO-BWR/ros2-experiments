//
// Created by yossi on 11/12/24.
//

#include "ComposablePub.h"

std::string formatNumber(int number) {
  char buffer[4];
  snprintf(buffer, sizeof(buffer), "%03d", number);
  return std::string(buffer);
}
void yo_exp2::ComposablePub::init() {
  declareParameters();
  createPublishers();
  createTimer();
}
void yo_exp2::ComposablePub::createPublishers() {
  int queue_size = 1;
  get_parameter("queue_size", queue_size);

  rclcpp::QoS qos(queue_size);
  bool best_effort = false;
  get_parameter<bool>("best_effort", best_effort);
  if (best_effort) {
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  }

  int pubs_count = 2;
  get_parameter("pubs_count", pubs_count);
  publishers_.reserve(pubs_count);
  for (int i = 0; i < pubs_count; i++) {
    publishers_.emplace_back(create_publisher<std_msgs::msg::UInt8MultiArray>("/topic_" + formatNumber(i), qos));
  }
  RCLCPP_INFO(get_logger(), "Created %lu  publishers", publishers_.size());
}
void yo_exp2::ComposablePub::declareParameters() {
  declare_parameter<int>("pubs_count", 10);
  declare_parameter<int>("queue_size", 1);
  declare_parameter<double>("hz", 10);
  declare_parameter<int>("data_size", 31457280);
  declare_parameter<bool>("best_effort", false);
}
void yo_exp2::ComposablePub::createTimer() {
  std_msgs::msg::UInt8MultiArray msg;
  {
    size_t data_size = 30;
    get_parameter("data_size", data_size);
    msg.data.resize(data_size);
  }

  double hz = 1;
  get_parameter("hz", hz);
  size_t dt = 1e6 / hz;

  RCLCPP_INFO(get_logger(), "Publishing a message with size: %lu bytes at rate %f hz", msg.data.size(), hz);
  timer_ = create_wall_timer(std::chrono::microseconds(dt), [this, msg]() {
    for (const auto& pub : publishers_) {
      pub->publish(msg);
    }
  });
}
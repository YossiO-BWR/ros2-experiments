//
// Created by yossi on 11/12/24.
//

#ifndef COMPOSABLESUB_H
#define COMPOSABLESUB_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

inline std::string formatNumber(int number);

namespace yo_exp2 {
class ComposableSub final : public rclcpp::Node {
 public:
  ComposableSub() : Node("composable_sub") { init(); }
  explicit ComposableSub(const rclcpp::NodeOptions& options) : Node("composable_sub", options) { init(); }

 private:
  std::vector<rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr> subscribers_;
  std::unordered_map<int, int> msgs_count_;
  std::unordered_map<int, size_t> msgs_size_;
  std::mutex mutex_;
  rclcpp::TimerBase::SharedPtr timer_;

  void init() {
    declareParameters();
    createSubscribers();
    createTimer();
  }
  void declareParameters();
  void createSubscribers();
  void createTimer();
};
}  // namespace yo_exp2

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(yo_exp2::ComposableSub)

#endif  // COMPOSABLESUB_H

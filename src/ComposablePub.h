//
// Created by yossi on 11/12/24.
//

#ifndef COMPOSABLEPUB_H
#define COMPOSABLEPUB_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

inline std::string formatNumber(int number);

namespace yo_exp2 {
class ComposablePub final : public rclcpp::Node {
 public:
  ComposablePub() : Node("composable_pub") { init(); }
  explicit ComposablePub(const rclcpp::NodeOptions& options) : Node("composable_pub", options) { init(); }

 private:
  std::vector<rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr> publishers_;
  rclcpp::TimerBase::SharedPtr timer_;

  void init();
  void declareParameters();
  void createPublishers();
  void createTimer();
};
}  // namespace yo_exp2

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(yo_exp2::ComposablePub)

#endif  // COMPOSABLEPUB_H

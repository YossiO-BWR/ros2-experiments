#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <thread>
#include "std_msgs/msg/u_int8_multi_array.hpp"

using namespace std::chrono_literals;

    std::string formatNumber(int number) {
        char buffer[4];
        snprintf(buffer, sizeof(buffer), "%03d", number);
        return std::string(buffer);
    }

class MyNode : public rclcpp::Node {
public:
    MyNode() : Node("node_a") {
        declare_parameter<int>("pubs_count");
        declare_parameter<int>("queue_size");
        declare_parameter<double>("hz");
        declare_parameter<int>("data_size");

        int pubs_count = 2;
        get_parameter("pubs_count", pubs_count);
        int queue_size = 1;
        get_parameter("queue_size", queue_size);
        publishers_.reserve(pubs_count);
        for (int i = 0; i < pubs_count; i++) {
            publishers_.emplace_back(
                create_publisher<std_msgs::msg::UInt8MultiArray>("topic_" + formatNumber(i), queue_size));
        }
        RCLCPP_INFO(get_logger(), "Created %lu  publishers", publishers_.size());

        std_msgs::msg::UInt8MultiArray msg; {
            size_t data_size = 30;
            get_parameter("data_size", data_size);
            msg.data.resize(data_size);
        }

        double hz = 1;
        get_parameter("hz", hz);
        size_t dt = 1e6 / hz;


        RCLCPP_INFO(get_logger(), "Publishing a message with size: %lu at rate %f hz", msg.data.size(), hz);
        timer_ = create_wall_timer(std::chrono::microseconds(dt),[this, msg]() {
                for (const auto &pub: publishers_) {
                    pub->publish(msg);
                }
        });
    }

private:
    using PubT = rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr;
    std::vector<PubT> publishers_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv); {
        auto node = std::make_shared<MyNode>();
        rclcpp::spin(node);
    }
    rclcpp::shutdown();

    return 0;
}

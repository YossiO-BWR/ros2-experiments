#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8MultiArray.h>
#include <vector>
#include <string>

class MyNode {
public:
    MyNode() {
        ros::NodeHandle private_nh("~");

        int pubs_count = 2;
        private_nh.getParam("pubs_count", pubs_count);

        int queue_size = 1;
        private_nh.getParam("queue_size", queue_size);
        publishers_.reserve(pubs_count);

        for (int i = 0; i < pubs_count; ++i) {
            publishers_.emplace_back(
                private_nh.advertise<std_msgs::UInt8MultiArray>("topic_" + formatNumber(i), queue_size));
        }
        ROS_INFO("Total pubs: %lu (%d)", publishers_.size(), pubs_count);

        int data_size = 30;
        private_nh.getParam("data_size", data_size);
        msg_.data.resize(data_size);

        double hz = 1.0;
        private_nh.getParam("hz", hz);
        ros::Duration dt(1.0 / hz);

        timer_ = private_nh.createTimer(dt, &MyNode::timerCallback, this);
    }

private:
    void timerCallback(const ros::TimerEvent&) {
            for (const auto &pub : publishers_) {
                pub.publish(msg_);

        }
    }

    std::string formatNumber(int number) {
        char buffer[4];
        snprintf(buffer, sizeof(buffer), "%03d", number);
        return std::string(buffer);
    }

    std_msgs::UInt8MultiArray msg_;
    std::vector<ros::Publisher> publishers_;
    ros::Timer timer_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "node_a");
    MyNode my_node;
    ros::spin();
    return 0;
}

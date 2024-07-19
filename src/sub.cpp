// #include "tf_test/tf_test.hpp"

// C++ system
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <std_msgs/msg/detail/float64__struct.hpp>
#include <string>
#include <vector>

#include "std_msgs/msg/float64.hpp"
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <serviceinterface.h> // local
// #include <Poco/Net/ServerSocket.h> // system

namespace mini_latency {

class Sub: public rclcpp::Node {
private:
    // 声明一个订阅者
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr command_subscribe;
    // 收到话题数据的回调函数
    void command_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        auto now = this->get_clock()->now().seconds();
        double diff = now - msg->data;
        RCLCPP_INFO(this->get_logger(), "I heard: '%f', diff: '%f'", msg->data, diff);
    }

public:
    explicit Sub(const rclcpp::NodeOptions& options): Node("mini_latency_sub", options) {
        RCLCPP_INFO(this->get_logger(), "hello");
        this->command_subscribe = this->create_subscription<std_msgs::msg::Float64>(
            "command",
            10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) { this->command_callback(msg); }
        );
    };

    ~Sub() override = default;

private:
};

} // namespace mini_latency

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(mini_latency::Sub)

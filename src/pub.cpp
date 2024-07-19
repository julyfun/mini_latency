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

class Pub: public rclcpp::Node {
private:
    rclcpp::TimerBase::SharedPtr timer;
    // 声明话题发布者指针
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr command_publisher;

public:
    explicit Pub(const rclcpp::NodeOptions& options): Node("mini_latency_pub", options) {
        RCLCPP_INFO(this->get_logger(), "hello");
        this->command_publisher = this->create_publisher<std_msgs::msg::Float64>("command", 10);
        // 创建定时器，500ms为周期，定时发布
        this->timer = this->create_wall_timer(std::chrono::milliseconds(500), [this]() {
            this->timer_callback();
        });
    };

    ~Pub() override = default;

private:
    void timer_callback() {
        // 创建消息
        std_msgs::msg::Float64 message;
        // get time
        auto now = this->get_clock()->now();
        message.data = now.seconds();
        RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.data);
        // 日志打印
        this->command_publisher->publish(message);
    }
};

} // namespace mini_latency

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(mini_latency::Pub)

//获取键盘消息控制机械臂的末端姿态

#include <termios.h>
#include <unistd.h>

#include <complex>
#include <cstddef>
#include <cstdio>
#include <custom_interfaces/msg/detail/key__struct.hpp>
#include <functional>
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/utilities.hpp>

#include "custom_interfaces/msg/key.hpp"

using namespace std::chrono_literals;

class KeyboardNode : public rclcpp::Node {
private:
    rclcpp::TimerBase::SharedPtr timer_;
    custom_interfaces::msg::Key key;
    rclcpp::Publisher<custom_interfaces::msg::Key>::SharedPtr publisher_;

public:
    KeyboardNode() : Node("keyboard_node") {
        publisher_ = this->create_publisher<custom_interfaces::msg::Key>("key", 10);
        timer_ = this->create_wall_timer(1ms, std::bind(&KeyboardNode::timer_callback, this));
    }

    void timer_callback() {
        this->key.key_code = getch();
        RCLCPP_INFO(this->get_logger(), "Pressed :'%c'", key.key_code);
        auto message = custom_interfaces::msg::Key();
        message.key_code = this->key.key_code;
        publisher_->publish(message);
    }

    /**获取键盘输入 */
    int getch() {
        struct termios oldt, newt;
        int ch;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        ch = getchar();
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        return ch;
    }
};

int main(int argc, char *argv[]) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KeyboardNode>());
    rclcpp::shutdown();
    return 0;
}
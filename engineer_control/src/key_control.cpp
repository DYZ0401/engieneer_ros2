//获取键盘消息控制机械臂的末端姿态

#include <termios.h>
#include <unistd.h>

#include <complex>
#include <cstddef>
#include <cstdio>
#include <custom_interfaces/msg/detail/key__struct.hpp>
#include <functional>
#include <memory>
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
    size_t                                                    count_;
    rclcpp::TimerBase::SharedPtr                              timer_;
    int                                                       key;
    rclcpp::Publisher<custom_interfaces::msg::Key>::SharedPtr publisher_;

public:
    KeyboardNode() : Node("keyboard_node") {
        publisher_ = this->create_publisher<custom_interfaces::msg::Key>("key", 10);
        timer_     = this->create_wall_timer(1ms, std::bind(&KeyboardNode::timer_callback, this));
    }

    void timer_callback() {
        auto message     = custom_interfaces::msg::Key();
        message.key_code = this->key;
        publisher_->publish(message);
    }

    /**获取键盘输入 */
    int getch() {
        struct termios oldt, newt;
        int            ch;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        ch = getchar();
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        return ch;
    }

    void run() {
        while (rclcpp::ok()) {
            this->key = getch();
            RCLCPP_INFO(this->get_logger(), "Pressed :'%c'", key);
        }
    }
};

int main(int argc, char *argv[]) {

    rclcpp::init(argc, argv);
    auto keyboard_node = std::make_shared<KeyboardNode>();
    keyboard_node->run();
    rclcpp::shutdown();
    return 0;
}
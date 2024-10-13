//接收控制消息（如键盘），转化为对六个自由度的控制

#include <custom_interfaces/msg/ctrl_dof.hpp>
#include <custom_interfaces/msg/detail/ctrl_dof__struct.hpp>
#include <custom_interfaces/msg/key.hpp>
#include <functional>
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/utilities.hpp>

#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class ControlSchedulingNode : public rclcpp::Node {
private:
    bool initail_flag = false;
    custom_interfaces::msg::Key key;
    custom_interfaces::msg::CtrlDof ctrl_dof;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<custom_interfaces::msg::CtrlDof>::SharedPtr publisher_;
    rclcpp::Subscription<custom_interfaces::msg::Key>::SharedPtr subscription_;

    void timer_callback() {
        if (!initail_flag) {
            this->init_dof();
            initail_flag = true;
        }
        auto message = custom_interfaces::msg::CtrlDof();
        message = this->ctrl_dof;
        publisher_->publish(message);
    }

    void key_topic_callback(const custom_interfaces::msg::Key& key) {
        this->key = key;
        this->ctrl_to_dof();
        RCLCPP_INFO(this->get_logger(), "key_code: %c", this->key.key_code);
    }

    //把控制消息改变自由度的控制
    void ctrl_to_dof() {
        float ctrl_proportion = 0.2F;
        switch (this->key.key_code) {
            case 'w':
                this->ctrl_dof.x += ctrl_proportion;
                break;
            case 's':
                this->ctrl_dof.x -= ctrl_proportion;
                break;
        }
    }

    void declare_dof_param() {
        this->declare_parameter<double>("initial_ctrl_dof.x", 0);
        this->declare_parameter<double>("initial_ctrl_dof.y", 0);
        this->declare_parameter<double>("initial_ctrl_dof.z", 0);
        this->declare_parameter<double>("initial_ctrl_dof.pitch", 0);
        this->declare_parameter<double>("initial_ctrl_dof.roll", 0);
        this->declare_parameter<double>("initial_ctrl_dof.yaw", 0);
        this->declare_parameter<double>("initial_ctrl_dof.arm_yaw", 0);
        this->declare_parameter<double>("initial_ctrl_dof.arm_pitch", 0);
    }

    void init_dof() {

        this->get_parameter("initial_ctrl_dof.x", this->ctrl_dof.x);
        this->get_parameter("initial_ctrl_dof.y", this->ctrl_dof.y);
        this->get_parameter("initial_ctrl_dof.z", this->ctrl_dof.z);
        this->get_parameter("initial_ctrl_dof.yaw", this->ctrl_dof.yaw);
        this->get_parameter("initial_ctrl_dof.pitch", this->ctrl_dof.pitch);
        this->get_parameter("initial_ctrl_dof.roll", this->ctrl_dof.roll);
        this->get_parameter("initial_ctrl_dof.arm_yaw", this->ctrl_dof.arm_yaw);
        this->get_parameter("initial_ctrl_dof.arm_pitch", this->ctrl_dof.arm_pitch);
        RCLCPP_INFO(this->get_logger(), "initial_ctrl_dof.x: %f",
                    this->get_parameter("initial_ctrl_dof.x").get_value<double>());
    }

public:
    ControlSchedulingNode() : Node("control_scheduling") {
        subscription_ = this->create_subscription<custom_interfaces::msg::Key>(
            "key", 10, std::bind(&ControlSchedulingNode::key_topic_callback, this, _1));
        publisher_ = this->create_publisher<custom_interfaces::msg::CtrlDof>("ctrl_dof", 10);
        timer_ = this->create_wall_timer(5000ms, std::bind(&ControlSchedulingNode::timer_callback, this));
        this->declare_dof_param();
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlSchedulingNode>());
    rclcpp::shutdown();
    return 0;
}
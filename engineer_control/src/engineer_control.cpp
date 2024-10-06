#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <Eigen/Dense>
#include <cstdio>
#include <custom_interfaces/msg/ctrl_dof.hpp>
#include <functional>
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <memory>
#include <moveit_msgs/msg/detail/constraints__struct.hpp>
#include <moveit_msgs/msg/detail/joint_constraint__struct.hpp>
#include <numbers>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/utilities.hpp>
#include <rviz_visual_tools/rviz_visual_tools.hpp>
#include <thread>
#include <utility>

using std::placeholders::_1;

class EngineerControlNode : public rclcpp::Node {
private:
    custom_interfaces::msg::CtrlDof ctrl_dof_mm;  //以mm和度为单位的控制量
    custom_interfaces::msg::CtrlDof ctrl_dof_m;   //以m和rad为单位的控制量
    rclcpp::Subscription<custom_interfaces::msg::CtrlDof>::SharedPtr subscription_;

    void ctrl_dof_topic_callback(const custom_interfaces::msg::CtrlDof& ctrl_dof) {
        this->ctrl_dof_mm = ctrl_dof;
        this->unit_conversion();
    }

    void unit_conversion() {
        this->ctrl_dof_m.x = this->ctrl_dof_mm.x / 100.F;
        this->ctrl_dof_m.y = this->ctrl_dof_mm.y / 100.F;
        this->ctrl_dof_m.z = this->ctrl_dof_mm.z / 100.F;
        this->ctrl_dof_m.roll = this->ctrl_dof_mm.roll / 180.F * std::numbers::pi;
        this->ctrl_dof_m.yaw = this->ctrl_dof_mm.yaw / 180.F * std::numbers::pi;
        this->ctrl_dof_m.pitch = this->ctrl_dof_mm.pitch / 180.F * std::numbers::pi;
        this->ctrl_dof_m.arm_yaw = this->ctrl_dof_mm.arm_yaw / 180.F * std::numbers::pi;
        this->ctrl_dof_m.arm_pitch = this->ctrl_dof_mm.arm_pitch / 180.F * std::numbers::pi;
    }

public:
    EngineerControlNode() : Node("engineer_control") {
        subscription_ = this->create_subscription<custom_interfaces::msg::CtrlDof>(
            "ctrl_dof", 10, std::bind(&EngineerControl::ctrl_dof_topic_callback, this, _1));
    }
};

int main(int argc, char** argv) {

    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "engineer_control", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    auto const logger = rclcpp::get_logger("engineer_control");

    rclcpp::executors::SingleThreadedExecutor excutor;
    excutor.add_node(node);
    auto spinner = std::thread([&excutor]() {
        excutor.spin();
    });

    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "arm");

    // gui注释
    auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
        node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC, move_group_interface.getRobotModel()};
    moveit_visual_tools.deleteAllMarkers();
    moveit_visual_tools.loadRemoteControl();

    auto const draw_title = [&moveit_visual_tools](auto text) {
        auto const text_pose = [] {
            auto msg = Eigen::Isometry3d::Identity();
            msg.translation().z() = 1.0;
            return msg;
        }();
        moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    };

    auto const prompt = [&moveit_visual_tools](auto text) {
        moveit_visual_tools.prompt(text);
    };

    auto const draw_trajectory_tool_path =
        [&moveit_visual_tools,
         jmg = move_group_interface.getRobotModel()->getJointModelGroup("arm")](auto const trajectory) {
            moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);
        };

    //定义冗余关节角度
    float arm_yaw_set = 0.0f;
    float arm_pitch_set = 0.0f;
    // 声明目标位置

    moveit_msgs::msg::Constraints constraints;
    moveit_msgs::msg::JointConstraint joint_constraint;

    joint_constraint.joint_name = "arm_yaw";
    joint_constraint.position = arm_yaw_set;
    joint_constraint.tolerance_above = 0.001;
    joint_constraint.tolerance_below = 0.001;
    constraints.joint_constraints.push_back(joint_constraint);

    joint_constraint.joint_name = "arm_pitch";
    joint_constraint.position = arm_pitch_set;
    joint_constraint.tolerance_above = 0.001;
    joint_constraint.tolerance_below = 0.001;
    constraints.joint_constraints.push_back(joint_constraint);

    move_group_interface.setPathConstraints(constraints);

    auto const target_pose = [] {
        geometry_msgs::msg::Pose msg;
        msg.orientation.w = 0.0;
        msg.position.x = 0.40;
        msg.position.y = 0.07;
        msg.position.z = 0.5;
        return msg;
    }();

    move_group_interface.setPoseReferenceFrame("engineer_frame");
    move_group_interface.setPoseTarget(target_pose);

    auto const [success, plan] = [&move_group_interface] {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();

    if (success) {
        draw_trajectory_tool_path(plan.trajectory_);
        moveit_visual_tools.trigger();
        prompt("Press 'Next' in the RvizVisualToolsGui window to excute");
        draw_title("Executing");
        moveit_visual_tools.trigger();
        move_group_interface.execute(plan);
    } else {
        draw_title("Planning Failed!");
        moveit_visual_tools.trigger();
        RCLCPP_ERROR(logger, "Planning failed");
    }

    rclcpp::shutdown();
    spinner.join();
    return 0;
}

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <Eigen/Dense>
#include <cstdio>
#include <custom_interfaces/msg/ctrl_dof.hpp>
#include <functional>
#include <geometry_msgs/msg/pose.hpp>
#include <memory>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
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

using moveit::planning_interface::MoveGroupInterface;
using std::placeholders::_1;

class EngineerControlNode : public rclcpp::Node {
private:
    bool initial_flag = false;                    //避免在构造函数中使用shared_form_this
    custom_interfaces::msg::CtrlDof ctrl_dof_mm;  //以mm和度为单位的控制量
    custom_interfaces::msg::CtrlDof ctrl_dof_m;   //以m和rad为单位的控制量
    rclcpp::Subscription<custom_interfaces::msg::CtrlDof>::SharedPtr subscription_;

    std::shared_ptr<MoveGroupInterface> move_group_interface_;
    std::shared_ptr<moveit_visual_tools::MoveItVisualTools> moveit_visual_tools_;

    // gui绘制
    void draw_title(auto text) {
        auto const text_pose = [] {
            auto msg = Eigen::Isometry3d::Identity();
            msg.translation().z() = 1.0;
            return msg;
        }();
        moveit_visual_tools_->publishText(text_pose, text, rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    }

    void prompt(auto text) {
        moveit_visual_tools_->prompt(text);
    }

    void draw_trajectory_tool_path(auto trajectory) {
        moveit_visual_tools_->publishTrajectoryLine(trajectory,
                                                    move_group_interface_->getRobotModel()->getJointModelGroup("arm"));
    }

    void init() {
        move_group_interface_ = std::make_shared<MoveGroupInterface>(shared_from_this(), "arm");
        moveit_visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
            shared_from_this(), "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
            move_group_interface_->getRobotModel());

        moveit_visual_tools_->deleteAllMarkers();
        moveit_visual_tools_->loadRemoteControl();
        move_group_interface_->setStartStateToCurrentState();
        move_group_interface_->clearPathConstraints();
        move_group_interface_->setPlanningTime(100.0);
        // move_group_interface_->setPoseReferenceFrame("engineer_frame");
    }

    //话题节点通讯
    void ctrl_dof_topic_callback(const custom_interfaces::msg::CtrlDof& ctrl_dof) {
        if (!initial_flag) {  //避免在构造函数中使用shared_from_this
            this->init();
            initial_flag = true;
        }
        this->ctrl_dof_mm = ctrl_dof;
        // this->unit_conversion();
        // this->limit_redundant_degrees_of_freedom();
        this->set_pose();
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

    //限制冗余自由度
    void limit_redundant_degrees_of_freedom() {

        moveit_msgs::msg::Constraints constraints;
        moveit_msgs::msg::JointConstraint joint_constraint;
        move_group_interface_->clearPathConstraints();
        joint_constraint.joint_name = "arm_yaw_joint";
        joint_constraint.position = 0.0;
        joint_constraint.tolerance_above = 10;
        joint_constraint.tolerance_below = 10;
        joint_constraint.weight = 1.0;
        constraints.joint_constraints.push_back(joint_constraint);

        joint_constraint.joint_name = "arm_pitch_joint";
        joint_constraint.position = 0.0;
        joint_constraint.tolerance_above = 10;
        joint_constraint.tolerance_below = 10;
        joint_constraint.weight = 1.0;
        constraints.joint_constraints.push_back(joint_constraint);

        move_group_interface_->setPathConstraints(constraints);
    }

    void set_pose() {
        auto const target_pose = [this] {
            geometry_msgs::msg::Pose msg;
            tf2::Quaternion q;
            // q.setRPY(this->ctrl_dof_m.roll, this->ctrl_dof_m.pitch, this->ctrl_dof_m.yaw);
            // q.setRPY(0, 0, 0);

            // msg.orientation.w = q.w();
            // msg.orientation.x = q.x();
            // msg.orientation.y = q.y();
            // msg.orientation.z = q.z();

            // // msg.position.x = this->ctrl_dof_m.x;
            // // msg.position.y = this->ctrl_dof_m.y;
            // // msg.position.z = this->ctrl_dof_m.z;
            // msg.position.x = 0.6;
            // msg.position.y = 0.2;
            // msg.position.z = 0.5;

            msg.orientation.w = 1.0;
            msg.position.x = 0.28;
            msg.position.y = -0.2;
            msg.position.z = 0.5;
            return msg;
        }();

        if (!move_group_interface_->setPoseTarget(target_pose)) {
            RCLCPP_ERROR(this->get_logger(), "Pose Error");
        }

        auto const [success, plan] = [this] {
            moveit::planning_interface::MoveGroupInterface::Plan msg;
            auto const ok = static_cast<bool>(this->move_group_interface_->plan(msg));
            return std::make_pair(ok, msg);
        }();

        if (success) {
            draw_trajectory_tool_path(plan.trajectory_);
            moveit_visual_tools_->trigger();
            prompt("Press 'Next' in the RvizVisualToolsGui window to excute");
            draw_title("Executing");
            moveit_visual_tools_->trigger();
            move_group_interface_->execute(plan);
        } else {
            draw_title("Planning Failed!");
            moveit_visual_tools_->trigger();
            RCLCPP_ERROR(this->get_logger(), "Planning failed");
        }
    }

public:
    EngineerControlNode()
        : Node("engineer_control", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {
        subscription_ = this->create_subscription<custom_interfaces::msg::CtrlDof>(
            "ctrl_dof", 10, std::bind(&EngineerControlNode::ctrl_dof_topic_callback, this, _1));
    }
};

int main(int argc, char** argv) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EngineerControlNode>());

    rclcpp::shutdown();
    return 0;
}

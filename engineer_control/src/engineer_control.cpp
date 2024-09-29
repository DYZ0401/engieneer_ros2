#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <Eigen/Dense>
#include <cstdio>
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <memory>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <rviz_visual_tools/rviz_visual_tools.hpp>
#include <thread>
#include <utility>

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

    auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
        node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC, move_group_interface.getRobotModel()};
    moveit_visual_tools.deleteAllMarkers();
    moveit_visual_tools.loadRemoteControl();

    auto const draw_title = [&moveit_visual_tools](auto text) {
        auto const text_pose = [] {
            auto msg              = Eigen::Isometry3d::Identity();
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

    auto const target_pose = [] {
        geometry_msgs::msg::Pose msg;
        msg.orientation.w = 1.0;
        msg.position.x    = 0.28;
        msg.position.y    = -0.2;
        msg.position.z    = 0.5;
        return msg;
    }();

    move_group_interface.setPoseTarget(target_pose);

    auto const [success, plan] = [&move_group_interface] {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const                                           ok = static_cast<bool>(move_group_interface.plan(msg));
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

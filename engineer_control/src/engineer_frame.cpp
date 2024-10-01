
//声明工程上的控制坐标系与base_link的关系，方便与下位机统一

#include <tf2_ros/static_transform_broadcaster.h>

#include <cstdlib>
#include <cstring>
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

class StaticFramePublisher : public rclcpp::Node {
public:
    explicit StaticFramePublisher(char *transformation[]) : Node("engineer_frame") {
        tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        this->make_transforms(transformation);
    }

private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

    void make_transforms(char *transformation[]) {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp    = this->get_clock()->now();
        t.header.frame_id = "base_link";
        t.child_frame_id  = transformation[1];

        t.transform.translation.x = atof(transformation[2]);
        t.transform.translation.y = atof(transformation[3]);
        t.transform.translation.z = atof(transformation[4]);
        tf2::Quaternion q;
        q.setRPY(atof(transformation[5]), atof(transformation[6]), atof(transformation[7]));
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        tf_static_broadcaster_->sendTransform(t);
    }
};

int main(int argc, char *argv[]) {
    auto logger = rclcpp::get_logger("engieneer_frame");

    if (argc != 8) {
        RCLCPP_INFO(logger, "Invalid number of parameters\nusage: "
                            "$ ros2 run learning_tf2_cpp static_turtle_tf2_broadcaster "
                            "child_frame_name x y z roll pitch yaw");
        return 1;
    }

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StaticFramePublisher>(argv));
    rclcpp::shutdown();
    return 0;
}
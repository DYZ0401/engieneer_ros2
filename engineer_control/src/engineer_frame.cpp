
//声明工程上的控制坐标系与base_link的关系，方便与下位机统一

#include <tf2_ros/static_transform_broadcaster.h>

#include <cstdlib>
#include <cstring>
#include <memory>
#include <numbers>
#include <rclcpp/executors.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

struct frame_data_t {
    std::string frame_id;
    std::string child_id;
    float       x;
    float       y;
    float       z;
    float       yaw;
    float       pitch;
    float       roll;
};

const frame_data_t engineer_frame = {
    "base_link", "engineer_frame", -0.36, 0.08, 0.30, -3.1415926 / 2.f, 0, 0,
};

class StaticFramePublisher : public rclcpp::Node {
public:
    explicit StaticFramePublisher(const frame_data_t *data) : Node("engineer_frame") {
        tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        this->make_transforms(data);
    }

private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

    void make_transforms(const frame_data_t *data) {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp    = this->get_clock()->now();
        t.header.frame_id = data->frame_id;
        t.child_frame_id  = data->child_id;

        t.transform.translation.x = data->x;
        t.transform.translation.y = data->y;
        t.transform.translation.z = data->z;
        tf2::Quaternion q;
        q.setRPY(data->roll, data->pitch, data->yaw);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        tf_static_broadcaster_->sendTransform(t);
    }
};

int main(int argc, char *argv[]) {
    auto logger = rclcpp::get_logger("engieneer_frame");
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StaticFramePublisher>(&engineer_frame));
    rclcpp::shutdown();
    return 0;
}
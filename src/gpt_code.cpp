


#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

class UavFramePublisher : public rclcpp::Node {
public:
    UavFramePublisher() : Node("uav_frame_publisher"), theta(0.) {
        uav1_name_ = this->declare_parameter<std::string>("frame_name", "uav1");
        uav2_name_ = this->declare_parameter<std::string>("frame_name", "uav2");

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "winch1/uav_pose", 10, std::bind(&UavFramePublisher::callback_pub_uav1_frame, this, _1)
        );
    }

private:
    void callback_pub_uav1_frame(const std::shared_ptr<geometry_msgs::msg::Pose> msg) {
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "winch1";
        t.child_frame_id = uav1_name_;

        t.transform.translation.x = msg->position.x;
        t.transform.translation.y = msg->position.y;
        t.transform.translation.z = 0.0;
        theta = std::atan2(msg->position.y, msg->position.x);

        tf2::Quaternion q;
        q.setRPY(0, 0, theta);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        // Send the transformation
        tf_broadcaster_->sendTransform(t);

        // Call second function
        callback_pub_uav2_frame();
    }

    void callback_pub_uav2_frame() {
        geometry_msgs::msg::TransformStamped t_listener;
        geometry_msgs::msg::TransformStamped t_broadcaster;

        try {
            t_listener = tf_buffer_->lookupTransform("winch2", "uav1", tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
            RCLCPP_INFO(this->get_logger(), "Could not transform in uav2_frame_publisher: %s", ex.what());
            return;
        }

        t_broadcaster.header.stamp = this->get_clock()->now();
        t_broadcaster.header.frame_id = "winch2";
        t_broadcaster.child_frame_id = uav2_name_;

        t_broadcaster.transform.translation.x = t_listener.transform.translation.x;
        t_broadcaster.transform.translation.y = t_listener.transform.translation.y;
        t_broadcaster.transform.translation.z = t_listener.transform.translation.z;
        theta = std::atan2(t_listener.transform.translation.y, t_listener.transform.translation.x);

        tf2::Quaternion q;
        q.setRPY(0, 0, theta);
        t_broadcaster.transform.rotation.x = q.x();
        t_broadcaster.transform.rotation.y = q.y();
        t_broadcaster.transform.rotation.z = q.z();
        t_broadcaster.transform.rotation.w = q.w();

        // Send the transformation
        tf_broadcaster_->sendTransform(t_broadcaster);
    }

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::string uav1_name_;
    std::string uav2_name_;
    double theta;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UavFramePublisher>());
    rclcpp::shutdown();
    return 0;
}




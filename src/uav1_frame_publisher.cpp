
/*
porpose: to broadcast uav1 frame which is from winch1 

1. subscribe geometry_msgs/msg/Pose which is uav position.
2. In subscription callback function "callback_pub_uav1_frame", 
    put x,y,z & phi,theta,psi (from winch1 to uav1) to geometry_msgs/msg/TransformStamped
    and sendTransform().
*/



#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using std::placeholders::_1;

class Uav1FramePublisher : public rclcpp::Node{
public:
    Uav1FramePublisher() : Node("uav1_frame_publisher"), theta(0.){
        uavname_ = this->declare_parameter<std::string>("frame_name", "uav1");

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        subscription_ = this -> create_subscription<geometry_msgs::msg::Pose>(
            "winch1/uav_pose", 10, std::bind(&Uav1FramePublisher::callback_pub_uav1_frame, this, _1)
        );
    }
private:
    void callback_pub_uav1_frame(const std::shared_ptr<geometry_msgs::msg::Pose> msg)
    {
      geometry_msgs::msg::TransformStamped t;

      t.header.stamp = this->get_clock()->now();
      t.header.frame_id = "winch1";
      t.child_frame_id = "uav1";  // uav1

      t.transform.translation.x = msg->position.x;  // msg.position.x
      t.transform.translation.y = msg->position.y;  // msg.position.y
      t.transform.translation.z = 0.0;
      theta = std::atan2(msg->position.y, msg->position.x);

      tf2::Quaternion q;
      q.setRPY(0, 0, theta);   // eular x,y,z
      t.transform.rotation.x = q.x(); // hennaknngono q wo dainyuu
      t.transform.rotation.y = q.y();
      t.transform.rotation.z = q.z();
      t.transform.rotation.w = q.w();

      // Send the transformation
      tf_broadcaster_->sendTransform(t);
    }

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::string uavname_;
    double theta;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Uav1FramePublisher>());
    rclcpp::shutdown();
    return 0;
}


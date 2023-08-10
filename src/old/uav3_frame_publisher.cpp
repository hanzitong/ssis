
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

using namespace std::chrono_literals;

class Uav3FramePublisher : public rclcpp::Node{
public:
    Uav3FramePublisher() : Node("uav3_frame_publisher"), theta(0.){
        uavname_ = this->declare_parameter<std::string>("frame_name", "uav3");

        // Initialize the transform broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        timer_ = this -> create_wall_timer(100ms, std::bind(&Uav3FramePublisher::callback_pub_uav3_frame, this));
    }
private:
    void callback_pub_uav3_frame(){
        geometry_msgs::msg::TransformStamped t_listener;
        geometry_msgs::msg::TransformStamped t_broadcaster;

        try{
            t_listener = tf_buffer_ -> lookupTransform("winch3","uav1",tf2::TimePointZero);
        }catch(const tf2::TransformException & ex){
            RCLCPP_INFO(this->get_logger(), "Could not transform in uav3_frame_publisher: %s", ex.what());
            return;
        }

        t_broadcaster.header.stamp = this->get_clock()->now();
        t_broadcaster.header.frame_id = "winch3";
        t_broadcaster.child_frame_id = "uav3";

        t_broadcaster.transform.translation.x = t_listener.transform.translation.x;
        t_broadcaster.transform.translation.y = t_listener.transform.translation.y;
        t_broadcaster.transform.translation.z = t_listener.transform.translation.z;
        theta = std::atan2(t_listener.transform.translation.y, t_listener.transform.translation.x);

        tf2::Quaternion q;
        q.setRPY(0, 0, theta);   // eular x,y,z
        t_broadcaster.transform.rotation.x = q.x();
        t_broadcaster.transform.rotation.y = q.y();
        t_broadcaster.transform.rotation.z = q.z();
        t_broadcaster.transform.rotation.w = q.w();

        // Send the transformation
        tf_broadcaster_->sendTransform(t_broadcaster);
    }

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    std::string uavname_;
    double theta;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Uav3FramePublisher>());
    rclcpp::shutdown();
    return 0;
}




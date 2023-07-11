



#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tether_msgs/msg/tether_compare.hpp>
#include <smart_tether/tether_opt.hpp>
#include <smart_tether/multi_tether_opt.hpp>
#include <smart_tether/c_bounds.hpp>

#include <visualization_msgs/msg/marker.hpp>


using namespace std::chrono_literals;

#define RHO_VALUE 0.045
#define S_MARGIN 0.5
#define V_MARGIN 20.



class MinTuav1tether : public rclcpp::Node{
public:
    MinTuav1tether() : Node("min_tuav_1tether")//, c_opt_{0.}, c_upper_{0.}, c_lower_{0.}
    {
        // tf listener 
        // tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        // tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        subscription_ = this->create_subscription<tether_msgs::msg::TetherCompare>("", 10, std::bind(&MinimalSubscriber::tuav1_callback, this, _1));
        tuav1_marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("marker_tuav1", 1);
        // timer_ = this->create_wall_timer(0.5s, std::bind(&MinTuav1tether::on_timer, this));
    }

private:
    void tuav1_callback(){
        // RCLCPP_INFO(this->get_logger(),"call on_timer()");
        // geometry_msgs::msg::TransformStamped t1;
        // geometry_msgs::msg::TransformStamped t2;
        // geometry_msgs::msg::TransformStamped t3;
        tether_msgs::msg::TetherCompare msg;

        visualization_msgs::msg::Marker tuav1_marker;
        visualization_msgs::msg::Marker tuav2_marker;
        visualization_msgs::msg::Marker tuav3_marker;

        // try{
        //     t1 = tf_buffer_->lookupTransform("winch1", "uav1", tf2::TimePointZero);
        //     t2 = tf_buffer_->lookupTransform("winch1", "uav2", tf2::TimePointZero);
        //     t3 = tf_buffer_->lookupTransform("winch1", "uav3", tf2::TimePointZero);
        // }catch(const tf2::TransformException & ex) {
        //     RCLCPP_INFO(this->get_logger(), "Could not transform : %s", ex.what());
        //     return;
        // }

        // // tf listener here. get quaternion & uav_position
        // t1 = tf_buffer_ -> lookupTransform("winch1", "uav1", tf2::TimePointZero);
        // t2 = tf_buffer_ -> lookupTransform("winch1", "uav2", tf2::TimePointZero);
        // t3 = tf_buffer_ -> lookupTransform("winch1", "uav3", tf2::TimePointZero);


        // visualization 
        tuav1_marker.header.frame_id = "uav1";
        tuav1_marker.header.stamp = this->get_clock()->now();
        tuav1_marker.ns = "tuav1";
        tuav1_marker.id = 0;
        tuav1_marker.type = visualization_msgs::msg::Marker::ARROW;
        tuav1_marker.action = visualization_msgs::msg::Marker::ADD;
        tuav1_marker.pose.position.x = 10.0;
        tuav1_marker.pose.position.y = 10.0;
        tuav1_marker.pose.position.z = 10.0;
        tuav1_marker.pose.orientation.x = q2to1.x();
        tuav1_marker.pose.orientation.y = q2to1.y();
        tuav1_marker.pose.orientation.z = q2to1.z();
        tuav1_marker.pose.orientation.w = q2to1.w();
        tuav1_marker.scale.x = 10.1;  // 矢印の幅
        tuav1_marker.scale.y = 10.2;  // 矢印の先端のサイズ
        tuav1_marker.scale.z = 10.2;  // 矢印の先端のサイズ
        tuav1_marker.color.r = 1.0;
        tuav1_marker.color.g = 0.0;
        tuav1_marker.color.b = 0.0;
        tuav1_marker.color.a = 1.0;
        tuav1_marker.points[0].x = 10.0;
        tuav1_marker.points[0].y = 10.0;
        tuav1_marker.points[0].z = 0.0;
        // tuav1_marker.points[1] = vector;
        
        tuav1_marker_publisher_->publish(tuav1_marker);
        RCLCPP_INFO(this->get_logger(), "end timer() .................");

        return;
    }

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
    rclcpp::Publisher<tether_msgs::msg::TetherCompare>::SharedPtr publisher_{nullptr};
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr tuav1_marker_publisher_{nullptr};
    // rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr tuav2_marker_publisher_{nullptr};
    // rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr tuav3_marker_publisher_{nullptr};
    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    std::vector<double> c_opt1_ = {0.};
    std::vector<double> c_opt12_ = {0., 0.};
    std::vector<double> c_opt123_ = {0., 0., 0.,};
    tf2::Quaternion q2to1;
    tf2::Quaternion q3to1;

    int compare_result;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinTuav1tether>());
    rclcpp::shutdown();
    return 0;
}











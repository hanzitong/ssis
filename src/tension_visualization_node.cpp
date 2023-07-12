
#include <chrono>
#include <functional>
#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <tether_msgs/msg/tether_compare.hpp>

using namespace std::chrono_literals;

#define VECTOR_SCALE_X 1.0
#define VECTOR_SCALE_Y 3.0
#define VECTOR_SCALE_Z 3.0
#define VECTOR_COLOR_R 1.0
#define VECTOR_COLOR_G 0.0
#define VECTOR_COLOR_B 0.0
#define VECTOR_COLOR_A 1.0


class TensionVisualizationNode : public rclcpp::Node
{
public:
    TensionVisualizationNode(): Node("minimal_subscriber")
    {
        subscription_ = this->create_subscription<tether_msgs::msg::TetherCompare>(
                            "optimize_results", 10, std::bind(&TensionVisualizationNode::callback_subscribe_tether, this, std::placeholders::_1)
                        );

        publisher_uav_ = this->create_publisher<visualization_msgs::msg::Marker>("marker_uav", 10);
        publisher1_ = this->create_publisher<visualization_msgs::msg::Marker>("marker_tuav1_1tether", 10);
        publisher2_ = this->create_publisher<visualization_msgs::msg::Marker>("marker_tuav1_2tether", 10);
        publisher3_ = this->create_publisher<visualization_msgs::msg::Marker>("marker_tuav2_2tether", 10);
        publisher4_ = this->create_publisher<visualization_msgs::msg::Marker>("marker_tuav1_3tether", 10);
        publisher5_ = this->create_publisher<visualization_msgs::msg::Marker>("marker_tuav2_3tether", 10);
        publisher6_ = this->create_publisher<visualization_msgs::msg::Marker>("marker_tuav3_3tether", 10);

        publisher7_ = this->create_publisher<visualization_msgs::msg::Marker>("marker_tuav_sum_2tether", 10);
        publisher8_ = this->create_publisher<visualization_msgs::msg::Marker>("marker_tuav_sum_3tether", 10);

    }

private:
    void callback_subscribe_tether(const tether_msgs::msg::TetherCompare &msg_sub)
    {
        // plot uav 
        visualization_msgs::msg::Marker uav_marker;
        uav_marker.header.frame_id = msg_sub.tuav1_1tether.header.frame_id;
        // uav_marker.header.stamp = this->now();
        uav_marker.ns = "uav";
        uav_marker.id = 0;
        uav_marker.type = visualization_msgs::msg::Marker::SPHERE;
        uav_marker.action = visualization_msgs::msg::Marker::ADD;
        uav_marker.pose.position.x = 0.;
        uav_marker.pose.position.y = 0.;
        uav_marker.pose.position.z = 0.;
        uav_marker.scale.x = 7.;
        uav_marker.scale.y = 7.;
        uav_marker.scale.z = 7.;
        uav_marker.color.r = 0.;
        uav_marker.color.g = 1.;
        uav_marker.color.b = 1.;
        uav_marker.color.a = 0.7;

        // tether1 pattern 
        visualization_msgs::msg::Marker msg_pub1;
        msg_pub1.header.frame_id = msg_sub.tuav1_1tether.header.frame_id;
        msg_pub1.ns = "1tether";
        msg_pub1.id = 1;
        msg_pub1.type = visualization_msgs::msg::Marker::ARROW;
        msg_pub1.action = visualization_msgs::msg::Marker::ADD;
        msg_pub1.points.resize(2);
        msg_pub1.points[0].x = 0.;
        msg_pub1.points[0].y = 0.;
        msg_pub1.points[0].z = 0.;
        msg_pub1.points[1].x = msg_sub.tuav1_1tether.vector.x;
        msg_pub1.points[1].y = msg_sub.tuav1_1tether.vector.y;
        msg_pub1.points[1].z = msg_sub.tuav1_1tether.vector.z;
        msg_pub1.scale.x = VECTOR_SCALE_X;
        msg_pub1.scale.y = VECTOR_SCALE_Y;
        msg_pub1.scale.z = VECTOR_SCALE_Z;
        msg_pub1.color.r = VECTOR_COLOR_R;
        msg_pub1.color.g = VECTOR_COLOR_G;
        msg_pub1.color.b = VECTOR_COLOR_B;
        msg_pub1.color.a = VECTOR_COLOR_A;


        // tether2 pattern 
        visualization_msgs::msg::Marker msg_pub2;
        msg_pub2.header.frame_id = msg_sub.tuav1_2tether.header.frame_id;
        msg_pub2.ns = "2tether";
        msg_pub2.id = 1;
        msg_pub2.type = visualization_msgs::msg::Marker::ARROW;
        msg_pub2.action = visualization_msgs::msg::Marker::ADD;
        msg_pub2.points.resize(2);
        msg_pub2.points[0].x = 0.;
        msg_pub2.points[0].y = 0.;
        msg_pub2.points[0].z = 0.;
        msg_pub2.points[1].x = msg_sub.tuav1_2tether.vector.x;
        msg_pub2.points[1].y = msg_sub.tuav1_2tether.vector.y;
        msg_pub2.points[1].z = msg_sub.tuav1_2tether.vector.z;
        msg_pub2.scale.x = VECTOR_SCALE_X;
        msg_pub2.scale.y = VECTOR_SCALE_Y;
        msg_pub2.scale.z = VECTOR_SCALE_Z;
        msg_pub2.color.r = 0.;
        msg_pub2.color.g = 1.;
        msg_pub2.color.b = 0.;
        msg_pub2.color.a = VECTOR_COLOR_A;

        visualization_msgs::msg::Marker msg_pub3;
        msg_pub3.header.frame_id = msg_sub.tuav2_2tether.header.frame_id;
        msg_pub3.ns = "2tether";
        msg_pub3.id = 2;
        msg_pub3.type = visualization_msgs::msg::Marker::ARROW;
        msg_pub3.action = visualization_msgs::msg::Marker::ADD;
        msg_pub3.points.resize(2);
        msg_pub3.points[0].x = 0.;
        msg_pub3.points[0].y = 0.;
        msg_pub3.points[0].z = 0.;
        msg_pub3.points[1].x = msg_sub.tuav2_2tether.vector.x;
        msg_pub3.points[1].y = msg_sub.tuav2_2tether.vector.y;
        msg_pub3.points[1].z = msg_sub.tuav2_2tether.vector.z;
        msg_pub3.scale.x = VECTOR_SCALE_X;
        msg_pub3.scale.y = VECTOR_SCALE_Y;
        msg_pub3.scale.z = VECTOR_SCALE_Z;
        msg_pub3.color.r = 0.;
        msg_pub3.color.g = 1.;
        msg_pub3.color.b = 0.;
        msg_pub3.color.a = VECTOR_COLOR_A;


        // tether3 patter
        visualization_msgs::msg::Marker msg_pub4;
        msg_pub4.header.frame_id = msg_sub.tuav1_3tether.header.frame_id;
        msg_pub4.ns = "3tether";
        msg_pub4.id = 2;
        msg_pub4.type = visualization_msgs::msg::Marker::ARROW;
        msg_pub4.action = visualization_msgs::msg::Marker::ADD;
        msg_pub4.points.resize(2);
        msg_pub4.points[0].x = 0.;
        msg_pub4.points[0].y = 0.;
        msg_pub4.points[0].z = 0.;
        msg_pub4.points[1].x = msg_sub.tuav1_3tether.vector.x;
        msg_pub4.points[1].y = msg_sub.tuav1_3tether.vector.y;
        msg_pub4.points[1].z = msg_sub.tuav1_3tether.vector.z;
        msg_pub4.scale.x = VECTOR_SCALE_X;
        msg_pub4.scale.y = VECTOR_SCALE_Y;
        msg_pub4.scale.z = VECTOR_SCALE_Z;
        msg_pub4.color.r = 0.;
        msg_pub4.color.g = 0.;
        msg_pub4.color.b = 1.;
        msg_pub4.color.a = VECTOR_COLOR_A;

        visualization_msgs::msg::Marker msg_pub5;
        msg_pub5.header.frame_id = msg_sub.tuav2_3tether.header.frame_id;
        msg_pub5.ns = "3tether";
        msg_pub5.id = 2;
        msg_pub5.type = visualization_msgs::msg::Marker::ARROW;
        msg_pub5.action = visualization_msgs::msg::Marker::ADD;
        msg_pub5.points.resize(2);
        msg_pub5.points[0].x = 0.;
        msg_pub5.points[0].y = 0.;
        msg_pub5.points[0].z = 0.;
        msg_pub5.points[1].x = msg_sub.tuav2_3tether.vector.x;
        msg_pub5.points[1].y = msg_sub.tuav2_3tether.vector.y;
        msg_pub5.points[1].z = msg_sub.tuav2_3tether.vector.z;
        msg_pub5.scale.x = VECTOR_SCALE_X;
        msg_pub5.scale.y = VECTOR_SCALE_Y;
        msg_pub5.scale.z = VECTOR_SCALE_Z;
        msg_pub5.color.r = 0.;
        msg_pub5.color.g = 1.;
        msg_pub5.color.b = 1.;
        msg_pub5.color.a = VECTOR_COLOR_A;

        visualization_msgs::msg::Marker msg_pub6;
        msg_pub6.header.frame_id = msg_sub.tuav3_3tether.header.frame_id;
        msg_pub6.ns = "3tether";
        msg_pub6.id = 3;
        msg_pub6.type = visualization_msgs::msg::Marker::ARROW;
        msg_pub6.action = visualization_msgs::msg::Marker::ADD;
        msg_pub6.points.resize(2);
        msg_pub6.points[0].x = 0.;
        msg_pub6.points[0].y = 0.;
        msg_pub6.points[0].z = 0.;
        msg_pub6.points[1].x = msg_sub.tuav3_3tether.vector.x;
        msg_pub6.points[1].y = msg_sub.tuav3_3tether.vector.y;
        msg_pub6.points[1].z = msg_sub.tuav3_3tether.vector.z;
        msg_pub6.scale.x = VECTOR_SCALE_X;
        msg_pub6.scale.y = VECTOR_SCALE_Y;
        msg_pub6.scale.z = VECTOR_SCALE_Z;
        msg_pub6.color.r = 1.;
        msg_pub6.color.g = 0.;
        msg_pub6.color.b = 1.;
        msg_pub6.color.a = VECTOR_COLOR_A;


        visualization_msgs::msg::Marker msg_pub7;
        msg_pub7.header.frame_id = msg_sub.tuav12_sum_vector.header.frame_id;
        msg_pub7.ns = "sum";
        msg_pub7.id = 7;
        msg_pub7.type = visualization_msgs::msg::Marker::ARROW;
        msg_pub7.action = visualization_msgs::msg::Marker::ADD;
        msg_pub7.points.resize(2);
        msg_pub7.points[0].x = 0.;
        msg_pub7.points[0].y = 0.;
        msg_pub7.points[0].z = 0.;
        msg_pub7.points[1].x = msg_sub.tuav12_sum_vector.vector.x;
        msg_pub7.points[1].y = msg_sub.tuav12_sum_vector.vector.y;
        msg_pub7.points[1].z = msg_sub.tuav12_sum_vector.vector.z;
        msg_pub7.scale.x = VECTOR_SCALE_X;
        msg_pub7.scale.y = VECTOR_SCALE_Y;
        msg_pub7.scale.z = VECTOR_SCALE_Z;
        msg_pub7.color.r = 0.;
        msg_pub7.color.g = 1.;
        msg_pub7.color.b = 0.;
        msg_pub7.color.a = VECTOR_COLOR_A;

        visualization_msgs::msg::Marker msg_pub8;
        msg_pub8.header.frame_id = msg_sub.tuav123_sum_vector.header.frame_id;
        msg_pub8.ns = "sum";
        msg_pub8.id = 8;
        msg_pub8.type = visualization_msgs::msg::Marker::ARROW;
        msg_pub8.action = visualization_msgs::msg::Marker::ADD;
        msg_pub8.points.resize(2);
        msg_pub8.points[0].x = 0.;
        msg_pub8.points[0].y = 0.;
        msg_pub8.points[0].z = 0.;
        msg_pub8.points[1].x = msg_sub.tuav123_sum_vector.vector.x;
        msg_pub8.points[1].y = msg_sub.tuav123_sum_vector.vector.y;
        msg_pub8.points[1].z = msg_sub.tuav123_sum_vector.vector.z;
        msg_pub8.scale.x = VECTOR_SCALE_X;
        msg_pub8.scale.y = VECTOR_SCALE_Y;
        msg_pub8.scale.z = VECTOR_SCALE_Z;
        msg_pub8.color.r = 1.;
        msg_pub8.color.g = 0.;
        msg_pub8.color.b = 0.;
        msg_pub8.color.a = VECTOR_COLOR_A;


        this -> publisher_uav_ -> publish(uav_marker);
        this -> publisher1_ -> publish(msg_pub1);
        this -> publisher2_ -> publish(msg_pub2);
        this -> publisher3_ -> publish(msg_pub3);
        this -> publisher4_ -> publish(msg_pub4);
        this -> publisher5_ -> publish(msg_pub5);
        this -> publisher6_ -> publish(msg_pub6);
        this -> publisher7_ -> publish(msg_pub7);
        this -> publisher8_ -> publish(msg_pub8);

    }

    rclcpp::Subscription<tether_msgs::msg::TetherCompare>::SharedPtr subscription_;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_uav_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher1_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher2_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher3_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher4_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher5_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher6_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher7_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher8_;

};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TensionVisualizationNode>());
    rclcpp::shutdown();


    return 0;
}








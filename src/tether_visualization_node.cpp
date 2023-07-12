

#include <chrono>
#include <functional>
#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <smart_tether/tensions.hpp>
#include <tether_msgs/msg/tether_compare.hpp>

using namespace std::chrono_literals;

#define PLOt_POINTS 100


// #define VECTOR_SCALE_X 1.0
// #define VECTOR_SCALE_Y 3.0
// #define VECTOR_SCALE_Z 3.0
// #define VECTOR_COLOR_R 1.0
// #define VECTOR_COLOR_G 0.0
// #define VECTOR_COLOR_B 0.0
// #define VECTOR_COLOR_A 1.0


class TetherVisualizationNode : public rclcpp::Node
{
public:
    TetherVisualizationNode(): Node("minimal_subscriber")
    {
        subscription_ = this->create_subscription<tether_msgs::msg::TetherCompare>(
                            "optimize_results", 10, std::bind(&TetherVisualizationNode::callback_subscribe_tether, this, std::placeholders::_1)
                        );

        publisher1_ = this->create_publisher<nav_msgs::msg::Path>("path_shape1_1tether", 10);
        publisher2_ = this->create_publisher<nav_msgs::msg::Path>("path_shape1_2tether", 10);
        publisher3_ = this->create_publisher<nav_msgs::msg::Path>("path_shape2_2tether", 10);
        publisher4_ = this->create_publisher<nav_msgs::msg::Path>("path_shape1_3tether", 10);
        publisher5_ = this->create_publisher<nav_msgs::msg::Path>("path_shape2_3tether", 10);
        publisher6_ = this->create_publisher<nav_msgs::msg::Path>("path_shape3_3tether", 10);

    }

private:
    void callback_subscribe_tether(const tether_msgs::msg::TetherCompare &msg_sub)
    {
        // tether1 pattern
        nav_msgs::msg::Path path1;
        path1.header.frame_id = msg_sub.tuav1_1tether.header.frame_id;

        // get catenary num frrm tether_msg, h(uav_position from each winch)
        double h1 = msg_sub.h1;
        double h2 = msg_sub.h2;
        double h3 = msg_sub.h3;
        double v1 = msg_sub.v1;
        double v2 = msg_sub.v2;
        double v3 = msg_sub.v3;
        double c1_1tether = msg_sub.c1_1tether;
        double c1_2tether = msg_sub.c1_2tether;
        double c2_2tether = msg_sub.c2_2tether;
        double c1_3tether = msg_sub.c1_3tether;
        double c2_3tether = msg_sub.c2_3tether;
        double c3_3tether = msg_sub.c3_3tether;

        // calculate x_uav, x_winch (need, h, v, c, val_for_contain_res uav,winth)
        double x_uav1 = 0.0, x_winch1 = 0.0;
        // smart_tether::calc_x_positions(h1, v1, c1_1tether, x_winch1, x_uav1);
        smart_tether::calc_x_positions(h1, 0., c1_1tether, x_winch1, x_uav1);

        // calc h/100 = point interval 
        double interval = h1 / 100.;

        for (int i = 0; i < 100; i++)
        {
            // calc each x using point interval: i * interval
            double x = x_winch1 + i * interval;
            // calc each z using catenary function
            double z = smart_tether::catenary(x, c1_1tether);

            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = msg_sub.tuav1_1tether.header.frame_id;

            pose.pose.position.y = 0.;
            // tralslation for each x and y
            pose.pose.position.x = x - x_uav1;
            pose.pose.position.z = z - smart_tether::catenary(x_uav1, c1_1tether);
            // pose.pose.position.x = x;
            // pose.pose.position.z = z;

            path1.poses.push_back(pose);
        }



        // tether2 pattern




        // tether3 pattern





        this -> publisher1_ -> publish(path1);
        // this -> publisher2_ -> publish(path2);
        // this -> publisher3_ -> publish(path3);
        // this -> publisher4_ -> publish(path4);
        // this -> publisher5_ -> publish(path5);
        // this -> publisher6_ -> publish(path6);

    }

    rclcpp::Subscription<tether_msgs::msg::TetherCompare>::SharedPtr subscription_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher1_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher2_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher3_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher4_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher5_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher6_;

};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TetherVisualizationNode>());
    rclcpp::shutdown();


    return 0;
}






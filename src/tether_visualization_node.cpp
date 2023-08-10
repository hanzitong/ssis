

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

        // tether1 pattern
        nav_msgs::msg::Path path1_1tether;
        path1_1tether.header.frame_id = msg_sub.tuav1_1tether.header.frame_id;
        make_path(h1, v1, c1_1tether, path1_1tether, msg_sub.tuav1_1tether.header.frame_id);
        // double x_uav1 = 0.0, x_winch1 = 0.0;
        // smart_tether::calc_x_positions(h1, 0., c1_1tether, x_winch1, x_uav1);
        // double interval = h1 / 100.;
        // for (int i = 0; i < 100; i++)
        // {
        //     double x = x_winch1 + i * interval;
        //     double z = smart_tether::catenary(x, c1_1tether);
        //     geometry_msgs::msg::PoseStamped pose;
        //     pose.header.frame_id = msg_sub.tuav1_1tether.header.frame_id;
        //     pose.pose.position.y = 0.;
        //     pose.pose.position.x = x - x_uav1;
        //     pose.pose.position.z = z - smart_tether::catenary(x_uav1, c1_1tether);
        //     path1.poses.push_back(pose);
        // }


        // tether2 pattern
        nav_msgs::msg::Path path1_2tether;
        path1_2tether.header.frame_id = msg_sub.tuav1_2tether.header.frame_id;
        make_path(h1, v1, c1_2tether, path1_2tether, msg_sub.tuav1_2tether.header.frame_id);

        nav_msgs::msg::Path path2_2tether;
        path2_2tether.header.frame_id = msg_sub.tuav2_2tether.header.frame_id;
        make_path(h2, v2, c2_2tether, path2_2tether, msg_sub.tuav2_2tether.header.frame_id);


        // tether3 pattern
        nav_msgs::msg::Path path1_3tether;
        path1_3tether.header.frame_id = msg_sub.tuav1_3tether.header.frame_id;
        make_path(h1, v1, c1_3tether, path1_3tether, msg_sub.tuav1_3tether.header.frame_id);

        nav_msgs::msg::Path path2_3tether;
        path2_3tether.header.frame_id = msg_sub.tuav2_3tether.header.frame_id;
        make_path(h2, v2, c2_3tether, path2_3tether, msg_sub.tuav2_3tether.header.frame_id);

        nav_msgs::msg::Path path3_3tether;
        path3_3tether.header.frame_id = msg_sub.tuav3_3tether.header.frame_id;
        make_path(h3, v3, c3_3tether, path3_3tether, msg_sub.tuav3_3tether.header.frame_id);


        this -> publisher1_ -> publish(path1_1tether);
        this -> publisher2_ -> publish(path1_2tether);
        this -> publisher3_ -> publish(path2_2tether);
        this -> publisher4_ -> publish(path1_3tether);
        this -> publisher5_ -> publish(path2_3tether);
        this -> publisher6_ -> publish(path3_3tether);

    }

    void make_path(double h, double v, double catenary_num, nav_msgs::msg::Path &path, std::string frame_id)
    {
        // calculate x_uav, x_winch (need, h, v, c, val_for_contain_res uav,winth)
        double x_uav = 0.0, x_winch = 0.0;
        // smart_tether::calc_x_positions(h1, v1, catenary_num, x_winch1, x_uav1);
        smart_tether::calc_x_positions(h, v, catenary_num, x_winch, x_uav);
        // calc h/100 = point interval 
        double interval = h / 100.;

        for (int i = 0; i < 100; i++)
        {
            // calc each x using point interval: i * interval
            double x = x_winch + i * interval;
            // calc each z using catenary function
            double z = smart_tether::catenary(x, catenary_num);
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = frame_id;
            pose.pose.position.y = 0.;
            pose.pose.position.x = x - x_uav;
            pose.pose.position.z = z - smart_tether::catenary(x_uav, catenary_num);

            path.poses.push_back(pose);
        }

        return;
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






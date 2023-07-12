

#include <chrono>
#include <functional>
#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>

#include <tether_msgs/msg/tether_compare.hpp>

using namespace std::chrono_literals;

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

        publisher1_ = this->create_publisher<nav_msgs::msg::Path>("marker_shape1_1tether", 10);
        publisher2_ = this->create_publisher<nav_msgs::msg::Path>("marker_shape1_2tether", 10);
        publisher3_ = this->create_publisher<nav_msgs::msg::Path>("marker_shape2_2tether", 10);
        publisher4_ = this->create_publisher<nav_msgs::msg::Path>("marker_shape1_3tether", 10);
        publisher5_ = this->create_publisher<nav_msgs::msg::Path>("marker_shape2_3tether", 10);
        publisher6_ = this->create_publisher<nav_msgs::msg::Path>("marker_shape3_3tether", 10);

    }

private:
    void callback_subscribe_tether(const tether_msgs::msg::TetherCompare &msg_sub)
    {
        // tether1 pattern
        nav_msgs::msg::Path path1;


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






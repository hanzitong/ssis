
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"

using namespace std::chrono_literals;

class UavPositionPublisher : public rclcpp::Node{
public:
    UavPositionPublisher() : Node("uav_position_publisher"), x(0.), y(0.), L(50.), interval(10.){
        publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("winch1/uav_pose", 10);
        // timer_ = this->create_wall_timer(500ms, std::bind(&UavPositionPublisher::callback_pub_uav_posision, this));
        timer_ = this->create_wall_timer(1s, std::bind(&UavPositionPublisher::callback_pub_uav_posision, this));
    }
private:
    void callback_pub_uav_posision(){
        auto message = geometry_msgs::msg::Pose();
        message.position.x = x;
        message.position.y = y;
        message.position.z = 0.;
        message.orientation.x = 0.;
        message.orientation.y = 0.;
        message.orientation.z = 0.;
        message.orientation.w = 1.;

        if(x<L && y<=L){
            x += interval;
        }else if(x>=L && y<L){
            x = 0.;
            y += interval;
        }else{
            RCLCPP_INFO(this->get_logger(), "Calculation finished.");
            x = 0.;
            y = 0.;
        }

        // for debag 
        // message.position.x = 25.;
        // message.position.y = 0.;
        // message.position.z = 0.;
        // message.orientation.x = 0.;
        // message.orientation.y = 0.;
        // message.orientation.z = 0.;
        // message.orientation.w = 1.;

        // RCLCPP_INFO(this->get_logger(), "Publishing uav position x:%f, y:%f", message.position.x, message.position.y);
        publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_;
    double x, y, L, interval;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UavPositionPublisher>());
  rclcpp::shutdown();
  return 0;
}


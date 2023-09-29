
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
    UavPositionPublisher() : Node("uav_position_publisher"), x(0.), y(25.), z(-25.), lim_x(50.), lim_y(50.), lim_z(25.), interval(1.){
        publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("winch1/uav_pose", 10);
        // timer_ = this->create_wall_timer(500ms, std::bind(&UavPositionPublisher::callback_pub_uav_posision, this));
        timer_ = this->create_wall_timer(0.6s, std::bind(&UavPositionPublisher::callback_pub_uav_posision, this));
    }
private:
    void callback_pub_uav_posision(){
        auto message = geometry_msgs::msg::Pose();
        message.position.x = x;
        message.position.y = y;
        message.position.z = z;
        message.orientation.x = 0.;
        message.orientation.y = 0.;
        message.orientation.z = 0.;
        message.orientation.w = 1.;


        RCLCPP_INFO(this->get_logger(), "Publishing uav position x:%f, y:%f, z:%f", message.position.x, message.position.y, message.position.z);
        publisher_->publish(message);

        // if(x < lim_x && y <= lim_y){
        //         x += interval;
        //     }else if(x >= lim_x && y < lim_y){
        //         x = 0.;
        //         y += interval;
        //     }else{
        //         RCLCPP_INFO(this->get_logger(), "Round finished.");
        //         x = 0.;
        //         y = 0.;
        //     }
        // }

        // for calculate xz plane. need to flip result-mesh in plot-script
        if(x < lim_x && z <= lim_z){
                x += interval;
            }else if(x >= lim_x && z < lim_z){
                x = 0.;
                z += interval;
            }else{
                RCLCPP_INFO(this->get_logger(), "Calculation finished.");
                x = 0.;
                z = -25.;
            }
        }


    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_;
    double x, y, z, lim_x, lim_y, lim_z, interval;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UavPositionPublisher>());
  rclcpp::shutdown();

  return 0;
}


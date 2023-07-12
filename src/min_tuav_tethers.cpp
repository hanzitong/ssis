
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
// #include <visualization_msgs/msg/marker.hpp>

#include <tether_msgs/msg/tether_compare.hpp>
#include <smart_tether/tether_opt.hpp>
#include <smart_tether/multi_tether_opt.hpp>
#include <smart_tether/c_bounds.hpp>
#include <ssis/ssis_tools.hpp>


using namespace std::chrono_literals;

// #define RHO_VALUE 0.045
#define RHO_VALUE 0.0225
#define S_MARGIN 0.5
#define V_MARGIN 20.



class MinTuav1tether : public rclcpp::Node{
public:
    MinTuav1tether() : Node("min_tuav_1tether")//, c_opt_{0.}, c_upper_{0.}, c_lower_{0.}
    {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        publisher_ = this->create_publisher<tether_msgs::msg::TetherCompare>("optimize_results", 1);
        // tuav1_marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("marker_tuav1", 1);
        timer_ = this->create_wall_timer(0.5s, std::bind(&MinTuav1tether::on_timer, this));
    }

private:
    void on_timer(){
        // RCLCPP_INFO(this->get_logger(),"call on_timer()");
        geometry_msgs::msg::TransformStamped t1;
        geometry_msgs::msg::TransformStamped t2;
        geometry_msgs::msg::TransformStamped t3;
        tether_msgs::msg::TetherCompare msg_pub;

        try{
            t1 = tf_buffer_->lookupTransform("winch1", "uav1", tf2::TimePointZero);
            t2 = tf_buffer_->lookupTransform("winch1", "uav2", tf2::TimePointZero);
            t3 = tf_buffer_->lookupTransform("winch1", "uav3", tf2::TimePointZero);
        }catch(const tf2::TransformException & ex){
            RCLCPP_INFO(this->get_logger(), "Could not transform : %s", ex.what());
            return;
        }

        // tf listener here. get quaternion & uav_position
        t1 = tf_buffer_ -> lookupTransform("winch1", "uav1", tf2::TimePointZero);
        t2 = tf_buffer_ -> lookupTransform("winch1", "uav2", tf2::TimePointZero);
        t3 = tf_buffer_ -> lookupTransform("winch1", "uav3", tf2::TimePointZero);

        // pass quaternion to tf2::Quaternion
        q2to1.setX(t2.transform.rotation.x);
        q2to1.setY(t2.transform.rotation.y);
        q2to1.setZ(t2.transform.rotation.z);
        q2to1.setW(t2.transform.rotation.w);
        q3to1.setX(t3.transform.rotation.x);
        q3to1.setY(t3.transform.rotation.y);
        q3to1.setZ(t3.transform.rotation.z);
        q3to1.setW(t3.transform.rotation.w);

        // set h, v, rho
        double h_1 = std::sqrt(t1.transform.translation.x * t1.transform.translation.x + t1.transform.translation.y * t1.transform.translation.y);
        double h_2 = std::sqrt(t2.transform.translation.x * t2.transform.translation.x + t2.transform.translation.y * t2.transform.translation.y);
        double h_3 = std::sqrt(t3.transform.translation.x * t3.transform.translation.x + t3.transform.translation.y * t3.transform.translation.y);
        std::vector<double> h_12 = {h_1, h_2};
        std::vector<double> h_123 = {h_1, h_2, h_3};
        double v_1 = t1.transform.translation.z;
        double v_2 = t2.transform.translation.z;
        double v_3 = t3.transform.translation.z;
        std::vector<double> v_12= {v_1, v_2};
        std::vector<double> v_123 = {v_1, v_2, v_3};
        std::vector<double> rho_12 = {RHO_VALUE, RHO_VALUE};
        std::vector<double> rho_123 = {RHO_VALUE, RHO_VALUE, RHO_VALUE};

        // calc c_upper
        double c_upper_1 = smart_tether::calc_c_upper(h_1, v_1, S_MARGIN);
        double c_upper_2 = smart_tether::calc_c_upper(h_2, v_2, S_MARGIN);
        double c_upper_3 = smart_tether::calc_c_upper(h_3, v_3, S_MARGIN);
        std::vector<double> c_upper_12{c_upper_1, c_upper_2};
        std::vector<double> c_upper_123{c_upper_1, c_upper_2, c_upper_3};

        // calc c_lower
        double c_lower_1 = smart_tether::calc_c_lower(h_1, v_1, V_MARGIN);
        double c_lower_2 = smart_tether::calc_c_lower(h_2, v_2, V_MARGIN);
        double c_lower_3 = smart_tether::calc_c_lower(h_3, v_3, V_MARGIN);
        std::vector<double> c_lower_12 = {c_lower_1, c_lower_2};
        std::vector<double> c_lower_123 = {c_lower_1, c_lower_2, c_lower_3};


        // tf2::Vector3 decralations for plotting 
        tf2::Vector3 tuav1_1tether = {0., 0., 0.};     // 1
        tf2::Vector3 tuav1_2tethers = {0., 0., 0.};     // 2
        tf2::Vector3 tuav2_2tethers = {0., 0., 0.};     // 2
        tf2::Vector3 tuav1_3tethers = {0., 0., 0.};     // 3
        tf2::Vector3 tuav2_3tethers = {0., 0., 0.};     // 3
        tf2::Vector3 tuav3_3tethers = {0., 0., 0.};     // 3
        tf2::Vector3 tuav_sum_2tethers = {0., 0., 0.};     // 2 sum
        tf2::Vector3 tuav_sum_3tethers = {0., 0., 0.};     // 3 sum

        // calc tether tension
        msg_pub.tuav1_sum_norm = smart_tether::optimize_1tether(h_1, v_1, 2. * RHO_VALUE, c_opt1_, tuav1_1tether);
        msg_pub.tuav12_sum_norm = smart_tether::optimize_2tethers(h_12, v_12, rho_12, c_opt12_, q2to1, tuav1_2tethers, tuav2_2tethers, tuav_sum_2tethers);
        msg_pub.tuav123_sum_norm = smart_tether::optimize_3tethers(h_123, v_123, rho_123, c_opt123_, q2to1, q3to1, tuav1_3tethers, tuav2_3tethers, tuav3_3tethers, tuav_sum_3tethers);

        // compare and put result to msg
        // std::vector<double> compare_tuav12 = {msg_pub.tuav1_sum_norm, msg_pub.tuav12_sum_norm};
        // msg_pub.result = compare_func::select_min(compare_tuav12);
        std::vector<double> compare_tuav123 = {msg_pub.tuav1_sum_norm, msg_pub.tuav12_sum_norm, msg_pub.tuav123_sum_norm};
        msg_pub.result = compare_func::select_min(compare_tuav123);


        // put each data to msg(tether_msgs/msg/TetherCompare.msg)
        msg_pub.uav_pos.x = t1.transform.translation.x;   // get from listener
        msg_pub.uav_pos.y = t1.transform.translation.y;   // get from listener
        msg_pub.uav_pos.z = t1.transform.translation.z;   // get from listener
        for(int i=0; i<3; i++){
            if(i==0){
                msg_pub.tuav1_1tether.vector.x = tuav1_1tether[i];
                msg_pub.tuav1_2tether.vector.x = tuav1_2tethers[i];
                msg_pub.tuav2_2tether.vector.x = tuav2_2tethers[i];
                msg_pub.tuav1_3tether.vector.x = tuav1_3tethers[i];
                msg_pub.tuav2_3tether.vector.x = tuav2_3tethers[i];
                msg_pub.tuav3_3tether.vector.x = tuav3_3tethers[i];
                msg_pub.tuav12_sum_vector.vector.x = tuav_sum_2tethers[i];
                msg_pub.tuav123_sum_vector.vector.x = tuav_sum_3tethers[i];

            }else if(i==1){                                   
                msg_pub.tuav1_1tether.vector.y = tuav1_1tether[i];
                msg_pub.tuav1_2tether.vector.y = tuav1_2tethers[i];
                msg_pub.tuav2_2tether.vector.y = tuav2_2tethers[i];
                msg_pub.tuav1_3tether.vector.y = tuav1_3tethers[i];
                msg_pub.tuav2_3tether.vector.y = tuav2_3tethers[i];
                msg_pub.tuav3_3tether.vector.y = tuav3_3tethers[i];
                msg_pub.tuav12_sum_vector.vector.y = tuav_sum_2tethers[i];
                msg_pub.tuav123_sum_vector.vector.y = tuav_sum_3tethers[i];
            
            }else if(i==2){                                   
                msg_pub.tuav1_1tether.vector.z = tuav1_1tether[i];
                msg_pub.tuav1_2tether.vector.z = tuav1_2tethers[i];
                msg_pub.tuav2_2tether.vector.z = tuav2_2tethers[i];
                msg_pub.tuav1_3tether.vector.z = tuav1_3tethers[i];
                msg_pub.tuav2_3tether.vector.z = tuav2_3tethers[i];
                msg_pub.tuav3_3tether.vector.z = tuav3_3tethers[i];
                msg_pub.tuav12_sum_vector.vector.z = tuav_sum_2tethers[i];
                msg_pub.tuav123_sum_vector.vector.z = tuav_sum_3tethers[i];
            
            }else{
                RCLCPP_INFO(this->get_logger(),"error");
            }
        }

        msg_pub.tuav1_1tether.header.frame_id = "uav1";
        msg_pub.tuav1_2tether.header.frame_id = "uav1";
        msg_pub.tuav2_2tether.header.frame_id = "uav2";
        msg_pub.tuav1_3tether.header.frame_id = "uav1";
        msg_pub.tuav2_3tether.header.frame_id = "uav2";
        msg_pub.tuav3_3tether.header.frame_id = "uav3";


        RCLCPP_INFO(this->get_logger(),"q2to1-> x:%f, y:%f, z:%f, w:%f", q2to1.x(), q2to1.y(), q2to1.z(), q2to1.w());
        publisher_->publish(msg_pub);

        return;
    }

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
    rclcpp::Publisher<tether_msgs::msg::TetherCompare>::SharedPtr publisher_{nullptr};
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



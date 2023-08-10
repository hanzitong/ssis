
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <tether_msgs/msg/tether_compare.hpp>
#include <fstream>

using std::placeholders::_1;


void writeArrayToFile(double array[51][51], const std::string& filename)
{
    std::ofstream file(filename);

    if(file.is_open()){
        for(int i = 0; i < 51; i++){
            for (int j = 0; j < 51; j++) {
                file << array[i][j] << " "; // array[y][x]
            }
            file << "\n";
        }
        file.close();
    }else{
        std::cerr << "Unable to open file" << std::endl;
    }

}


class GetResultNode : public rclcpp::Node
{
public:
    GetResultNode(): Node("get_result_node")
    {
      // subscription_ = this -> create_subscription<tether_msgs::msg::TetherCompare>("result_compare", 10, std::bind(&GetResultNode::topic_callback, this, _1));
      subscription_ = this -> create_subscription<tether_msgs::msg::TetherCompare>("optimize_results", 10, std::bind(&GetResultNode::topic_callback, this, _1));

      // initialize result-array
      for(int i; i<51; i++){
        for(int j; j<51; j++){
          result_mesh[i][j] = -1.;
        }
      }
    }

private:
    void topic_callback(const tether_msgs::msg::TetherCompare &msg)
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%f' '%f'", msg.uav_pos.x, msg.uav_pos.y);
        int pos_x = static_cast<int>(msg.uav_pos.x);
        int pos_y = static_cast<int>(msg.uav_pos.y);
        result_mesh[pos_y][pos_x] = msg.result;

        if(pos_x == 50 && pos_y == 50){
            writeArrayToFile(result_mesh, "output.txt");
            RCLCPP_INFO(this->get_logger(), "made output.txt");
        }
    }

    double result_mesh[51][51];
    rclcpp::Subscription<tether_msgs::msg::TetherCompare>::SharedPtr subscription_;

};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GetResultNode>());
  rclcpp::shutdown();

  return 0;
}


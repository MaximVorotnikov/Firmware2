#include <chrono>
#include <memory>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

using std::placeholders::_1;
using namespace std;
rosgraph_msgs::msg::Clock curr_time;

int flag = 0;
double start = 0.0;

class GzTimeNode : public rclcpp::Node
{
public:
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr sub_gz_time;

  GzTimeNode()
  : Node("gz_time_node")
  {
  sub_gz_time = this->create_subscription<rosgraph_msgs::msg::Clock>("/clock", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(), std::bind(&GzTimeNode::gz_time_cb, this, std::placeholders::_1));
  }

private:

  void gz_time_cb(const rosgraph_msgs::msg::Clock::SharedPtr time)
  {
      curr_time = *time;

      flag++;
      if (flag == 1)
      {
        start = curr_time.clock.sec + curr_time.clock.nanosec / 1e9;
      }

      double time_since_start = curr_time.clock.sec + curr_time.clock.nanosec / 1e9; 
      RCLCPP_INFO_STREAM(this->get_logger(), "Gz Time = " << time_since_start);
      RCLCPP_INFO_STREAM(this->get_logger(), "Minutes = " << floor(time_since_start / 60));
      RCLCPP_INFO_STREAM(this->get_logger(), "Seconds = " << time_since_start - floor(time_since_start / 60) * 60);
      RCLCPP_INFO_STREAM(this->get_logger(), "Start gz time = " << start);
      RCLCPP_INFO_STREAM(this->get_logger(), "Finish gz time = " << time_since_start - start);
      RCLCPP_INFO_STREAM(this->get_logger(), "Finish minutes = " << floor((time_since_start - start) / 60));
      RCLCPP_INFO_STREAM(this->get_logger(), "Finish seconds = " << (time_since_start - start) - floor((time_since_start - start) / 60) * 60);

  }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GzTimeNode>());
  rclcpp::shutdown();
  return 0;
}
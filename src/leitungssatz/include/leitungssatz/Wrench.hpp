#pragma once

#include "Main.hpp"

#include <geometry_msgs/msg/wrench_stamped.hpp>


namespace leitungssatz  {
    
class Wrench
{
private:
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;

public:
  geometry_msgs::msg::WrenchStamped wrenchMsg;

  
  Wrench();
  Wrench(std::shared_ptr<rclcpp::Node> node);
  void wrench_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
};

} // namespace leitungssatz
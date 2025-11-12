#include <leitungssatz/Wrench.hpp>

using namespace leitungssatz;

Wrench::Wrench()
{

}

Wrench::Wrench(std::shared_ptr<rclcpp::Node> node)
{
    wrenchMsg = geometry_msgs::msg::WrenchStamped();
    wrench_sub_ = node->create_subscription<geometry_msgs::msg::WrenchStamped>(
        "/ur_hardware_interface/wrench", 
        10, 
        std::bind(&Wrench::wrench_callback, 
        this, 
        std::placeholders::_1));
}


 void Wrench::wrench_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
    wrenchMsg = *msg;
}

// This source file and its header file are manually checked for
// consistency with the ROS1 version.
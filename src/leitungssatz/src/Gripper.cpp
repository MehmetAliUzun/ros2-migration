#include "leitungssatz/Gripper.hpp"

using namespace leitungssatz;


Gripper::Gripper(std::shared_ptr<rclcpp::Node> node): node_(node){

    
    RCLCPP_INFO(node_->get_logger(), "Gripper initialized");

    gripper_move_ = node_->create_client<leitungssatz_interfaces::srv::SetGripper>
    ("/ur_hardware_interface/robotiq/set_gripper");//Service client, sends requests
    
    gripper_info_sub_ = node_->create_subscription<leitungssatz_interfaces::msg::GripperInfo>
    ("/ur_hardware_interface/robotiq/gripper_infos", 
    10,
    std::bind(&Gripper::gripper_info_callback, 
    this, 
    std::placeholders::_1));
    
}



void Gripper::gripper_info_callback(const leitungssatz_interfaces::msg::GripperInfo::SharedPtr msg)
{
    gripper_info_ = *msg;
    position = gripper_info_.position_per;
    // RCLCPP_INFO(node_->get_logger(), "Gripper position: %f", position);
}
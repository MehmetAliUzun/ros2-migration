#pragma once

#include "Main.hpp"
#include <rclcpp/rclcpp.hpp>
#include "leitungssatz_interfaces/msg/gripper_info.hpp"
#include "leitungssatz_interfaces/srv/set_gripper.hpp"



namespace leitungssatz
{
    class Gripper
    {
    private:
        rclcpp::Subscription<leitungssatz_interfaces::msg::GripperInfo>::SharedPtr gripper_info_sub_;
        leitungssatz_interfaces::msg::GripperInfo gripper_info_;
        
        std::shared_ptr<rclcpp::Node> node_;
        
    public:
        Gripper(std::shared_ptr<rclcpp::Node> node);
        void gripper_info_callback(const leitungssatz_interfaces::msg::GripperInfo::SharedPtr msg);
        rclcpp::Client<leitungssatz_interfaces::srv::SetGripper>::SharedPtr gripper_move_;
        double position = 0.0;
    };
}

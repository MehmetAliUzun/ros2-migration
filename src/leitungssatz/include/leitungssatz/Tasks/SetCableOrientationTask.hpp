#pragma once

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "leitungssatz/srv/set_cable_orientation.hpp"  
#include "leitungssatz/Task.hpp"


namespace leitungssatz
{
    
class SetCableOrientationTask : public Task {

public:

    SetCableOrientationTask(Robot_ptr robot, std::shared_ptr<rclcpp::Node> node, CableType cableType, const std::string& serviceName);
    TaskStatus execute() override;

private:
   
    rclcpp::Client<leitungssatz::srv::SetCableOrientation>::SharedPtr setCable_;
    rclcpp::Client<leitungssatz::srv::SetCableOrientation>::SharedPtr setCable2_;
    CableType cableType_;
    std::shared_ptr<rclcpp::Node> node_;

};

}
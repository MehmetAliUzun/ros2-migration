#pragma once

#include "leitungssatz/Task.hpp"
#include "leitungssatz/Wrench.hpp"
#include "leitungssatz/Tasks/MoveTask.hpp"
#include "leitungssatz/Gripper.hpp"
#include "leitungssatz/Tasks/GripperTask.hpp"
#include "leitungssatz/Tasks/ForceTask.hpp"
#include <leitungssatz_interfaces/srv/set_force_target.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>

namespace leitungssatz {

class WirePush : public Task
{
private:
    double loop = 3;
    Robot_ptr rob;
    double vel_multi_ = 1;
    leitungssatz_interfaces::srv::SetForceTarget::Request _service_msg;
    rclcpp::Client<leitungssatz_interfaces::srv::SetForceTarget>::SharedPtr force_move_;
    Wrench_ptr wrench_;
    std::string close_point, far_point;
    double max_f;
    double check;
    int reinsertion = 0;
    std::shared_ptr<rclcpp::Node> node_;

public:
    
    leitungssatz_interfaces::srv::SetForceTarget::Request force_target(
        bool IO, array6d free_axis = {1, 1, 1, 0, 0, 0},
        array6d wrench = {-25, 0, 0, 0, 0, 0}, double vel = 3);
    
    bool call_force_target(const leitungssatz_interfaces::srv::SetForceTarget::Request& srv);    
        
    WirePush(Robot_ptr robot, Wrench_ptr wrench, std::shared_ptr<rclcpp::Node> node)
        : Task(robot), rob(robot), wrench_(wrench), node_(node)
    {
        force_move_ = node_->create_client<leitungssatz_interfaces::srv::SetForceTarget>("/ur_hardware_interface/set_force_mode");
    }

    
    WirePush(Robot_ptr robot, Wrench_ptr wrench, std::shared_ptr<rclcpp::Node> node,
             std::string point_name, std::string point2, int no, double maximum_force, int checkforce);

    
    bool checkInsertion();
    TaskStatus execute() override;
};

} // namespace leitungssatz
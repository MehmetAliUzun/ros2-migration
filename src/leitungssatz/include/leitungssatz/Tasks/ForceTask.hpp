#pragma once

#include "leitungssatz/Task.hpp"
#include "leitungssatz/Wrench.hpp"
#include <rclcpp/rclcpp.hpp>
#include "leitungssatz_interfaces/srv/set_force_target.hpp"
#include <array>

namespace leitungssatz {

    // using array6d = std::array<double, 6>;
    // array6d type defined in Main.hpp alongside with array6i
    
    class ForceTask : public Task
    {
    private:
        double vel_multi_ = 1.0;
        leitungssatz_interfaces::srv::SetForceTarget::Request _service_msg;
        rclcpp::Client<leitungssatz_interfaces::srv::SetForceTarget>::SharedPtr force_move_;
        Wrench_ptr wrench_;
        std::shared_ptr<rclcpp::Node> node_;

    public:
        ForceTask(Robot_ptr robot, std::shared_ptr<rclcpp::Node> node, bool IO, array6d free_axis, array6d wrench, double vel, Wrench_ptr wrenchObj);
        leitungssatz_interfaces::srv::SetForceTarget::Request force_target(bool IO, array6d free_axis = {0, 0, 1, 0, 0, 0}, array6d wrench = {0, 0, -10.0, 0, 0, 0}, double vel = 3.0);
        bool call_force_target();
        TaskStatus execute() override;
    }; 


} // namespace leitungssatz
#pragma once

#include "leitungssatz/Task.hpp"
#include "leitungssatz/Wrench.hpp"
#include <leitungssatz_interfaces/srv/set_force_target.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <cmath>

namespace leitungssatz {

class SpiralTask : public Task
{
private:
    double vel_multi_ = 1.0;
    leitungssatz_interfaces::srv::SetForceTarget::Request _service_msg;
    rclcpp::Client<leitungssatz_interfaces::srv::SetForceTarget>::SharedPtr force_move_;
    Wrench_ptr wrench_;

    double start_;
    double stop_;
    int step_c_;
    double step_f_;
    double x_limit_;
    double x_force_;
    bool dir_;
    std::shared_ptr<rclcpp::Node> node_;

public:
    SpiralTask(Robot_ptr robot, std::shared_ptr<rclcpp::Node> node, Wrench_ptr wrench)
        : Task(robot), wrench_(wrench), node_(node)
    {
        force_move_ = node_->create_client<leitungssatz_interfaces::srv::SetForceTarget>("/ur_hardware_interface/set_force_mode");
    }

    SpiralTask(Robot_ptr robot, std::shared_ptr<rclcpp::Node> node, Wrench_ptr wrenchObj,
               double start, double stop, int step_c, double step_f, double x_limit, double x_force, bool dir);

    ~SpiralTask() { /*override removed*/
        finish();
    }

    leitungssatz_interfaces::srv::SetForceTarget::Request force_target(
        bool IO, array6d free_axis = {0, 0, 1, 0, 0, 0},
        array6d wrench = {0, 0, -10.0, 0, 0, 0}, double vel = 3.0);

    bool call_force_target(leitungssatz_interfaces::srv::SetForceTarget::Request& srv);
    bool spiral_force(double start, double stop, int step_c, double step_f, double x_limit, double x_force, bool dir);
    TaskStatus execute() override;
    bool finish();
};

} // namespace leitungssatz
#pragma once

#include "leitungssatz/Task.hpp"
#include <leitungssatz_interfaces/srv/grab_buchse.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>

namespace leitungssatz {

class GrabBuchseTask : public Task
{
private:
    rclcpp::Client<leitungssatz_interfaces::srv::GrabBuchse>::SharedPtr grab_buchse_client_;
    std::string service_name_;

public:
    GrabBuchseTask(Robot_ptr robot, std::shared_ptr<rclcpp::Node> node, std::string& service_name);
    TaskStatus execute() override;
    virtual ~GrabBuchseTask();
};

} // namespace leitungssatz
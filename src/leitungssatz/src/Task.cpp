#include "leitungssatz/Task.hpp"
#include <rclcpp/rclcpp.hpp>

using namespace leitungssatz;

TaskStatus Task::execute() {
    RCLCPP_ERROR(rclcpp::get_logger("Task"), "DONT USE THIS CLASS");
    return TaskStatus::FAILED;
}

bool Task::init() {
    //RCLCPP_ERROR(rclcpp::get_logger("Task"), "DONT USE THIS CLASS");
    return false;
}

Task::~Task() = default; 


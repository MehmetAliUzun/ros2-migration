#include "leitungssatz/Tasks/GrabBuchseTask.hpp"

using namespace leitungssatz;

GrabBuchseTask::GrabBuchseTask(Robot_ptr robot, std::shared_ptr<rclcpp::Node> node, std::string& service_name)
: Task(robot), service_name_(service_name)
{
    grab_buchse_client_ = node->create_client<leitungssatz_interfaces::srv::GrabBuchse>(service_name_);
    RCLCPP_INFO(node->get_logger(), "Waiting for service %s ...", service_name_.c_str());
    grab_buchse_client_->wait_for_service();
}

TaskStatus GrabBuchseTask::execute()
{
    RCLCPP_INFO(rclcpp::get_logger("GrabBuchseTask"), "Executing GrabBuchseTask");
    _task_status = TaskStatus::RUNNING;

    auto request = std::make_shared<leitungssatz_interfaces::srv::GrabBuchse::Request>();
    request->start = true;

    auto future = grab_buchse_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(_robot->get_node(), future) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("GrabBuchseTask"), "Failed to call service");
        return TaskStatus::FAILED;
    }

    auto response = future.get();
    if (response->success)
        _task_status = TaskStatus::FINISHED;
    else
        _task_status = TaskStatus::FAILED;
    return _task_status;
}

GrabBuchseTask::~GrabBuchseTask() = default;
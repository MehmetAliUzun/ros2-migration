#include "leitungssatz/Tasks/SetCableOrientationTask.hpp"

using namespace leitungssatz;

SetCableOrientationTask::SetCableOrientationTask(Robot_ptr robot, std::shared_ptr<rclcpp::Node> node, CableType cableType, const std::string& serviceName)
    : Task(robot), cableType_(cableType), node_(node)
{
    setCable_ = node_->create_client<leitungssatz::srv::SetCableOrientation>(serviceName);

    if (serviceName == "/CableOrienter2/SetCableOrientation")
    {
        setCable2_ = node_->create_client<leitungssatz::srv::SetCableOrientation>(serviceName);
    }

    RCLCPP_INFO(node_->get_logger(), "Waiting for service %s ...", serviceName.c_str());
    setCable_->wait_for_service();
    if (setCable2_)
        setCable2_->wait_for_service();
    
        /* In the original ros1 file:
    
                setCable.waitForExistence();
                setCable2.waitForExistence();
    
        Here first checking if the service exists before waiting for it.    */
}

TaskStatus SetCableOrientationTask::execute()
{
    RCLCPP_INFO(node_->get_logger(), "Executing cableTask for cable %i", static_cast<int>(cableType_));

    _task_status = TaskStatus::RUNNING;

    auto request = std::make_shared<leitungssatz::srv::SetCableOrientation::Request>();
    request->cable_number = static_cast<int>(cableType_);
    request->learning = false;

    bool success = false;

    // Try first service
    if (setCable_)
    {
        auto future = setCable_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            if (future.get()->success)
                success = true;
        }
    }

    // Try second service if first failed and it exists
    if (!success && setCable2_)
    {
        auto future2 = setCable2_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node_, future2) == rclcpp::FutureReturnCode::SUCCESS)
        {
            if (future2.get()->success)
                success = true;
        }
    }

    if (success)
    {
        RCLCPP_INFO(node_->get_logger(), "Success");
        _task_status = TaskStatus::FINISHED;
    }
    else
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call service");
        _task_status = TaskStatus::FAILED;
    }

    return _task_status;
}
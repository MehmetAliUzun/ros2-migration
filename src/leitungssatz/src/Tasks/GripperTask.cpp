#include <leitungssatz/Tasks/GripperTask.hpp>

using namespace leitungssatz;

bool GripperTask::call_gripper(leitungssatz_interfaces::srv::SetGripper::Request &request, leitungssatz_interfaces::srv::SetGripper::Response &response)
{
    
    auto client = _robot->_gripper.gripper_move_;
    if (!client->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(rclcpp::get_logger("gripper_task"), "Gripper service not available");
        return false;
    }

    auto future = client->async_send_request(std::make_shared<leitungssatz_interfaces::srv::SetGripper::Request>(request));
    if (rclcpp::spin_until_future_complete(_robot->get_node(), future) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("gripper_task"), "Failed to call gripper service");
        return false;
    }
    response = *(future.get());
    if (!response.success) {
        RCLCPP_ERROR(rclcpp::get_logger("gripper_task"), "Gripper service call failed");
        return false;
    }
    return true;
}/* This call_gripper function definition looks complicated than intended. 
    I will check it during runtime and adjust if needed.
    For now funtionality could be different than the original ros1 implementation*/

bool GripperTask::move_gripper()
{
    leitungssatz_interfaces::srv::SetGripper::Request request;
    leitungssatz_interfaces::srv::SetGripper::Response response;

    double target_position = ((double)100 - _target_position) / 100 * 255;
    RCLCPP_INFO(rclcpp::get_logger("gripper_task"), "Executing GripperTask; Gripping in position %.2f with speed %.2f and force %.2f async: %d",
                target_position, _speed, _force, _asynchronous);

    request.position = target_position;
    request.position_unit = 2; // 0 --> unit = percent
    request.force = _force;
    request.speed = _speed;
    request.asynchronous = _asynchronous;

    // Call the gripper service using the robot's gripper client
    call_gripper(request, response);

    RCLCPP_INFO(rclcpp::get_logger("gripper_task"), "Gripper Response status: %ld", response.e_object_status);

    bool ret = false;
    if (this->_asynchronous)
        ret = true;
    else
        ret = response.success;
    return ret;
}

bool GripperTask::init()
{
    // Initialize the gripper task, if needed
    //RCLCPP_INFO(rclcpp::get_logger("gripper_task"), "Initializing GripperTask");
    return true; // Return true to indicate successful initialization
}

TaskStatus GripperTask::execute()
{
    _task_status = TaskStatus::RUNNING;
    bool ret = move_gripper();
    if (ret)
        _task_status = TaskStatus::FINISHED;
    else
        _task_status = TaskStatus::FAILED;
    return _task_status;
}

GripperTask::~GripperTask() = default;
#include "leitungssatz/Tasks/ForceTask.hpp"

using namespace leitungssatz;

ForceTask::ForceTask(Robot_ptr robot, std::shared_ptr<rclcpp::Node> node, bool IO, array6d free_axis, array6d wrench, double vel, Wrench_ptr wrenchObj)
    : Task(robot), wrench_(wrenchObj)
{
    vel_multi_ = 1.0;
    force_move_ = node->create_client<leitungssatz_interfaces::srv::SetForceTarget>("/ur_hardware_interface/set_force_mode");
    _service_msg = force_target(IO, free_axis, wrench, vel);
}

leitungssatz_interfaces::srv::SetForceTarget::Request ForceTask::force_target(bool IO, array6d free_axis, array6d wrench, double vel)
{
    vel = vel * vel_multi_;
    leitungssatz_interfaces::srv::SetForceTarget::Request srv;
    srv.io = IO;
    srv.wrench = wrench;
    srv.limits = {vel, vel, vel, vel, vel, vel};
    
    // srv.selection_vector = free_axis;
    // Above line gives an error, because types do not match.
    // Here is a workaround to set the selection vector.
    // Typecast the values inside array6d to int64_t
    // and assign them to the selection_vector.
    // This is necessary because the selection_vector expects int64_t values.
    // Could there be any value loss here? -> Inmportant!

    for (size_t i = 0; i < free_axis.size(); ++i) 
    {
    srv.selection_vector[i] = static_cast<int64_t>(free_axis[i]);
    }

    srv.type = 2;
    return srv;
}

bool ForceTask::call_force_target()
{
    auto request = std::make_shared<leitungssatz_interfaces::srv::SetForceTarget::Request>(_service_msg);
    auto result = force_move_->async_send_request(request);

    // Wait for the service response
    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("force_task"), "Failed to call service /ur_hardware_interface/set_force_mode");
        return false;
    }

    if (!result.get()->success)
    {
        RCLCPP_ERROR(rclcpp::get_logger("force_task"), "Service call failed");
        return false;
    }

    return true;
}

TaskStatus ForceTask::execute()
{
    _task_status = TaskStatus::RUNNING;
    RCLCPP_INFO(rclcpp::get_logger("force_task"), "Executing ForceTask");

    bool ret = call_force_target();
    float feedback = wrench_->wrenchMsg.wrench.force.x;

    rclcpp::Rate rate(2); // 0.5 seconds
    while (wrench_->wrenchMsg.wrench.force.x > -30 && rclcpp::ok() && wrench_->wrenchMsg.wrench.force.x < feedback)
    {
        feedback = wrench_->wrenchMsg.wrench.force.x;
        rate.sleep();

        _service_msg.wrench = {20, 0, 0, 0, 0, 0};
        call_force_target();

        RCLCPP_INFO(rclcpp::get_logger("force_task"), "Wrench force is x: %f y: %f z: %f",
                    wrench_->wrenchMsg.wrench.force.x, wrench_->wrenchMsg.wrench.force.y, wrench_->wrenchMsg.wrench.force.z);
    }

    _service_msg.io = false;
    call_force_target();

    RCLCPP_INFO(rclcpp::get_logger("force_task"), "Finished ForceTask");

    _task_status = ret ? TaskStatus::FINISHED : TaskStatus::FAILED;
    return _task_status;
}
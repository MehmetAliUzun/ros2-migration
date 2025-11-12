#include <leitungssatz/Tasks/WirePushTask.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace leitungssatz;

WirePush::WirePush(Robot_ptr robot, Wrench_ptr wrench, std::shared_ptr<rclcpp::Node> node,
                   std::string close_point, std::string far_point, int loop_iterations, double maximum_force, int checkforce)
    : WirePush(robot, wrench, node)
{
    this->loop = loop_iterations;
    this->close_point = close_point;
    this->far_point = far_point;
    this->rob = robot;
    this->max_f = maximum_force;
    this->check = checkforce;
}

leitungssatz_interfaces::srv::SetForceTarget::Request WirePush::force_target(
    bool IO, array6d free_axis, array6d wrench, double vel)
{
    vel = vel * vel_multi_;
    leitungssatz_interfaces::srv::SetForceTarget::Request srv;
    srv.io = IO;
    srv.wrench = wrench;
    srv.limits = {vel, vel, vel, vel, vel, vel};
    // Same as in ForceTask.cpp
    for (size_t i = 0; i < free_axis.size(); ++i)
        srv.selection_vector[i] = static_cast<int64_t>(free_axis[i]);
    srv.type = 2;
    return srv;
}

bool WirePush::call_force_target(const leitungssatz_interfaces::srv::SetForceTarget::Request& srv)
{
    auto request = std::make_shared<leitungssatz_interfaces::srv::SetForceTarget::Request>(srv);
    auto result = force_move_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call service /ur_hardware_interface/force_mode");
        return false;
    }
    if (!result.get()->success)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to move to force_target");
        return false;
    }
    return true;
}

bool WirePush::checkInsertion()
{
    bool inserted = false;
    _service_msg = force_target(true, {1, 1, 1, 0, 0, 0}, {-25, 0, 0, 0, 0, 0});
    int i = 0;
    while (_robot->tcp_position_.transform.translation.x >= -0.5 && i < 4)
    {
        i++;
        RCLCPP_INFO(node_->get_logger(), "Wrench force in x = %f, y = %f, z= %f",
                    wrench_->wrenchMsg.wrench.force.x, wrench_->wrenchMsg.wrench.force.y, wrench_->wrenchMsg.wrench.force.z);
        bool ret = call_force_target(_service_msg);
        (void)ret; // Not required, but good to avoid unused variable warning
        rclcpp::sleep_for(std::chrono::milliseconds(100));
        if (wrench_->wrenchMsg.wrench.force.x > check)
        {
            RCLCPP_INFO(node_->get_logger(), "FORCE REACHED");
            inserted = true;
            i = 4;
        }
    }
    RCLCPP_INFO(node_->get_logger(), "checking insertion");
    _service_msg.io = false;
    bool re = call_force_target(_service_msg);
    (void)re;
    return inserted;
}

TaskStatus WirePush::execute()
{
    leitungssatz::Task_ptr grip_opens(new leitungssatz::GripperTask(rob, 10, false));
    leitungssatz::Task_ptr grip_close(new leitungssatz::GripperTask(rob, 0, 0, 50, false));
    leitungssatz::Task_ptr moveFront(new leitungssatz::MoveTask(rob, close_point, leitungssatz::MOVEL));
    leitungssatz::Task_ptr moveBack(new leitungssatz::MoveTask(rob, far_point, leitungssatz::MOVEL));
    RCLCPP_INFO(node_->get_logger(), "Wire Insertion");
    _task_status = TaskStatus::RUNNING;
    for (int i = 0; i < loop && wrench_->wrenchMsg.wrench.force.x < max_f; i++)
    {
        RCLCPP_INFO(node_->get_logger(), "wrench force is %f", wrench_->wrenchMsg.wrench.force.x);
        grip_opens->execute();
        moveBack->execute();
        grip_close->execute();
        moveFront->execute();
    }

    while (!checkInsertion() && this->reinsertion < 1)
    {
        this->reinsertion++;
        RCLCPP_INFO(node_->get_logger(), "Reinserting due to failure");
        RCLCPP_INFO(node_->get_logger(), "Try number %d", this->reinsertion);
        loop = 3;
        execute();
    }

    return _task_status;
}
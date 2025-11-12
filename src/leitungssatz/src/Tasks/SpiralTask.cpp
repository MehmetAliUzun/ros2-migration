#include "leitungssatz/Tasks/SpiralTask.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>

using namespace leitungssatz;

SpiralTask::SpiralTask(Robot_ptr robot, std::shared_ptr<rclcpp::Node> node, Wrench_ptr wrenchObj,
                       double start, double stop, int step_c, double step_f, double x_limit, double x_force, bool dir)
    : SpiralTask(robot, node, wrenchObj)
{
    start_ = start;
    stop_ = stop;
    step_c_ = step_c;
    step_f_ = step_f;
    x_limit_ = x_limit;
    dir_ = dir;
    x_force_ = x_force;
}

leitungssatz_interfaces::srv::SetForceTarget::Request SpiralTask::force_target(
    bool IO, array6d free_axis, array6d wrench, double vel)
{
    vel = vel * vel_multi_;
    leitungssatz_interfaces::srv::SetForceTarget::Request srv;
    srv.io = IO;
    srv.wrench = wrench;
    srv.limits = {vel, vel, vel, vel, vel, vel};
    for (size_t i = 0; i < free_axis.size(); ++i)
        srv.selection_vector[i] = static_cast<int64_t>(free_axis[i]);
    srv.type = 2;
    return srv;
}

bool SpiralTask::call_force_target(leitungssatz_interfaces::srv::SetForceTarget::Request& srv)
{
    auto request = std::make_shared<leitungssatz_interfaces::srv::SetForceTarget::Request>(srv);
    auto result = force_move_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call service /ur_hardware_interface/set_force_mode");
        return false;
    }
    if (!result.get()->success)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to move to force_target");
        return false;
    }
    return true;
}

bool SpiralTask::spiral_force(double start, double stop, int step_c, double step_f, double x_limit, double x_force, bool dir)
{
    double radius = start;
    double inc = (2 * M_PI) / step_c;
    double inc_f = (stop - start) / (step_f * step_c);

    int counter = 0;
    double y, z;
    auto force_srv = force_target(true, {1, 1, 1, 0, 0, 0}, {0, 0, 0, 0, 0, 0});

    while ((radius <= stop && _robot->tcp_position_.transform.translation.x <= x_limit))
    {
        if (dir)
        {
            y = radius * sin(inc * counter);
            z = radius * cos(inc * counter);
        }
        else
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000, "Spiral 2"); //Check this line for correctness, compare with the old one
            y = radius * sin(-inc * counter);
            z = radius * cos(-inc * counter);
        }

        force_srv.wrench = {x_force, y, z, 0, 0, 0};
        bool ret = call_force_target(force_srv);
        (void)ret;
        counter += 1;
        radius += inc_f;
        rclcpp::sleep_for(std::chrono::milliseconds(50));
    }

    force_srv.io = false;
    call_force_target(force_srv);

    if (_robot->tcp_position_.transform.translation.z >= x_limit)
        return false;
    else
        return true;
}

TaskStatus SpiralTask::execute()
{
    RCLCPP_INFO(node_->get_logger(), "Executing SpiralTask");
    _task_status = TaskStatus::RUNNING;
    // bool ret = false; -> Is this line really needed? It seems unused in the original code. 
    bool ret = spiral_force(start_, stop_, step_c_, step_f_, x_limit_, x_force_, dir_);

    if (ret)
        _task_status = TaskStatus::FINISHED;
    else
        _task_status = TaskStatus::FAILED;
    return _task_status;
}

bool SpiralTask::finish()
{
    auto force_srv = force_target(true, {1, 1, 1, 0, 0, 0}, {0, 0, 0, 0, 0, 0});
    force_srv.io = false;
    call_force_target(force_srv);
    return true;
}
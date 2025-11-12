#include "leitungssatz/Tasks/ToolChangeTask.hpp"

using namespace leitungssatz;

ToolChangeTask::ToolChangeTask(Robot_ptr robot)
    : Task(robot), rob(robot)
{
    configurePayload(ToolType::STANDARD);
    RCLCPP_INFO(rclcpp::get_logger("tool_change_task"), "ToolChangeTask initialized");
}

bool ToolChangeTask::configurePayload(ToolType type)
{
    geometry_msgs::msg::Vector3 cog; //center of gravity
    double mass;

    switch (type)
    {
    case ToolType::NONE:
        mass = 0.0;
        cog.x = 0.0;
        cog.y = 0.0;
        cog.z = 0.0;
        break;
    case ToolType::STANDARD:
        mass = 1.350; // Standard tool mass in kg
        cog.x = 0.003;
        cog.y = 0.002;
        cog.z = 0.050;
        break;
    case ToolType::HEAVY:
        mass = 2.180; // Heavy tool mass in kg
        cog.x = 0.005;
        cog.y = 0.024;
        cog.z = 0.094;
        break;
    }

    return rob->set_payload(mass, cog);
}

bool ToolChangeTask::moveY(double distance, double speed, double acceleration)
{
    auto rel_cart_srv = std::make_shared<leitungssatz_interfaces::srv::SetRelCartTarget::Request>();
    rel_cart_srv->mode = 1;
    rel_cart_srv->rel_goal = {0.0, distance, 0.0, 0.0, 0.0, 0.0};
    rel_cart_srv->speed = speed;
    rel_cart_srv->acceleration = acceleration;
    rel_cart_srv->asynchronous = false;
    
    return rob->call_rel_cart_target(rel_cart_srv);
}

bool ToolChangeTask::clip()
{

    auto request = std::make_shared<leitungssatz_interfaces::srv::SetClipGun::Request>();
    request->io = true; // Check this line during runtime. Differently implemented than original code

    if (!rob->setClipGun(true)) { 
        
        RCLCPP_ERROR(rclcpp::get_logger("tool_change_task"), "Failed to turn on clip gun");
        return false;
    }
    if (!rob->setClipGun(false)) { 
    
        RCLCPP_ERROR(rclcpp::get_logger("tool_change_task"), "Failed to turn off clip gun");
        return false;
    }
    rclcpp::sleep_for(std::chrono::seconds(1));
    return true;

    // Placeholder: always return true
    rclcpp::sleep_for(std::chrono::seconds(1));
    return true;
}

TaskStatus ToolChangeTask::execute()
{
    _task_status = TaskStatus::RUNNING;

    double VEL = 0.2;
    double ACC = 0.2;
    try
    {
        std::vector<Task_ptr> detach_sequence = {
            Task_ptr(new MoveTask(rob, "aproachChangeGripper", MOVEL, VEL, ACC)),
            Task_ptr(new MoveTask(rob, "aproachChangeGripper1", MOVEL, VEL, ACC)),
            Task_ptr(new MoveTask(rob, "preChangeGripper", MOVEL, VEL, ACC)),
            Task_ptr(new MoveTask(rob, "preChangeGripper1", MOVEL, VEL, ACC)),
            Task_ptr(new MoveTask(rob, "changeGripperUp", MOVEL, VEL, ACC)),
            Task_ptr(new MoveTask(rob, "changeGripperDown", MOVEL, VEL, ACC)),
            Task_ptr(new MoveTask(rob, "changeGripperHalfOut", MOVEL, VEL, ACC)),
            Task_ptr(new MoveTask(rob, "changeGripperOut", MOVEL, VEL, ACC)),
        };

        std::vector<Task_ptr> retreat_sequence = {
            Task_ptr(new MoveTask(rob, "noGripperUp", MOVEL, VEL, ACC)),
            Task_ptr(new MoveTask(rob, "noGripperUp2", MOVEL, VEL, ACC)),
            Task_ptr(new MoveTask(rob, "noGripperDown", MOVEL, VEL, ACC)),
        };

        std::vector<Task_ptr> return_sequence = {
            Task_ptr(new MoveTask(rob, "clipGunUp", MOVEL, VEL, ACC)),
            Task_ptr(new MoveTask(rob, "clipGunHalfOut", MOVEL, VEL, ACC)),
            Task_ptr(new MoveTask(rob, "clipGunOut", MOVEL, VEL, ACC)),
            Task_ptr(new MoveTask(rob, "clipGunUpRoute", MOVEL, VEL, ACC)),
            Task_ptr(new MoveTask(rob, "clipGunUpRoute2", MOVEL, VEL, ACC)),
            Task_ptr(new MoveTask(rob, "clipGunUpRoute3", MOVEL, VEL, ACC)),
            Task_ptr(new MoveTask(rob, "clipGunUpRoute4", MOVEL, VEL, ACC)),
            Task_ptr(new MoveTask(rob, "clipGunUpRoute5", MOVEL, VEL, ACC)),
            Task_ptr(new MoveTask(rob, "clipGunUpRoute6", MOVEL, VEL, ACC)),
            Task_ptr(new MoveTask(rob, "clipGunUpRoute7", MOVEL, VEL, ACC)),
            Task_ptr(new MoveTask(rob, "clipGunUpRoute8", MOVEL, VEL, ACC))
        };

        std::vector<Task_ptr> clip_sequence1 = {
            Task_ptr(new MoveTask(rob, "clipJoin1", MOVEL, VEL, ACC)),
        };

        std::vector<Task_ptr> clip_sequence2 = {
            Task_ptr(new MoveTask(rob, "clipJoin2", MOVEL, VEL, ACC)),
        };

        std::vector<Task_ptr> clip_sequence3 = {
            Task_ptr(new MoveTask(rob, "clipJoin3", MOVEL, VEL, ACC)),
        };

        std::vector<Task_ptr> clip_sequence4 = {
            Task_ptr(new MoveTask(rob, "clipJoin4", MOVEL, VEL, ACC)),
        };

        std::vector<Task_ptr> clip_sequence5 = {
            Task_ptr(new MoveTask(rob, "clipGunUpRoute9", MOVEL, VEL, ACC)),
            Task_ptr(new MoveTask(rob, "clipGunUpRoute10", MOVEL, VEL, ACC)),
            Task_ptr(new MoveTask(rob, "clipGunUpRoute11", MOVEL, VEL, ACC)),
            Task_ptr(new MoveTask(rob, "clipGunUpRoute12", MOVEL, VEL, ACC)),
            Task_ptr(new MoveTask(rob, "clipGunUpRoute13", MOVEL, VEL, ACC)),
            Task_ptr(new MoveTask(rob, "clipGunUpRoute14", MOVEL, VEL, ACC)),
            Task_ptr(new MoveTask(rob, "clipGunUpRoute15", MOVEL, VEL, ACC)),
            Task_ptr(new MoveTask(rob, "clipJoin5", MOVEL, VEL, ACC)),
        };

        std::vector<Task_ptr> last_sequence = {
            Task_ptr(new MoveTask(rob, "clipGunUpReturn1", MOVEL, VEL, ACC)),
            Task_ptr(new MoveTask(rob, "clipGunUpReturn2", MOVEL, VEL, ACC)),
            Task_ptr(new MoveTask(rob, "clipGunUpRoute14", MOVEL, VEL, ACC)),
            Task_ptr(new MoveTask(rob, "clipGunUpRoute13", MOVEL, VEL, ACC)),
            Task_ptr(new MoveTask(rob, "clipGunUpRoute12", MOVEL, VEL, ACC)),
            Task_ptr(new MoveTask(rob, "clipGunUpRoute11", MOVEL, VEL, ACC)),
            Task_ptr(new MoveTask(rob, "clipGunUpRoute10", MOVEL, VEL, ACC)),
            Task_ptr(new MoveTask(rob, "clipGunUpRoute3", MOVEL, VEL, ACC)),
            Task_ptr(new MoveTask(rob, "clipGunUpRoute2", MOVEL, VEL, ACC)),
            Task_ptr(new MoveTask(rob, "clipGunUpRoute", MOVEL, VEL, ACC)),
        };

        if (!configurePayload(ToolType::STANDARD))
        {
            RCLCPP_ERROR(rclcpp::get_logger("tool_change_task"), "Failed to configure payload for standard tool");
            return TaskStatus::FAILED;
        }

        for (auto &task : detach_sequence)
        {
            if (task->execute() != TaskStatus::FINISHED)
                return TaskStatus::FAILED;
        }

        if (!configurePayload(ToolType::NONE))
        {
            RCLCPP_ERROR(rclcpp::get_logger("tool_change_task"), "Failed to configure payload for no tool");
            return TaskStatus::FAILED;
        }

        for (auto &task : retreat_sequence)
        {
            if (task->execute() != TaskStatus::FINISHED)
                return TaskStatus::FAILED;
        }

        for (int i = 0; i < 5; ++i)
        {
            if (!moveY(0.002, VEL, ACC))
            {
                RCLCPP_ERROR(rclcpp::get_logger("tool_change_task"), "Failed to move robot in Y direction");
                return TaskStatus::FAILED;
            }
            rclcpp::sleep_for(std::chrono::milliseconds(500));
        }

        if (!configurePayload(ToolType::HEAVY))
        {
            RCLCPP_ERROR(rclcpp::get_logger("tool_change_task"), "Failed to configure payload for heavy tool");
            return TaskStatus::FAILED;
        }

        for (auto &task : return_sequence)
        {
            if (task->execute() != TaskStatus::FINISHED)
                return TaskStatus::FAILED;
        }

        for (auto &clip_sequence : {clip_sequence1, clip_sequence2, clip_sequence3, clip_sequence4, clip_sequence5})
        {
            for (auto &task : clip_sequence)
            {
                if (task->execute() != TaskStatus::FINISHED)
                    return TaskStatus::FAILED;
            }
            if (!clip())
                return TaskStatus::FAILED;
        }

        for (auto &task : last_sequence)
        {
            if (task->execute() != TaskStatus::FINISHED)
                return TaskStatus::FAILED;
        }

        _task_status = TaskStatus::FINISHED;
        return _task_status;
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("tool_change_task"), "Exception in ToolChangeTask: %s", e.what());
        _task_status = TaskStatus::FAILED;
        return _task_status;
    }
}
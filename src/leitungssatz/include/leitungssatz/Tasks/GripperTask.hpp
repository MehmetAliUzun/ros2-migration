#pragma once

#include "leitungssatz/Task.hpp"
#include <leitungssatz_interfaces/srv/set_gripper.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>

namespace leitungssatz {

class GripperTask : public Task
{
private:
    double _target_position;
    double _speed = 100;
    double _force = 0;
    bool _asynchronous = false;

    bool call_gripper(leitungssatz_interfaces::srv::SetGripper::Request &request, leitungssatz_interfaces::srv::SetGripper::Response &response);
    bool move_gripper();

public:
    GripperTask(Robot_ptr robot)
    : Task(robot)
    {
      _target_position = 100;
    } 

    GripperTask(Robot_ptr robot, double target_position, bool asynchronous = false)
    : Task(robot), _target_position(target_position), _asynchronous(asynchronous){}

    GripperTask(Robot_ptr robot, double target_position, double speed, double force, bool asynchronous = false)
    : Task(robot), _target_position(target_position), _speed(speed), _force(force), _asynchronous(asynchronous){}


    bool init();
    TaskStatus execute() override;

    virtual ~GripperTask();
};

} // namespace leitungssatz

/*This header file and its source file are manually checked for
consistency with ros1 version*/
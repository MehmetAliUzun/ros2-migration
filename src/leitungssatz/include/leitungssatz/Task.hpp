#pragma once

#include <memory>
#include "Robot.hpp"
#include "Gripper.hpp"
#include "Main.hpp"

namespace leitungssatz {

class Task {
protected:
    Robot_ptr _robot;
    TaskStatus _task_status = TaskStatus::IDLE;

public:
    Task(Robot_ptr robot) : _robot(robot) {}
    
    virtual bool init();
    virtual TaskStatus execute();

    virtual ~Task(); 
};

}  // namespace leitungssatz

/*This header file and its source file are manually checked for
consistency with ros1 version*/
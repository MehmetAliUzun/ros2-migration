#pragma once

#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <array>


typedef std::array<double, 6> array6d;
typedef std::array<long int, 6> array6i;

namespace leitungssatz {
    class Main;
    class TaskScheduler;
    class Task;
    class CameraNode;
    class IDSCamera;
    class Robot;
    class Gripper;
    class MoveTask;
    class GripperTask;
    class InterTask;
    class ForceTask;
    class SpiralTask;
    class GrabBuchseTask;
    class SetCableOrientationTask;
    class FlirCamera;
    class WirePush;
    class CableCheckTask;

    enum MoveType { MOVEJ, MOVEL, MOVEP, MOVEC };
    enum TaskStatus { IDLE, RUNNING, FINISHED, FAILED, FAILEDSAFE }; // IDLE = task still idling, RUNNING = task still running, FINISHED = task finished successfully, FAILED = task failed, FAILEDSAFE = task failed but it's safe to continue
    enum CableType { BLACKT1  = 1, BLACKT2 = 2, YELLOW_LOWER = 3, YELLOW_UPPER= 4, YELLOW2_LOWER = 5, YELLOW2_UPPER = 6, YELLOWT1 = 7 , YELLOWT2=8, YELLOW2T1 = 9, YELLOW2T2 = 10 ,BLACK = 11, UPPER_BLUE = 12, LOWER_BLUE=13};
    
    class Wrench;

    typedef std::shared_ptr<Task> Task_ptr;
    typedef std::shared_ptr<InterTask> InterTask_ptr;
    typedef std::shared_ptr<Wrench> Wrench_ptr;
    typedef std::shared_ptr<Robot> Robot_ptr;
}

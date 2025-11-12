#include <leitungssatz/TaskScheduler.hpp>


using namespace leitungssatz;

bool TaskScheduler::init(){
    return true; 
};

void TaskScheduler::run()
{
   int i = 1;
  
    for(InterTask_ptr currTask : _intertasks){
        //ROS_INFO(std::string("Executing Task " + std::to_string(i)).c_str());
        RCLCPP_INFO(rclcpp::get_logger("task_scheduler"), "Executing Task %d", i);
        TaskStatus status = currTask->exec();
        if (status == TaskStatus::FAILED){
            //ROS_ERROR("Aborting, task %d failed", i);
            RCLCPP_ERROR(rclcpp::get_logger("task_scheduler"), "Aborting, task %d failed", i);
            break;
        }

        i++;
    }

    
};
void TaskScheduler::addTask(InterTask_ptr task)
{
    _intertasks.push_back(task);
};

// This source file and its header file are manually checked for
// consistency with the ROS1 version.
#include "Main.hpp"
#include "Task.hpp"
#include "Tasks/InterTask.hpp"

#include "rclcpp/rclcpp.hpp"
#include <list>
#include <memory>

#pragma once

namespace leitungssatz {

    class TaskScheduler
    {
    private:
        
        std::list<InterTask_ptr> _intertasks;
        
    public:
        
        TaskScheduler() = default;
        /*This tells the compiler to generate the default constructor, 
        which automatically default-initializes all member variables 
        (including _intertasks).*/
        
        bool init();
        void run();
        void addTask(InterTask_ptr task);
    };
}
#include "leitungssatz/Main.hpp"
#include "leitungssatz/Task.hpp"
#include <memory>
#include <list>
#include <rclcpp/rclcpp.hpp>

#pragma once

namespace leitungssatz {
    
    class InterTask{

        public:
            InterTask() : InterTask(rclcpp::get_logger("InterTask")) {}
            
            InterTask(const rclcpp::Logger& logger) : _logger(logger), _queue(std::list<Task_ptr>()) {};

            TaskStatus exec();
            void queue(Task_ptr task);

        private:
            std::list<Task_ptr> _queue;
            TaskStatus status = TaskStatus::IDLE;
           
           rclcpp::Logger _logger = rclcpp::get_logger("inter_task");
            // rclcpp::Logger _logger;
    };
} // namespace leitungssatz
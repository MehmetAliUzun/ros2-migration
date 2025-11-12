#pragma once

#include <memory>
#include <string>
#include <stdexcept>

#include <leitungssatz/Task.hpp>
#include <leitungssatz_interfaces/srv/set_cart_target.hpp>
// Updated to use custom ROS2 interfaces through bridge node

namespace leitungssatz {
    class MoveTask : public Task {
        private:
            
            const std::string _parent_name = "base";
            std::string _point_name;
            MoveType _movetype;
            const double _vel = 0.5;
            const double _acc = 0.5;
            

        public:
            MoveTask();

            MoveTask(Robot_ptr robot): 
            Task(robot){
                init();
            };
            
            MoveTask(Robot_ptr robot, std::string point_name) : 
            Task(robot), _point_name(point_name){
                init();
            };
            
            MoveTask(Robot_ptr robot, std::string point_name, MoveType movetype) : 
            Task(robot), _point_name(point_name), _movetype(movetype){
                init();
            };
            
            MoveTask(Robot_ptr robot, std::string point_name, MoveType movetype, double vel, double acc) : 
            Task(robot), _point_name(point_name), _movetype(movetype), _vel(vel), _acc(acc){
                init();
            };
            //This last constructor is used in the main to construct moveHome task

            
            bool init(); 
            bool moveJ(); 
            bool moveL(); 
            bool moveP();
            bool moveC();
            TaskStatus execute() override;
};

}  // namespace leitungssatz

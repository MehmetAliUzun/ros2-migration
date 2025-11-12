#include <leitungssatz/Main.hpp>

#include <leitungssatz/Robot.hpp>
#include <leitungssatz/Gripper.hpp>
#include <leitungssatz/Wrench.hpp>
#include <leitungssatz/Task.hpp>
#include <leitungssatz/Tasks/MoveTask.hpp>
#include <leitungssatz/TaskScheduler.hpp>
#include <leitungssatz/Tasks/GripperTask.hpp>
#include <leitungssatz/Tasks/ForceTask.hpp>
#include <leitungssatz/Tasks/InterTask.hpp>

#include <rclcpp/rclcpp.hpp>
#include <boost/algorithm/string.hpp>
#include <vector>
#include <iostream>
#include <thread> // To be able to use multi-threading in node execution

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("main_leitungssatz");
    
    //---------------------------------
    
    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
    exec.add_node(node);
    std::thread spin_thread([&exec]() { exec.spin(); });

    //This part is to replace >ros::AsyncSpinner spinner(2)< logic
    //---------------------------------

    std::string eingabe;
    leitungssatz::Gripper grip = leitungssatz::Gripper(node);
    leitungssatz::Robot_ptr rob (new leitungssatz::Robot(node, grip));
    leitungssatz::Wrench_ptr wrench (new leitungssatz::Wrench(node));

    leitungssatz::TaskScheduler taskscheduler =  leitungssatz::TaskScheduler();
    leitungssatz::InterTask_ptr interTask (new leitungssatz::InterTask());

    rob->setSpeed(0.1);
    std::cout << "start? " << std::endl;
    std::cin >> eingabe;
    boost::algorithm::to_lower(eingabe);

    leitungssatz::Task_ptr moveHome( new leitungssatz::MoveTask(rob, "home", leitungssatz::MOVEJ));
    leitungssatz::Task_ptr grip_task_1(new leitungssatz::GripperTask(rob, 100, false));
   // leitungssatz::Task_ptr movepress( new leitungssatz::MoveTask(rob, "aufnahme_press", leitungssatz::MOVEL));
    leitungssatz::Task_ptr grip_task_2(new leitungssatz::GripperTask(rob,0, false));

    leitungssatz::Task_ptr forceMove(new leitungssatz::ForceTask(rob,node,1,{1,1,1,1,1,1},{20,0,0,0,0,0},3,wrench));
    leitungssatz::Task_ptr forceMove2(new leitungssatz::ForceTask(rob,node,1,{1,1,1,1,1,1},{0,20,0,0,0,0},3,wrench));
    leitungssatz::Task_ptr forceMove3(new leitungssatz::ForceTask(rob,node,1,{1,1,1,1,1,1},{0,0,19,0,0,0},3,wrench));
    
    //interTask->queue(moveHome);
    //interTask->queue(movepress);
    //interTask->queue(forceMove);

    //taskscheduler.addTask(interTask);
    
    if(eingabe == "yes")
    {
        //rob.ptpHome();
        moveHome->execute();
        //grip_task_1->execute();
        //movepress->execute();
        RCLCPP_INFO_ONCE(node->get_logger(), "x-d");
        forceMove->execute();
        RCLCPP_INFO_ONCE(node->get_logger(), "y-d");
        forceMove2->execute();
        RCLCPP_INFO_ONCE(node->get_logger(), "z-d");
        forceMove3->execute();
        
        //taskscheduler.run();
    }

    exec.cancel();
    if (spin_thread.joinable()) spin_thread.join();

    rclcpp::shutdown();
    return 0;
}
#include <leitungssatz/Main.hpp>

#include <leitungssatz/Robot.hpp>
#include <leitungssatz/Gripper.hpp>
#include <leitungssatz/Task.hpp>
#include <leitungssatz/TaskScheduler.hpp>
#include <leitungssatz/Tasks/MoveTask.hpp>
#include <leitungssatz/Tasks/GripperTask.hpp>
#include <leitungssatz/Tasks/InterTask.hpp>

#include <rclcpp/rclcpp.hpp>
#include <boost/algorithm/string.hpp>
#include <vector>
#include <iostream>
#include <thread>

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("grab_buchse");

   
    // rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
    // exec.add_node(node);
    // std::thread spin_thread([&exec]() { exec.spin(); });
    // Multi-threading above is not needed so I commented it out.

    std::string eingabe;
    leitungssatz::Gripper grip = leitungssatz::Gripper(node);
    leitungssatz::Robot_ptr rob (new leitungssatz::Robot(node, grip));

    rob->setSpeed(0.1);
    std::cout << "start? " << std::endl;
    std::cin >> eingabe;
    boost::algorithm::to_lower(eingabe);

    leitungssatz::TaskScheduler taskscheduler;
    
    leitungssatz::InterTask_ptr buchse (new leitungssatz::InterTask());

    leitungssatz::Task_ptr moveHome( new leitungssatz::MoveTask(rob, "home", leitungssatz::MOVEJ));
    //leitungssatz::Task_ptr moveHome( new leitungssatz::MoveTask(rob, "wrist_3_link", "tcp", leitungssatz::MOVEJ));
    leitungssatz::Task_ptr grip_task_1(new leitungssatz::GripperTask(rob, 100, false));
    leitungssatz::Task_ptr moveBuchseApp( new leitungssatz::MoveTask(rob, "buchse_app", leitungssatz::MOVEJ));
    leitungssatz::Task_ptr moveBuchseGrip( new leitungssatz::MoveTask(rob, "buchse_grip", leitungssatz::MOVEJ));
    //leitungssatz::Task_ptr grip_task_2(new leitungssatz::GripperTask(rob,0));
    buchse->queue(moveHome);
    buchse->queue(grip_task_1);
    buchse->queue(moveBuchseApp);
    buchse->queue(moveBuchseGrip);

    taskscheduler.addTask(buchse);
    //taskscheduler.addTask(grip_task_2);

    if(eingabe == "yes")
    {
        //rob.ptpHome();
        taskscheduler.run();
    }

    // Two below lines are for the mt exexutor above, hendce not needed so I commented them out
    // exec.cancel();
    // if (spin_thread.joinable()) spin_thread.join();
    
    
    rclcpp::shutdown();
    return 0;
}
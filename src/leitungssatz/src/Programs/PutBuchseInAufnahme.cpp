#include <leitungssatz/Main.hpp>
#include <leitungssatz/Robot.hpp>
#include <leitungssatz/Gripper.hpp>
#include <leitungssatz/Task.hpp>
#include <leitungssatz/Tasks/MoveTask.hpp>
#include <leitungssatz/TaskScheduler.hpp>
#include <leitungssatz/Tasks/GripperTask.hpp>

#include <rclcpp/rclcpp.hpp>
#include <boost/algorithm/string.hpp>
#include <vector>
#include <iostream>
#include <thread>

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("grab_buchse");

    // background executor to service callbacks while main thread runs tasks/user input
    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
    exec.add_node(node);
    std::thread spin_thread([&exec](){ exec.spin(); });

    std::string eingabe;
    leitungssatz::Gripper grip = leitungssatz::Gripper(node);
    leitungssatz::Robot_ptr rob (new leitungssatz::Robot(node, grip));

    rob->setSpeed(0.2);
    std::cout << "start? " << std::endl;
    std::cin >> eingabe;
    boost::algorithm::to_lower(eingabe);

    leitungssatz::Task_ptr moveHome( new leitungssatz::MoveTask(rob, "home", leitungssatz::MOVEJ));
    leitungssatz::Task_ptr grip_task_init(new leitungssatz::GripperTask(rob, 100));
    leitungssatz::Task_ptr moveBuchseApp( new leitungssatz::MoveTask(rob, "buchse_app", leitungssatz::MOVEJ));
    leitungssatz::Task_ptr moveBuchseGrip( new leitungssatz::MoveTask(rob, "buchse_grip", leitungssatz::MOVEJ));
    leitungssatz::Task_ptr grip_task_buchse(new leitungssatz::GripperTask(rob,0,0,15));
    leitungssatz::Task_ptr moveAufnahmeClose( new leitungssatz::MoveTask(rob, "aufnahme_2_close", leitungssatz::MOVEJ));
    leitungssatz::Task_ptr moveAufnahmeIn1( new leitungssatz::MoveTask(rob, "aufnahme_2_in_1", leitungssatz::MOVEJ));
    leitungssatz::Task_ptr grip_task_aufnahme_1(new leitungssatz::GripperTask(rob, rob->_gripper.position + 0.05,0,0));
    leitungssatz::Task_ptr moveAufnahmeIn2( new leitungssatz::MoveTask(rob, "aufnahme_2_in_2", leitungssatz::MOVEJ));
    leitungssatz::Task_ptr moveAufnahmeIn3( new leitungssatz::MoveTask(rob, "aufnahme_2_in_3", leitungssatz::MOVEJ));
    leitungssatz::Task_ptr grip_task_aufnahme_3(new leitungssatz::GripperTask(rob, 100));
    leitungssatz::Task_ptr moveAufnahmePressApp( new leitungssatz::MoveTask(rob, "aufnahme_2_press_app", leitungssatz::MOVEJ));
    leitungssatz::Task_ptr grip_task_aufnahme_Press(new leitungssatz::GripperTask(rob, 0));
    leitungssatz::Task_ptr moveAufnahmePress( new leitungssatz::MoveTask(rob, "aufnahme_2_press", leitungssatz::MOVEJ));

    if(eingabe == "yes")
    {
        moveHome->execute();
        grip_task_init->execute();
        moveBuchseApp->execute();
        moveBuchseGrip->execute();
        grip_task_buchse->execute();

        std::cout << "DID THE ROBOT DO ITS JOB? " << std::endl;
        std::cin >> eingabe;
        if(eingabe != "yes") {
            // stop executor and cleanup
            exec.cancel();
            if (spin_thread.joinable()) spin_thread.join();
            rclcpp::shutdown();
            return 0;
        }

        moveAufnahmeClose->execute();
        rob->setSpeed(0.05);
        moveAufnahmeIn1->execute();
        grip_task_aufnahme_1->execute();
        moveAufnahmeIn2->execute();
        moveAufnahmeIn3->execute();
        grip_task_aufnahme_3->execute();
        moveAufnahmePressApp->execute();
        grip_task_aufnahme_Press->execute();
        moveAufnahmePress->execute();
    }

    // stop executor and cleanup
    exec.cancel();
    if (spin_thread.joinable()) spin_thread.join();
    rclcpp::shutdown();
    return 0;
}

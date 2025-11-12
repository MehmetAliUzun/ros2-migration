#include "leitungssatz/Main.hpp"//
#include "leitungssatz/Robot.hpp"//
#include "leitungssatz/Gripper.hpp"//
#include "leitungssatz/Wrench.hpp"//
#include "leitungssatz/Task.hpp"//
#include "leitungssatz/Tasks/MoveTask.hpp"//
#include "leitungssatz/TaskScheduler.hpp"//
#include "leitungssatz/Tasks/GripperTask.hpp"//
#include "leitungssatz/Tasks/ForceTask.hpp"//
#include "leitungssatz/Tasks/InterTask.hpp"//
#include "leitungssatz/Tasks/SpiralTask.hpp"//
#include "leitungssatz/Tasks/GrabBuchseTask.hpp"//
#include "leitungssatz/Tasks/SetCableOrientationTask.hpp"//
#include "leitungssatz/Tasks/WirePushTask.hpp"//
#include "leitungssatz/Tasks/CableCheckTask.hpp"//
#include "leitungssatz/Tasks/ToolChangeTask.hpp"//

#include <vector>//
#include <memory>
#include <string>//
#include <chrono>
#include <cstdlib>

/*
Include directives are complete.

memory, chrono and stdlib are included for various 
functionalities such as memory management, 
time handling, and general utilities.

*/

bool isForceExceeded(const leitungssatz::Wrench_ptr& wrench, double force_limit) {
    return std::abs(wrench->wrenchMsg.wrench.force.x) > force_limit ||
           std::abs(wrench->wrenchMsg.wrench.force.y) > force_limit ||
           std::abs(wrench->wrenchMsg.wrench.force.z) > force_limit ;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("leitungssatz_main_node");
    /*Using auto allows the compiler to automatically deduce the type as 
    std::shared_ptr<rclcpp::Node>.
    Using a shared pointer ensures that the node remains alive as long as
    it is needed and is properly cleaned up when no longer in use.*/

    std::string eingabe;

    auto grip = std::make_shared<leitungssatz::Gripper>(node);
    /*This line creates a new instance of the Gripper class from 
    the leitungssatz namespace and stores it in a smart pointer 
    named grip. The std::make_shared<leitungssatz::Gripper>(node) 
    function allocates memory for the Gripper object and constructs
    it, passing the previously created ROS 2 node (node) as a 
    constructor argument.*/
    
    double VEL = 0.5;
    double ACC = 0.5;
    double force_limit = -2.5;
    auto rob = std::make_shared<leitungssatz::Robot>(node, *grip);
    auto wrench = std::make_shared<leitungssatz::Wrench>(node);

    rclcpp::sleep_for(std::chrono::milliseconds(500)); 
    
    //Gripper tasks
    auto grip_task_open_cable = std::make_shared<leitungssatz::GripperTask>(rob, 45, true);  // open gripper
    auto grip_task_pick_cable = std::make_shared<leitungssatz::GripperTask>(rob, 0, 100, 70, false); // close gripper
    
    auto grip_task_open_connector = std::make_shared<leitungssatz::GripperTask>(rob, 72, true); 
    auto grip_task_pick_connector = std::make_shared<leitungssatz::GripperTask>(rob, 0, 55, 30, false); // close gripper

    auto grip_task_pick_connector3 = std::make_shared<leitungssatz::GripperTask>(rob, 0, 80, 30, false); // close gripper

    auto grip_task_pick_connector4 = std::make_shared<leitungssatz::GripperTask>(rob, 0, 55, 30, false); // close gripper


    //Pick first terminal task
    auto moveHome = std::make_shared<leitungssatz::MoveTask>(rob, "home", leitungssatz::MOVEJ, VEL, ACC);

    auto cableApproachYellowT1 = std::make_shared<leitungssatz::MoveTask>(rob, "ApproachYellowTerminal1", leitungssatz::MOVEL, VEL, ACC);
    auto cableApproachYellowT12 = std::make_shared<leitungssatz::MoveTask>(rob, "ApproachYellowTerminal12", leitungssatz::MOVEL, VEL, ACC);

    auto cablePickApp = std::make_shared<leitungssatz::MoveTask>(rob, "PickYellowTerminal1", leitungssatz::MOVEL, VEL, ACC);
    auto cablePostPick = std::make_shared<leitungssatz::MoveTask>(rob, "PostPickYellowTerminal1", leitungssatz::MOVEL, VEL, ACC);
    auto cableCheckOrientationTerminal = std::make_shared<leitungssatz::MoveTask>(rob, "CheckOrientationPoint", leitungssatz::MOVEL, VEL, ACC);
    //auto SetcableOrientation = std::make_shared<leitungssatz::SetCableOrientationTask>(rob, node, leitungssatz::CableType::YELLOWT2, "/CableOrienter2/SetCableOrientation");
    
    //auto spiralmove_yellow = std::make_shared<leitungssatz::SpiralTask>(rob,node, wrench, 0.1,2.0,45,20,0.253,2,true); //default x-force = 1.8 and xlimit = 0.253 
    //auto insertionYellow = std::make_shared<leitungssatz::WirePush>(rob,wrench,node, "yellowT2_push_close", "yellowT2_push_far",2,10,10);
    
    auto PostInsertionPoint= std::make_shared<leitungssatz::MoveTask>(rob, "APPROACH_INSERT_YELLOWT22", leitungssatz::MOVEL, VEL, ACC);

    //Move to second Terminal 
    auto cableApproachYellowT2 = std::make_shared<leitungssatz::MoveTask>(rob, "ApproachYellowTerminal2", leitungssatz::MOVEL, VEL, ACC);
    auto cableApproachYellowT22 = std::make_shared<leitungssatz::MoveTask>(rob, "ApproachYellowTerminal22", leitungssatz::MOVEL, VEL, ACC);

    auto cablePickApp2 = std::make_shared<leitungssatz::MoveTask>(rob, "PickYellowTerminal2", leitungssatz::MOVEL, VEL, ACC); 
    auto cablePostPick2 = std::make_shared<leitungssatz::MoveTask>(rob, "PostPickYellowTerminal2", leitungssatz::MOVEL, VEL, ACC); 

    auto cableCheckOrientationTerminal2 = std::make_shared<leitungssatz::MoveTask>(rob, "CheckOrientationPoint", leitungssatz::MOVEL, VEL, ACC);
    //auto SetcableOrientation2 = std::make_shared<leitungssatz::SetCableOrientationTask>(rob, node, leitungssatz::CableType::YELLOWT1, "/CableOrienter2/SetCableOrientation");
    auto spiralmove_yellow2 = std::make_shared<leitungssatz::SpiralTask>(rob, node, wrench, 0.1,2.0,45,20,0.253,2,true); //default x-force = 1.8 and xlimit = 0.253 

    auto insertionYellowT2 = std::make_shared<leitungssatz::WirePush>(rob,wrench, node,"yellowT1_push_close","yellowT1_push_far",3,10,10);
    auto PostInsertionPoint21 = std::make_shared<leitungssatz::MoveTask>(rob, "PostInsertionPoint", leitungssatz::MOVEL, VEL, ACC);
    auto PostInsertionPoint2 = std::make_shared<leitungssatz::MoveTask>(rob, "FINISH_POINT_YELLOWT1", leitungssatz::MOVEL, VEL, ACC);

    //Move to Second Cable 
    auto cable2ApproachYellowT1 = std::make_shared<leitungssatz::MoveTask>(rob, "ApproachYellow2Terminal1", leitungssatz::MOVEL, VEL, ACC);
    auto cable2ApproachYellowT12 = std::make_shared<leitungssatz::MoveTask>(rob, "ApproachYellow2Terminal12", leitungssatz::MOVEL, VEL, ACC);

    auto cable2PickApp = std::make_shared<leitungssatz::MoveTask>(rob, "PickYellow2Terminal1", leitungssatz::MOVEL, VEL, ACC); 
    auto cable2PostPick = std::make_shared<leitungssatz::MoveTask>(rob, "PostPickYellow2Terminal1", leitungssatz::MOVEL, VEL, ACC); 

    auto cable2CheckOrientationTerminal1 = std::make_shared<leitungssatz::MoveTask>(rob, "CheckOrientationPoint", leitungssatz::MOVEL, VEL, ACC);
    //auto Setcable2Orientation = std::make_shared<leitungssatz::SetCableOrientationTask>(rob, node, leitungssatz::CableType::YELLOW2T2, "/CableOrienter2/SetCableOrientation");
    auto spiralmove2_yellow = std::make_shared<leitungssatz::SpiralTask>(rob, node, wrench, 0.1,2.0,45,20,0.253,2,true); //default x-force = 1.8 and xlimit = 0.253 

    auto insertionYellow2T2 = std::make_shared<leitungssatz::WirePush>(rob,wrench, node,"yellow2T2_push_close","yellow2T2_push_far",2,10,10);

    auto PostInsertionPoint3 = std::make_shared<leitungssatz::MoveTask>(rob, "APPROACH_POINT2_YELLOW2_T2", leitungssatz::MOVEL, VEL, ACC);

    //Second Yellow Cable Terminal2 
    
    auto cable2ApproachYellowT2 = std::make_shared<leitungssatz::MoveTask>(rob, "ApproachYellow2Terminal2", leitungssatz::MOVEL, VEL, ACC);
    auto cable2ApproachYellowT22 = std::make_shared<leitungssatz::MoveTask>(rob, "ApproachYellow2Terminal22", leitungssatz::MOVEL, VEL, ACC);

    auto cable2PickApp2 = std::make_shared<leitungssatz::MoveTask>(rob, "PickYellow2Terminal2", leitungssatz::MOVEL, VEL, ACC); 
    auto cable2PostPick2 = std::make_shared<leitungssatz::MoveTask>(rob, "PostPickYellow2Terminal2", leitungssatz::MOVEL, VEL, ACC); 

    auto cable2CheckOrientationTerminal2 = std::make_shared<leitungssatz::MoveTask>(rob, "CheckOrientationPoint", leitungssatz::MOVEL, VEL, ACC);
    //auto Setcable2Orientation2 = std::make_shared<leitungssatz::SetCableOrientationTask>(rob, node , leitungssatz::CableType::YELLOW2T1, "/CableOrienter2/SetCableOrientation");
    auto spiralmove2_yellow2 = std::make_shared<leitungssatz::SpiralTask>(rob, node, wrench, 0.1,2.0,45,20,0.253,2,true); //default x-force = 1.8 and xlimit = 0.253 

    auto insertionYellow2T1 = std::make_shared<leitungssatz::WirePush>(rob,wrench,node,"yellow2T1_push_close","yellow2T1_push_far",3,10,10);

    auto PostInsertionPoint4 = std::make_shared<leitungssatz::MoveTask>(rob, "APPROACH_POINT2_YELLOW2_T1", leitungssatz::MOVEL, VEL, ACC);

    // Black cable Temrinal 1 
    
    auto cableApproachBlackT1 = std::make_shared<leitungssatz::MoveTask>(rob, "ApproachBlackTerminal1", leitungssatz::MOVEL, VEL, ACC);
    auto cableApproachBlackT12 = std::make_shared<leitungssatz::MoveTask>(rob, "ApproachBlackTerminal12", leitungssatz::MOVEL, VEL, ACC);
    auto cableBlackPickApp = std::make_shared<leitungssatz::MoveTask>(rob, "PickBlackTerminal1", leitungssatz::MOVEL, VEL, ACC); 
    auto cableBlackPostPick = std::make_shared<leitungssatz::MoveTask>(rob, "PostPickBlackTerminal1", leitungssatz::MOVEL, VEL, ACC);
    auto cableBlackCheckOrientation = std::make_shared<leitungssatz::MoveTask>(rob, "CheckOrientationPointBlack", leitungssatz::MOVEL, VEL, ACC);
    //auto SetcableBlackOrientation = std::make_shared<leitungssatz::SetCableOrientationTask>(rob, node , leitungssatz::CableType::BLACKT1, "/CableOrienter2/SetCableOrientation");
    
    
    // Close connectors Tasks -> If problems occur try this option: auto move_home = std::make_shared<leitungssatz::InterTask>(node->get_logger());

     auto move_home = std::make_shared<leitungssatz::InterTask>(node->get_logger());

     auto connector1Grasping = std::make_shared<leitungssatz::InterTask>(node->get_logger());
     auto connector2Grasping = std::make_shared<leitungssatz::InterTask>(node->get_logger());
     auto connector3Grasping = std::make_shared<leitungssatz::InterTask>(node->get_logger());
     auto connector4Grasping = std::make_shared<leitungssatz::InterTask>(node->get_logger());

     auto cable_red = std::make_shared<leitungssatz::InterTask>(node->get_logger());
     auto cable_yellow = std::make_shared<leitungssatz::InterTask>(node->get_logger());
     auto cable_yellow2 = std::make_shared<leitungssatz::InterTask>(node->get_logger());

     auto cable_black = std::make_shared<leitungssatz::InterTask>(node->get_logger());
     auto cable_orientation = std::make_shared<leitungssatz::InterTask>(node->get_logger());

    // generic gripper tasks for connector pick and insert
    auto full_open_gripper = std::make_shared<leitungssatz::GripperTask>(rob, 100, false);  
    auto half_open_gripper = std::make_shared<leitungssatz::GripperTask>(rob, 50, false);
    auto third_open_gripper = std::make_shared<leitungssatz::GripperTask>(rob, 35, false);
    auto connector2_close_gripper = std::make_shared<leitungssatz::GripperTask>(rob, 35, 100, 45, false); 
    auto connector2_close_gripper2 = std::make_shared<leitungssatz::GripperTask>(rob, 25, 100, 45, false); 
    auto connector2_push_gripper = std::make_shared<leitungssatz::GripperTask>(rob, 35, false);
    auto connector3_close_gripper = std::make_shared<leitungssatz::GripperTask>(rob, 15, 100, 45, false); 
    auto connector3_push_gripper = std::make_shared<leitungssatz::GripperTask>(rob, 25, false);
    auto connector4_close_gripper = std::make_shared<leitungssatz::GripperTask>(rob, 30, 100, 45, false); 
    auto full_close_gripper = std::make_shared<leitungssatz::GripperTask>(rob, 0, 100, 45, false); 


    // Conector 1 pick up and insert task
    auto connector1_pick_up_and_insert = std::make_shared<leitungssatz::InterTask>(node->get_logger());

    auto connector1PrePickup = std::make_shared<leitungssatz::MoveTask>(rob, "connector1PrePickup", leitungssatz::MOVEL, VEL, ACC);
    auto connector1PullOut = std::make_shared<leitungssatz::MoveTask>(rob, "connector1PullOut", leitungssatz::MOVEL, VEL, ACC);
    auto connector1HalfPull = std::make_shared<leitungssatz::MoveTask>(rob, "connector1HalfPull", leitungssatz::MOVEJ, VEL, ACC);
    auto connector1PreClose = std::make_shared<leitungssatz::MoveTask>(rob, "connector1PreClose", leitungssatz::MOVEL, VEL, ACC);
    auto connector1Close = std::make_shared<leitungssatz::MoveTask>(rob, "connector1Close", leitungssatz::MOVEL, VEL, ACC);
    auto connector1Out = std::make_shared<leitungssatz::MoveTask>(rob, "connector1Out", leitungssatz::MOVEL, VEL, ACC);
    auto connector1PreInsert = std::make_shared<leitungssatz::MoveTask>(rob, "connector1PreInsert", leitungssatz::MOVEL, VEL, ACC);
    auto connector1PreInsert1 = std::make_shared<leitungssatz::MoveTask>(rob, "connector1PreInsert1", leitungssatz::MOVEL, VEL, ACC);
    auto connector1InitialInsert = std::make_shared<leitungssatz::MoveTask>(rob, "connector1InitialInsert", leitungssatz::MOVEL, VEL, ACC);
    auto connector1InitialInsert1 = std::make_shared<leitungssatz::MoveTask>(rob, "connector1InitialInsert1", leitungssatz::MOVEL, VEL, ACC);
    auto connector1Insert = std::make_shared<leitungssatz::MoveTask>(rob, "connector1Insert", leitungssatz::MOVEL, VEL, ACC);
    auto connector1PostInsert = std::make_shared<leitungssatz::MoveTask>(rob, "connector1PostInsert", leitungssatz::MOVEL, VEL, ACC);
    auto connector1PrePush = std::make_shared<leitungssatz::MoveTask>(rob, "connector1PrePush", leitungssatz::MOVEL, VEL, ACC);
    auto connector1Push = std::make_shared<leitungssatz::MoveTask>(rob, "connector1Push", leitungssatz::MOVEL, VEL, ACC);
    auto connector1Done = std::make_shared<leitungssatz::MoveTask>(rob, "connector1Done", leitungssatz::MOVEL, VEL, ACC);

    // Conector 2 pick up and insert task
    auto connector2_pick_up_and_insert = std::make_shared<leitungssatz::InterTask>(node->get_logger());

    auto connector2PrePickup = std::make_shared<leitungssatz::MoveTask>(rob, "connector2PrePickup", leitungssatz::MOVEL, VEL, ACC);
    auto connector2PullOut = std::make_shared<leitungssatz::MoveTask>(rob, "connector2PullOut", leitungssatz::MOVEL, VEL, ACC);
    auto connector2HalfOut = std::make_shared<leitungssatz::MoveTask>(rob, "connector2HalfOut", leitungssatz::MOVEL, VEL, ACC);
    auto connector2Out = std::make_shared<leitungssatz::MoveTask>(rob, "connector2Out", leitungssatz::MOVEL, VEL, ACC);
    auto connector2PreInsert = std::make_shared<leitungssatz::MoveTask>(rob, "connector2PreInsert", leitungssatz::MOVEL, VEL, ACC);
    auto connector2PreInsert1 = std::make_shared<leitungssatz::MoveTask>(rob, "connector2PreInsert1", leitungssatz::MOVEL, VEL, ACC);
    auto connector2PreInsert2 = std::make_shared<leitungssatz::MoveTask>(rob, "connector2PreInsert2", leitungssatz::MOVEL, VEL, ACC);
    auto connector2InitialInsert = std::make_shared<leitungssatz::MoveTask>(rob, "connector2InitialInsert", leitungssatz::MOVEL, VEL, ACC);
    auto connector2PreClose = std::make_shared<leitungssatz::MoveTask>(rob, "connector2PreClose", leitungssatz::MOVEL, VEL, ACC);
    auto connector2Close = std::make_shared<leitungssatz::MoveTask>(rob, "connector2Close", leitungssatz::MOVEL, VEL, ACC);
    auto connector2PrePush = std::make_shared<leitungssatz::MoveTask>(rob, "connector2PrePush", leitungssatz::MOVEL, VEL, ACC);
    auto connector2Push = std::make_shared<leitungssatz::MoveTask>(rob, "connector2Push", leitungssatz::MOVEL, VEL, ACC);
    auto connector2Done = std::make_shared<leitungssatz::MoveTask>(rob, "connector1Done", leitungssatz::MOVEL, VEL, ACC);


    // Conector 3 pick up and insert task
    auto connector3_pick_up_and_insert = std::make_shared<leitungssatz::InterTask>(node->get_logger());

    auto connector3PrePickup = std::make_shared<leitungssatz::MoveTask>(rob, "connector3PrePickup", leitungssatz::MOVEL, VEL, ACC);
    auto connector3PullOut = std::make_shared<leitungssatz::MoveTask>(rob, "connector3PullOut", leitungssatz::MOVEL, VEL, ACC);
    auto connector3HalfPull = std::make_shared<leitungssatz::MoveTask>(rob, "connector3HalfPull", leitungssatz::MOVEJ, VEL, ACC);
    auto connector3Close = std::make_shared<leitungssatz::MoveTask>(rob, "connector3Close", leitungssatz::MOVEL, VEL, ACC);
    auto connector3HalfPull2 = std::make_shared<leitungssatz::MoveTask>(rob, "connector3HalfPull2", leitungssatz::MOVEJ, VEL, ACC);
    auto connector3Out = std::make_shared<leitungssatz::MoveTask>(rob, "connector3Out", leitungssatz::MOVEL, VEL, ACC);
    auto connector3PreClose = std::make_shared<leitungssatz::MoveTask>(rob, "connector3PreClose", leitungssatz::MOVEL, VEL, ACC);
    auto connector3PreClose2 = std::make_shared<leitungssatz::MoveTask>(rob, "connector3PreClose2", leitungssatz::MOVEL, VEL, ACC);
    auto connector3Close2 = std::make_shared<leitungssatz::MoveTask>(rob, "connector3Close2", leitungssatz::MOVEL, VEL, ACC);
    auto connector3PreInsert = std::make_shared<leitungssatz::MoveTask>(rob, "connector3PreInsert", leitungssatz::MOVEL, VEL, ACC);
    auto connector3PreInsert1 = std::make_shared<leitungssatz::MoveTask>(rob, "connector3PreInsert1", leitungssatz::MOVEL, VEL, ACC);
    auto connector3InitialInsert = std::make_shared<leitungssatz::MoveTask>(rob, "connector3InitialInsert", leitungssatz::MOVEL, VEL, ACC);
    auto connector3Push = std::make_shared<leitungssatz::MoveTask>(rob, "connector3Push", leitungssatz::MOVEL, VEL, ACC);
    auto connector3Done = std::make_shared<leitungssatz::MoveTask>(rob, "connector3Done", leitungssatz::MOVEL, VEL, ACC);

    // Conector 4 pick up and insert task

    auto connector4_pick_up_and_insert = std::make_shared<leitungssatz::InterTask>(node->get_logger());

    auto connector4PrePickup = std::make_shared<leitungssatz::MoveTask>(rob, "connector4PrePickup", leitungssatz::MOVEL, VEL, ACC);
    auto connector4PullOut = std::make_shared<leitungssatz::MoveTask>(rob, "connector4PullOut", leitungssatz::MOVEL, VEL, ACC);
    auto connector4PreClose = std::make_shared<leitungssatz::MoveTask>(rob, "connector4PreClose", leitungssatz::MOVEL, VEL, ACC);
    auto connector4PreClose2 = std::make_shared<leitungssatz::MoveTask>(rob, "connector4PreClose2", leitungssatz::MOVEL, VEL, ACC);
    auto connector4PreClose3 = std::make_shared<leitungssatz::MoveTask>(rob, "connector4PreClose3", leitungssatz::MOVEL, VEL, ACC);
    auto connector4PreClose4 = std::make_shared<leitungssatz::MoveTask>(rob, "connector4PreClose4", leitungssatz::MOVEL, VEL, ACC);
    auto connector4PreClose5 = std::make_shared<leitungssatz::MoveTask>(rob, "connector4PreClose5", leitungssatz::MOVEL, VEL, ACC);
    auto connector4PreClose6 = std::make_shared<leitungssatz::MoveTask>(rob, "connector4PreClose6", leitungssatz::MOVEL, VEL, ACC);
    auto connector4PreClose7 = std::make_shared<leitungssatz::MoveTask>(rob, "connector4PreClose7", leitungssatz::MOVEL, VEL, ACC);
    auto connector4Close = std::make_shared<leitungssatz::MoveTask>(rob, "connector4Close", leitungssatz::MOVEL, VEL, ACC);
    auto connector4HalfOut = std::make_shared<leitungssatz::MoveTask>(rob, "connector4HalfOut", leitungssatz::MOVEL, VEL, ACC);
    auto connector4Out = std::make_shared<leitungssatz::MoveTask>(rob, "connector4Out", leitungssatz::MOVEL, VEL, ACC);
    auto connector4PreInsert = std::make_shared<leitungssatz::MoveTask>(rob, "connector4PreInsert", leitungssatz::MOVEL, VEL, ACC);
    auto connector4PreInsert1 = std::make_shared<leitungssatz::MoveTask>(rob, "connector4PreInsert1", leitungssatz::MOVEL, VEL, ACC);
    auto connector4InitialInsert = std::make_shared<leitungssatz::MoveTask>(rob, "connector4InitialInsert", leitungssatz::MOVEL, VEL, ACC);
    auto connector4PrePush = std::make_shared<leitungssatz::MoveTask>(rob, "connector4PrePush", leitungssatz::MOVEL, VEL, ACC);
    auto connector4Push = std::make_shared<leitungssatz::MoveTask>(rob, "connector4Push", leitungssatz::MOVEL, VEL, ACC);
    auto connector4Done = std::make_shared<leitungssatz::MoveTask>(rob, "connector4Done", leitungssatz::MOVEL, VEL, ACC);

    // Conector 4 pick up and insert task
    auto connector2_to_tool_change = std::make_shared<leitungssatz::InterTask>(node->get_logger());

    auto preToolChange1 = std::make_shared<leitungssatz::MoveTask>(rob, "preToolChange1", leitungssatz::MOVEL, VEL, ACC);
    auto preToolChange2 = std::make_shared<leitungssatz::MoveTask>(rob, "preToolChange2", leitungssatz::MOVEL, VEL, ACC);



    // Tool change task
    auto tool_change = std::make_shared<leitungssatz::InterTask>(node->get_logger());

    auto aproachChangeGripper = std::make_shared<leitungssatz::MoveTask>(rob, "aproachChangeGripper", leitungssatz::MOVEL, VEL, ACC);
    auto aproachChangeGripper1 = std::make_shared<leitungssatz::MoveTask>(rob, "aproachChangeGripper1", leitungssatz::MOVEL, VEL, ACC);
    auto preChangeGripper = std::make_shared<leitungssatz::MoveTask>(rob, "preChangeGripper", leitungssatz::MOVEL, VEL, ACC);
    auto preChangeGripper1 = std::make_shared<leitungssatz::MoveTask>(rob, "preChangeGripper1", leitungssatz::MOVEL, VEL, ACC);
    auto changeGripperUp = std::make_shared<leitungssatz::MoveTask>(rob, "changeGripperUp", leitungssatz::MOVEL, VEL, ACC);
    auto changeGripperDown = std::make_shared<leitungssatz::MoveTask>(rob, "changeGripperDown", leitungssatz::MOVEL, VEL, ACC);
    auto changeGripperHalfOut = std::make_shared<leitungssatz::MoveTask>(rob, "changeGripperHalfOut", leitungssatz::MOVEL, VEL, ACC);
    auto changeGrippeyellow2T2_push_farrOut = std::make_shared<leitungssatz::MoveTask>(rob, "changeGripperOut", leitungssatz::MOVEL, VEL, ACC);
    auto noGripperUp = std::make_shared<leitungssatz::MoveTask>(rob, "noGripperUp", leitungssatz::MOVEL, VEL, ACC);
    auto noGripperUp2 = std::make_shared<leitungssatz::MoveTask>(rob, "noGripperUp2", leitungssatz::MOVEL, VEL, ACC);
    auto noGripperDown = std::make_shared<leitungssatz::MoveTask>(rob, "noGripperDown", leitungssatz::MOVEL, VEL, ACC);
    auto clipGunHalfIn = std::make_shared<leitungssatz::MoveTask>(rob, "clipGunHalfIn", leitungssatz::MOVEL, VEL, ACC);
    auto clipGunIn = std::make_shared<leitungssatz::MoveTask>(rob, "clipGunIn", leitungssatz::MOVEL, VEL, ACC);

    
    
    
    move_home->queue(moveHome);

          
    // First Yellow Cable 
    cable_yellow ->queue(grip_task_open_cable);
    cable_yellow ->queue(cableApproachYellowT1);
    cable_yellow ->queue(cableApproachYellowT12);
    cable_yellow ->queue(cablePickApp);
    cable_yellow ->queue(grip_task_pick_cable);
    cable_yellow ->queue(cablePostPick);
    cable_yellow ->queue(cableCheckOrientationTerminal);
    //cable_yellow ->queue(SetcableOrientation);
    //cable_yellow ->queue(insertionYellow);
    cable_yellow ->queue(grip_task_open_cable);
    cable_yellow ->queue(PostInsertionPoint);
    //cable_yellow ->queue(spiralmove_yellow)
   
    cable_yellow ->queue(cableApproachYellowT2);
    cable_yellow ->queue(cableApproachYellowT22);
    cable_yellow ->queue(cablePickApp2);
    cable_yellow ->queue(grip_task_pick_cable);
    cable_yellow ->queue(cablePostPick2);
    cable_yellow ->queue(cableCheckOrientationTerminal2);
    //cable_yellow ->queue(SetcableOrientation2);
    cable_yellow ->queue(insertionYellowT2);
    cable_yellow ->queue(grip_task_open_cable);
    cable_yellow ->queue(PostInsertionPoint21);
    cable_yellow ->queue(PostInsertionPoint2);

    //Second Yellow Cable
    cable_yellow2 ->queue(grip_task_open_cable);
    cable_yellow2 ->queue(cable2ApproachYellowT1);
    cable_yellow2 ->queue(cable2ApproachYellowT12);
    cable_yellow2 ->queue(cable2PickApp);
    cable_yellow2 ->queue(grip_task_pick_cable);
    cable_yellow2 ->queue(cable2PostPick);   
    cable_yellow2 ->queue(cable2CheckOrientationTerminal1);
    //cable_yellow2 ->queue(Setcable2Orientation);
    cable_yellow2 ->queue(insertionYellow2T2);
    cable_yellow2 ->queue(grip_task_open_cable);
    cable_yellow2 ->queue(PostInsertionPoint3);

    cable_yellow2 ->queue(grip_task_open_cable);
    cable_yellow2 ->queue(cable2ApproachYellowT2);
    cable_yellow2 ->queue(cable2ApproachYellowT22);
    cable_yellow2 ->queue(cable2PickApp2);
    cable_yellow2 ->queue(grip_task_pick_cable);
    cable_yellow2 ->queue(cable2PostPick);
    cable_yellow2 ->queue(cable2CheckOrientationTerminal2);
    //cable_yellow2 ->queue(Setcable2Orientation2);
    cable_yellow2 ->queue(insertionYellow2T1);
    cable_yellow2 ->queue(grip_task_open_cable);
    cable_yellow2 ->queue(PostInsertionPoint4);


    RCLCPP_INFO(node->get_logger(), "Wrench Force in Y %f", wrench->wrenchMsg.wrench.force.y);

    //First Black Cable

    cable_black ->queue(grip_task_open_cable);
    cable_black ->queue(cableApproachBlackT1);
    cable_black ->queue(cableApproachBlackT12);
    cable_black ->queue(cableBlackPickApp);
    cable_black ->queue(grip_task_pick_cable);
    cable_black ->queue(cableBlackPostPick);
    cable_black ->queue(cableBlackCheckOrientation);
    //cable_black ->queue(SetcableBlackOrientation);
    
  //  if (isForceExceeded(wrench, force_limit)) {
       // cable_yellow->queue(spiralmove_yellow);
    //}
    
   
    
   // cable_yellow ->queue(grip_task_open_cable);
   // cable_yellow ->queue(PostInsertionPoint);

   // cable_yellow ->queue(grip_task_pick_cable);
    //cable_yellow ->queue(SetcableOrientationUpper);
    //cable_yellow ->queue(insertionYellowUpper); 

    //Second Yellow Cable
   // cable_yellow ->queue(grip_task_open_cable);
   // cable_yellow ->queue(TransitionMove);
   // cable_yellow ->queue(TransitionMove2);
    
   // cable_yellow ->queue(cable2ApproachYellow);
   // cable_yellow ->queue(cable2ApproachYellow2);
   // cable_yellow ->queue(cable2PickApp);
   // cable_yellow ->queue(grip_task_pick_cable);
  //  cable_yellow ->queue(cable2PostPick);
   // cable_yellow ->queue(cable2CheckOrientation);
   // cable_yellow ->queue(SaveTerminal2Orientation);
   // cable_yellow ->queue(cable2CheckOrientationT2);
  //  cable_yellow ->queue(Setcable2Orientation);


    
    //cable_yellow ->queue(cableCheckOrientationT1);
    //cable_yellow ->queue(ApproachConnectorYellowT2);
   // cable_yellow ->queue(ApproachConnectorYellowT21);
   // cable_yellow ->queue(VibrationInsertionYellowT2);
    //cable_yellow ->queue(spiralmove_yellow);
    //cable_yellow ->queue(grip_task_open_cable);
   // cable_yellow ->queue(CompleteInsertionYellowT2);
   


    //cable_yellow ->queue(moveYellowT2);

   // cable_orientation ->queue(cableOrientationCheck);
   // cable_orientation ->queue(cableRedOrientation);

    // Connector 1 Pick up and Insert Task
    connector1_pick_up_and_insert->queue(full_open_gripper);
    connector1_pick_up_and_insert->queue(connector1PrePickup);
    connector1_pick_up_and_insert->queue(connector1PullOut);
    connector1_pick_up_and_insert->queue(connector4_close_gripper);

    connector1_pick_up_and_insert->queue(connector1HalfPull);
    connector1_pick_up_and_insert->queue(full_open_gripper);
    
    connector1_pick_up_and_insert->queue(connector1PrePickup);
    connector1_pick_up_and_insert->queue(full_close_gripper);
    connector1_pick_up_and_insert->queue(connector1PreClose);
    connector1_pick_up_and_insert->queue(connector1Close);

    connector1_pick_up_and_insert->queue(connector1PreClose);
    connector1_pick_up_and_insert->queue(third_open_gripper);
    connector1_pick_up_and_insert->queue(connector1Close);
    
    connector1_pick_up_and_insert->queue(connector1PrePickup);
    connector1_pick_up_and_insert->queue(full_open_gripper);

    connector1_pick_up_and_insert->queue(connector1PullOut);
    connector1_pick_up_and_insert->queue(connector4_close_gripper);
    connector1_pick_up_and_insert->queue(connector1Out);
    connector1_pick_up_and_insert->queue(connector1PreInsert);
    connector1_pick_up_and_insert->queue(connector1PreInsert1);

    connector1_pick_up_and_insert->queue(connector1InitialInsert);
    connector1_pick_up_and_insert->queue(full_open_gripper);

    connector1_pick_up_and_insert->queue(connector1PreInsert1);
    connector1_pick_up_and_insert->queue(half_open_gripper);
    connector1_pick_up_and_insert->queue(connector1Push);

    // Connector 2 Pick up and Insert Task
    connector2_pick_up_and_insert->queue(full_open_gripper);
    connector2_pick_up_and_insert->queue(connector2PrePickup);
    connector2_pick_up_and_insert->queue(connector2PullOut);
    connector2_pick_up_and_insert->queue(full_close_gripper);
    connector2_pick_up_and_insert->queue(connector2Out);

    connector2_pick_up_and_insert->queue(connector2PreInsert);
    connector2_pick_up_and_insert->queue(connector2PreInsert1);

    connector2_pick_up_and_insert->queue(connector2InitialInsert);
    connector2_pick_up_and_insert->queue(full_open_gripper);

    connector2_pick_up_and_insert->queue(connector2PreInsert1);
    connector2_pick_up_and_insert->queue(connector2PreClose);
    connector2_pick_up_and_insert->queue(full_close_gripper);

    connector2_pick_up_and_insert->queue(connector2Close);
    connector2_pick_up_and_insert->queue(connector2PreClose);
    connector2_pick_up_and_insert->queue(connector2Close);
    connector2_pick_up_and_insert->queue(connector2PreClose);

    connector2_pick_up_and_insert->queue(connector2PreInsert1);
    connector2_pick_up_and_insert->queue(connector2_push_gripper);
    connector2_pick_up_and_insert->queue(connector2Push);
    connector2_pick_up_and_insert->queue(connector2PreInsert1);

    // Connector 3 Pick up and Insert Task
    connector3_pick_up_and_insert->queue(full_open_gripper);    
    connector3_pick_up_and_insert->queue(connector3PrePickup);
    // connector3_pick_up_and_insert->queue(full_close_gripper);
    // connector3_pick_up_and_insert->queue(connector3Close);

    // connector3_pick_up_and_insert->queue(connector3PrePickup);
    // connector3_pick_up_and_insert->queue(connector3Close);
    
    // connector3_pick_up_and_insert->queue(connector3PrePickup);
    // connector3_pick_up_and_insert->queue(full_open_gripper);
    connector3_pick_up_and_insert->queue(connector3PullOut);
    connector3_pick_up_and_insert->queue(connector3_close_gripper);
    connector3_pick_up_and_insert->queue(connector3Out);

    // connector3_pick_up_and_insert->queue(connector3PreClose);
    // connector3_pick_up_and_insert->queue(connector3PreClose2);
    // connector3_pick_up_and_insert->queue(connector3Close2);

    // connector3_pick_up_and_insert->queue(connector3PreClose2);
    // connector3_pick_up_and_insert->queue(connector3Close2);
    // connector3_pick_up_and_insert->queue(connector3PreClose2);
    // connector3_pick_up_and_insert->queue(connector3PreClose);

    connector3_pick_up_and_insert->queue(connector3PreInsert);
    connector3_pick_up_and_insert->queue(connector3PreInsert1);
    connector3_pick_up_and_insert->queue(connector3InitialInsert);
    connector3_pick_up_and_insert->queue(full_open_gripper);
    connector3_pick_up_and_insert->queue(connector3PreInsert1);

    connector3_pick_up_and_insert->queue(connector3_push_gripper);
    connector3_pick_up_and_insert->queue(connector3Push);
    connector3_pick_up_and_insert->queue(connector3PreInsert1);
    connector3_pick_up_and_insert->queue(connector3Done);

    // Connector 4 Pick up and Insert Task
    connector4_pick_up_and_insert->queue(full_open_gripper);
    connector4_pick_up_and_insert->queue(connector4PrePickup);
    connector4_pick_up_and_insert->queue(connector4PullOut);
    connector4_pick_up_and_insert->queue(full_close_gripper);
    // connector4_pick_up_and_insert->queue(connector4HalfOut);
    // connector4_pick_up_and_insert->queue(full_open_gripper);
    // connector4_pick_up_and_insert->queue(connector4PullOut);
    // connector4_pick_up_and_insert->queue(connector4_close_gripper);
    connector4_pick_up_and_insert->queue(connector4Out);

    // connector4_pick_up_and_insert->queue(connector4PreClose);
    // connector4_pick_up_and_insert->queue(connector4PreClose2);
    // connector4_pick_up_and_insert->queue(connector4PreClose3);
    // connector4_pick_up_and_insert->queue(full_open_gripper);
    // connector4_pick_up_and_insert->queue(connector4PreClose2);
    // connector4_pick_up_and_insert->queue(connector4PrePush);

    // connector4_pick_up_and_insert->queue(half_open_gripper);
    // connector4_pick_up_and_insert->queue(connector4Push);
    // connector4_pick_up_and_insert->queue(connector4PrePush);
    // connector4_pick_up_and_insert->queue(connectorPreClose);
    // connector4_pick_up_and_insert->queue(connector4PreClose2);
    // connector4_pick_up_and_insert->queue(connector4PreClose3);
    // connector4_pick_up_and_insert->queue(full_open_gripper);
    // connector4_pick_up_and_insert->queue(connector4PreClose2);r4PrePush);

    // connector4_pick_up_and_insert->queue(half_open_gripper);
    // connector4_pick_up_and_insert->queue(connector4Push);
    // connector4_pick_up_and_insert->queue(connector4PrePush);
    // connector4_pick_up_and_insert->queue(connector4PreClose);
    // connector4_pick_up_and_insert->queue(full_open_gripper);
    
    // connector4_pick_up_and_insert->queue(connector4PreClose4);
    // connector4_pick_up_and_insert->queue(connector4PreClose5);

    // connector4_pick_up_and_insert->queue(full_close_gripper);
    // connector4_pick_up_and_insert->queue(full_open_gripper);
    // connector4_pick_up_and_insert->queue(full_close_gripper);
    // connector4_pick_up_and_insert->queue(full_close_gripper);
    // connector4_pick_up_and_insert->queue(full_open_gripper);

    // connector4_pick_up_and_insert->queue(connector4PreClose4);
    // connector4_pick_up_and_insert->queue(connector4PreClose);
    // connector4_pick_up_and_insert->queue(connector4PreClose2);
    // connector4_pick_up_and_insert->queue(connector4PreClose6);
    // connector4_pick_up_and_insert->queue(connector4_close_gripper);
    // connector4_pick_up_and_insert->queue(connector4PreClose2);
    // connector4_pick_up_and_insert->queue(connector4PreClose);
    
    connector4_pick_up_and_insert->queue(connector4PreInsert);
    connector4_pick_up_and_insert->queue(connector4PreInsert1);
    connector4_pick_up_and_insert->queue(connector4InitialInsert);

    connector4_pick_up_and_insert->queue(full_open_gripper);
    // connector4_pick_up_and_insert->queue(connector4PreClose7);
    // connector4_pick_up_and_insert->queue(third_open_gripper);

    // connector4_pick_up_and_insert->queue(connector4Close);
    // connector4_pick_up_and_insert->queue(connector4PreClose7);
    // connector4_pick_up_and_insert->queue(connector4Close);
    // connector4_pick_up_and_insert->queue(connector4PreClose7);
    // connector4_pick_up_and_insert->queue(connector4PreInsert1);
    // connector4_pick_up_and_insert->queue(half_open_gripper);
    // connector4_pick_up_and_insert->queue(full_open_gripper);

    // connector4_pick_up_and_insert->queue(connector4PreClose4);
    // connector4_pick_up_and_insert->queue(connector4PreClose);
    // connector4_pick_up_and_insert->queue(connector4PreClose2);
    // connector4_pick_up_and_insert->queue(connector4PreClose6);
    // connector4_pick_up_and_insert->queue(connector4_close_gripper);
    // connector4_pick_up_and_insert->queue(connector4PreClose2);
    // connector4_pick_up_and_insert->queue(connector4PreClose);
    
    // connector4_pick_up_and_insert->queue(connector4PreInsert);
    // connector4_pick_up_and_insert->queue(connector4PreInsert1);
    // connector4_pick_up_and_insert->queue(connector4InitialInsert);

    // connector4_pick_up_and_insert->queue(full_open_gripper);
    // connector4_pick_up_and_insert->queue(connector4PreClose7);
    // connector4_pick_up_and_insert->queue(third_open_gripper);

    // connector4_pick_up_and_insert->queue(connector4Close);
    // connector4_pick_up_and_insert->queue(connector4PreClose7);
    // connector4_pick_up_and_insert->queue(connector4Close);
    // connector4_pick_up_and_insert->queue(connector4PreClose7);
    connector4_pick_up_and_insert->queue(connector4PreInsert1);
    connector4_pick_up_and_insert->queue(half_open_gripper);
    connector4_pick_up_and_insert->queue(connector4Push);
    connector4_pick_up_and_insert->queue(connector4Done);


    connector2_to_tool_change->queue(preToolChange1);
    connector2_to_tool_change->queue(preToolChange2);
    connector2_to_tool_change->queue(full_close_gripper);


    // Tool change Task
    //auto tool_change_task = std::make_shared<leitungssatz::ToolChangeTask>(rob);

    //tool_change->queue(tool_change_task);

    // Task scheduler belows is commented out for testing purposes
    /*
    leitungssatz::TaskScheduler sched1;
    sched1.addTask(cable_yellow);
    sched1.addTask(cable_yellow2);
    sched1.addTask(connector1_pick_up_and_insert);
    sched1.addTask(connector2_pick_up_and_insert);
    sched1.addTask(connector2_to_tool_change);
    // sched1.addTask(tool_change); // as needed
    */

    // Simple test - just to try basic movement
    leitungssatz::TaskScheduler sched1;
    auto move_home_task = std::make_shared<leitungssatz::InterTask>(node->get_logger());
    //move_home_task->queue(moveHome);
    move_home_task->queue(full_open_gripper);
    sched1.addTask(move_home_task);  // Just try to go home first

    std::cout << "start? " << std::endl;
    std::cin >> eingabe;
    std::transform(eingabe.begin(), eingabe.end(), eingabe.begin(), ::tolower); // To lower case

    if (eingabe == "yes") {
        sched1.run();
        // cable_blue2->exec();
        // cableCheckBlack->execute();
    }



    // Start the main loop
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
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
#include <leitungssatz/Tasks/SpiralTask.hpp>
#include <leitungssatz/Tasks/GrabBuchseTask.hpp>
#include <leitungssatz/Tasks/SetCableOrientationTask.hpp>
#include <leitungssatz/Tasks/WirePushTask.hpp>
#include <leitungssatz/Tasks/CableCheckTask.hpp>
#include <leitungssatz/Tasks/ToolChangeTask.hpp>

#include <rclcpp/rclcpp.hpp>
#include <boost/algorithm/string.hpp>
#include <vector>
#include <iostream>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

bool isForceExceeded(const leitungssatz::Wrench_ptr& wrench, double force_limit) {
    return std::abs(wrench->wrenchMsg.wrench.force.x) > force_limit ||
           std::abs(wrench->wrenchMsg.wrench.force.y) > force_limit ||
           std::abs(wrench->wrenchMsg.wrench.force.z) > force_limit;
}

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("main_leitungssatz");

    // background executor (equivalent to ros::AsyncSpinner(2))
    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
    exec.add_node(node);
    std::thread spin_thread([&exec]() { exec.spin(); });

    std::string eingabe;
    leitungssatz::Gripper grip = leitungssatz::Gripper(node);

    double VEL = 1.0/2;
    double ACC = 1.0/2;
    double force_limit = -2.5;
    
    leitungssatz::Robot_ptr rob (new leitungssatz::Robot(node, grip));
    leitungssatz::Wrench_ptr wrench (new leitungssatz::Wrench(node));

    // short sleep to emulate ros::Duration(0.5).sleep();
    rclcpp::sleep_for(500ms);
    
    // Gripper Tasks 
    leitungssatz::Task_ptr grip_task_open_cable(new leitungssatz::GripperTask(rob, 45, true));
    leitungssatz::Task_ptr grip_task_pick_cable(new leitungssatz::GripperTask(rob, 0, 100, 70, false));
    leitungssatz::Task_ptr grip_task_open_connector(new leitungssatz::GripperTask(rob,72, true));
    leitungssatz::Task_ptr grip_task_pick_connector(new leitungssatz::GripperTask(rob, 0, 55, 30, false));
    leitungssatz::Task_ptr grip_task_pick_connector3(new leitungssatz::GripperTask(rob, 0, 80, 30, false));
    leitungssatz::Task_ptr grip_task_pick_connector4(new leitungssatz::GripperTask(rob, 0, 55, 30, false));

    // Material Handling Tasks (Connectors)
    leitungssatz::Task_ptr moveHome( new leitungssatz::MoveTask(rob, "home", leitungssatz::MOVEJ, VEL, ACC));
     
    leitungssatz::Task_ptr conncectorApproach1( new leitungssatz::MoveTask(rob, "connectorApproach1", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr conncectorPick1( new leitungssatz::MoveTask(rob, "connectorPick1", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr conncectorPickRetreat( new leitungssatz::MoveTask(rob, "connectorPickRetreat1", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr conncectorInsertApp1( new leitungssatz::MoveTask(rob, "connectorInsertApp1", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr conncectorInsertApp12( new leitungssatz::MoveTask(rob, "connectorInsertApp12", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr conncectorApproach2( new leitungssatz::MoveTask(rob, "connectorApproach2", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr conncectorPick2( new leitungssatz::MoveTask(rob, "connectorPick2", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr conncectorPickRetreat2( new leitungssatz::MoveTask(rob, "connectorPickRetreat2", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr conncectorInsertApp2( new leitungssatz::MoveTask(rob, "connectorInsertApp2", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr conncectorInsert2( new leitungssatz::MoveTask(rob, "connectorInsert2", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr conncectorRetreat2( new leitungssatz::MoveTask(rob, "connector2Retreat", leitungssatz::MOVEL, VEL, ACC));

    leitungssatz::Task_ptr conncectorApproach3( new leitungssatz::MoveTask(rob, "connectorApproach3", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr conncectorPick3( new leitungssatz::MoveTask(rob, "connectorPick3", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr conncectorPickRetreat3( new leitungssatz::MoveTask(rob, "connectorPickRetreat3", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr conncectorInsertApp3( new leitungssatz::MoveTask(rob, "connectorInsertApp3", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr conncectorInsert3( new leitungssatz::MoveTask(rob, "connectorInsert3", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr conncectorRetreat3( new leitungssatz::MoveTask(rob, "connector3Retreat", leitungssatz::MOVEL, VEL, ACC));

    leitungssatz::Task_ptr conncectorApproach4( new leitungssatz::MoveTask(rob, "connectorApproach4", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr conncectorPick4( new leitungssatz::MoveTask(rob, "connectorPick4", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr conncectorPickRetreat4( new leitungssatz::MoveTask(rob, "connectorPickRetreat4", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr conncectorInsertApp4( new leitungssatz::MoveTask(rob, "connectorInsertApp4", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr conncectorInsert4( new leitungssatz::MoveTask(rob, "connectorInsert4", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr conncectorRetreat4( new leitungssatz::MoveTask(rob, "connector4Retreat", leitungssatz::MOVEL, VEL, ACC));
        
    // Pick cables Tasks 
    leitungssatz::Task_ptr cableApproachYellow( new leitungssatz::MoveTask(rob, "cableApproachYellow", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr cableApproachYellow2( new leitungssatz::MoveTask(rob, "cableApproachYellow2", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr cablePickApp( new leitungssatz::MoveTask(rob, "cablePickYellow", leitungssatz::MOVEL, VEL, ACC)); 
    leitungssatz::Task_ptr cablePostPick( new leitungssatz::MoveTask(rob, "cablePostPickYellow", leitungssatz::MOVEL, VEL, ACC)); 
    
    leitungssatz::Task_ptr cableCheckOrientation( new leitungssatz::MoveTask(rob, "CAMERA_POINT_YELLOW_UPPER", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr SaveTerminalOrientation( new leitungssatz::SetCableOrientationTask(rob, node, leitungssatz::CableType::YELLOW_UPPER, "/CableOrienter2/SaveUpperTerminalImage"));
   
    leitungssatz::Task_ptr cableCheckOrientationT2( new leitungssatz::MoveTask(rob, "CAMERA_POINT_YELLOW_T2", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr SetcableOrientation( new leitungssatz::SetCableOrientationTask(rob, node, leitungssatz::CableType::YELLOW_LOWER, "/CableOrienter2/SetCableOrientation"));
    
    leitungssatz::Task_ptr spiralmove_yellow(new leitungssatz::SpiralTask(rob, node, wrench, 0.1,2.0,45,20,0.253,2,true));
    
    leitungssatz::Task_ptr insertionYellowLower(new leitungssatz::WirePush(rob,wrench,node,"yellowT1_push_close","yellowT1_push_far",2,10,10));
    leitungssatz::Task_ptr PostInsertionPoint(new leitungssatz::MoveTask(rob, "FINISH_POINT_YELLOWT1", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr ApproachConnectorYellowT2(new leitungssatz::MoveTask(rob, "APPROACH_POINT1_YELLOWT2", leitungssatz::MOVEL, VEL, ACC));    
    leitungssatz::Task_ptr SetcableOrientationUpper( new leitungssatz::SetCableOrientationTask(rob, node, leitungssatz::CableType::YELLOW_UPPER, "/CableOrienter2/SetCableOrientation"));
    leitungssatz::Task_ptr insertionYellowUpper(new leitungssatz::WirePush(rob,wrench,node,"yellowT2_push_close","yellowT2_push_far",2,10,10));

    // Move to second Cable 
    leitungssatz::Task_ptr TransitionMove( new leitungssatz::MoveTask(rob, "postInsertionCable1", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr TransitionMove2( new leitungssatz::MoveTask(rob, "postInsertionCable12", leitungssatz::MOVEL, VEL, ACC));
  
    leitungssatz::Task_ptr cable2ApproachYellow( new leitungssatz::MoveTask(rob, "cable2ApproachYellow", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr cable2ApproachYellow2( new leitungssatz::MoveTask(rob, "cable2ApproachYellow2", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr cable2PickApp( new leitungssatz::MoveTask(rob, "cable2PickYellow", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr cable2PostPick( new leitungssatz::MoveTask(rob, "cable2PostPickYellow", leitungssatz::MOVEL, VEL, ACC)); 

    leitungssatz::Task_ptr cable2CheckOrientation( new leitungssatz::MoveTask(rob, "CAMERA_POINT_YELLOW_UPPER", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr SaveTerminal2Orientation( new leitungssatz::SetCableOrientationTask(rob, node, leitungssatz::CableType::YELLOW2_UPPER, "/CableOrienter2/SaveUpperTerminalImage"));
    leitungssatz::Task_ptr cable2CheckOrientationT2( new leitungssatz::MoveTask(rob, "CAMERA_POINT_YELLOW_T2", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr Setcable2Orientation( new leitungssatz::SetCableOrientationTask(rob, node, leitungssatz::CableType::YELLOW2_LOWER, "/CableOrienter2/SetCableOrientation"));

    // Pick Cables Task Red 
    leitungssatz::Task_ptr cableApproachRed1( new leitungssatz::MoveTask(rob, "cableApproachRed1", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr cableApproachRed12( new leitungssatz::MoveTask(rob, "cableApproachRed2", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr cablePickRed( new leitungssatz::MoveTask(rob, "cablePickRed", leitungssatz::MOVEL, VEL, ACC));  
    leitungssatz::Task_ptr cablePostPickRed( new leitungssatz::MoveTask(rob, "cablePostPickRed", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr cableCheckOrientationREDT2( new leitungssatz::MoveTask(rob, "CAMERA_POINT_RED_T2", leitungssatz::MOVEL, VEL, ACC));

    // Pick Cables Task Black 
    leitungssatz::Task_ptr cableApproachBlack1( new leitungssatz::MoveTask(rob, "cableApproachblack1", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr cableApproachBlack2( new leitungssatz::MoveTask(rob, "cableApproachblack2", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr cablePickBlack( new leitungssatz::MoveTask(rob, "cablePickblack", leitungssatz::MOVEL, VEL, ACC)); 
    leitungssatz::Task_ptr cablePostPickBlack( new leitungssatz::MoveTask(rob, "cablePostPickblack", leitungssatz::MOVEL, VEL, ACC));  

    // InterTask groupings
    leitungssatz::InterTask_ptr move_home( new leitungssatz::InterTask());
    leitungssatz::InterTask_ptr connector1Grasping( new leitungssatz::InterTask());
    leitungssatz::InterTask_ptr connector2Grasping( new leitungssatz::InterTask());
    leitungssatz::InterTask_ptr connector3Grasping( new leitungssatz::InterTask());
    leitungssatz::InterTask_ptr connector4Grasping( new leitungssatz::InterTask());

    leitungssatz::InterTask_ptr cable_red( new leitungssatz::InterTask());
    leitungssatz::InterTask_ptr cable_yellow( new leitungssatz::InterTask());
    leitungssatz::InterTask_ptr cable_black( new leitungssatz::InterTask());
    leitungssatz::InterTask_ptr cable_orientation( new leitungssatz::InterTask());

    // generic gripper tasks for connector pick and insert
    leitungssatz::Task_ptr full_open_gripper(new leitungssatz::GripperTask(rob, 100, false));  
    leitungssatz::Task_ptr half_open_gripper(new leitungssatz::GripperTask(rob, 50, false));
    leitungssatz::Task_ptr third_open_gripper(new leitungssatz::GripperTask(rob, 35, false));
    leitungssatz::Task_ptr connector_close_gripper(new leitungssatz::GripperTask(rob, 0, 100, 45, false)); 

    // Connector 1 pick up and insert task
    leitungssatz::InterTask_ptr connector1_pick_up_and_insert( new leitungssatz::InterTask());

    leitungssatz::Task_ptr connector1PrePickup(new leitungssatz::MoveTask(rob, "connector1PrePickup", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr connector1PullOut(new leitungssatz::MoveTask(rob, "connector1PullOut", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr connector1HalfPull(new leitungssatz::MoveTask(rob, "conncetor1HalfPull", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr connector1PreClose(new leitungssatz::MoveTask(rob, "connector1PreClose", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr connector1Close(new leitungssatz::MoveTask(rob, "connector1Close", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr connector1Out(new leitungssatz::MoveTask(rob, "connector1Out", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr connector1PreInsert(new leitungssatz::MoveTask(rob, "connector1PreInsert", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr connector1PreInsert1(new leitungssatz::MoveTask(rob, "connector1PreInsert1", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr connector1InitialInsert(new leitungssatz::MoveTask(rob, "connector1InitialInsert", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr connector1InitialInsert1(new leitungssatz::MoveTask(rob, "connector1InitialInsert1", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr connector1Insert(new leitungssatz::MoveTask(rob, "connector1Insert", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr connector1PostInsert(new leitungssatz::MoveTask(rob, "connector1PostInsert", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr connector1PrePush(new leitungssatz::MoveTask(rob, "connector1PrePush", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr connector1Push(new leitungssatz::MoveTask(rob, "connector1Push", leitungssatz::MOVEL, VEL, ACC));

    // Connector 2 pick up and insert task
    leitungssatz::InterTask_ptr connector2_pick_up_and_insert( new leitungssatz::InterTask());

    leitungssatz::Task_ptr connector2PrePickup(new leitungssatz::MoveTask(rob, "connector2PrePickup", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr connector2PullOut(new leitungssatz::MoveTask(rob, "connector2PullOut", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr connector2Out(new leitungssatz::MoveTask(rob, "connector2Out", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr connector2PreInsert(new leitungssatz::MoveTask(rob, "connector2PreInsert", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr connector2PreInsert1(new leitungssatz::MoveTask(rob, "connector2PreInsert1", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr connector2InitialInsert(new leitungssatz::MoveTask(rob, "connector2InitialInsert", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr connector2Push(new leitungssatz::MoveTask(rob, "connector2Push", leitungssatz::MOVEL, VEL, ACC));
 
    // Tool change task
    leitungssatz::InterTask_ptr tool_change( new leitungssatz::InterTask());

    leitungssatz::Task_ptr aproachChangeGripper(new leitungssatz::MoveTask(rob, "aproachChangeGripper", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr aproachChangeGripper1(new leitungssatz::MoveTask(rob, "aproachChangeGripper1", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr aproachChangeGripper2(new leitungssatz::MoveTask(rob, "aproachChangeGripper2", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr aproachChangeGripper3(new leitungssatz::MoveTask(rob, "aproachChangeGripper3", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr preChangeGripper(new leitungssatz::MoveTask(rob, "preChangeGripper", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr preChangeGripper1(new leitungssatz::MoveTask(rob, "preChangeGripper1", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr changeGripperUp(new leitungssatz::MoveTask(rob, "changeGripperUp", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr changeGripperDown(new leitungssatz::MoveTask(rob, "changeGripperDown", leitungssatz::MOVEL, VEL, ACC));
    leitungssatz::Task_ptr changeGripperHalfOut(new leitungssatz::MoveTask(rob, "changeGripperHalfOut", leitungssatz::MOVEL, VEL, ACC));

    move_home->queue(moveHome);
    connector1Grasping->queue(grip_task_open_connector);
    connector1Grasping->queue(conncectorApproach1);
    connector1Grasping->queue(conncectorPick1);
    connector1Grasping->queue(grip_task_pick_connector);
    connector1Grasping->queue(conncectorPickRetreat);
    connector1Grasping->queue(conncectorInsertApp1);
    connector1Grasping->queue(conncectorInsertApp12);

    connector2Grasping->queue(grip_task_open_connector);
    connector2Grasping->queue(conncectorApproach2);
    connector2Grasping->queue(conncectorPick2);
    connector2Grasping->queue(grip_task_pick_connector);
    connector2Grasping->queue(conncectorPickRetreat2);
    connector2Grasping->queue(conncectorInsertApp2);
    connector2Grasping->queue(conncectorInsert2);
    connector2Grasping->queue(grip_task_open_connector);
    connector2Grasping->queue(conncectorRetreat2);

    connector3Grasping->queue(grip_task_open_connector);
    connector3Grasping->queue(conncectorApproach3);
    connector3Grasping->queue(conncectorPick3);
    connector3Grasping->queue(grip_task_pick_connector3);
    connector3Grasping->queue(conncectorPickRetreat3);
    connector3Grasping->queue(conncectorInsertApp3);
    connector3Grasping->queue(conncectorInsert3);
    connector3Grasping->queue(grip_task_open_connector);
    connector3Grasping->queue(conncectorRetreat3);

    connector4Grasping->queue(grip_task_open_connector);
    connector4Grasping->queue(conncectorApproach4);
    connector4Grasping->queue(conncectorPick4);
    connector4Grasping->queue(grip_task_pick_connector4);
    connector4Grasping->queue(conncectorPickRetreat4);
    connector4Grasping->queue(conncectorInsertApp4);
    connector4Grasping->queue(conncectorInsert4);
    connector4Grasping->queue(grip_task_open_connector);
    connector4Grasping->queue(conncectorRetreat4);

    cable_red ->queue(grip_task_open_cable);
    cable_red ->queue(cableApproachRed1);
    cable_red ->queue(cableApproachRed12);      
    cable_red ->queue(cablePickRed);
    cable_red ->queue(grip_task_pick_cable);
    cable_red ->queue(cablePostPickRed);

    // First Yellow Cable 
    cable_yellow ->queue(grip_task_open_cable);
    cable_yellow ->queue(cableApproachYellow);
    cable_yellow ->queue(cableApproachYellow2);
    cable_yellow ->queue(cablePickApp);
    cable_yellow ->queue(grip_task_pick_cable);
    cable_yellow ->queue(cablePostPick);
    cable_yellow ->queue(cableCheckOrientation);
    cable_yellow ->queue(SaveTerminalOrientation);
    cable_yellow ->queue(cableCheckOrientationT2);
    cable_yellow ->queue(SetcableOrientation);
    cable_yellow ->queue(insertionYellowLower);

    RCLCPP_INFO(node->get_logger(), "Wrench Force in Y %f", wrench->wrenchMsg.wrench.force.y);
    
    // if (isForceExceeded(wrench, force_limit)) {
    //     cable_yellow->queue(spiralmove_yellow);
    // }
    
    cable_yellow ->queue(grip_task_open_cable);
    cable_yellow ->queue(PostInsertionPoint);

    cable_yellow ->queue(grip_task_pick_cable);
    cable_yellow ->queue(SetcableOrientationUpper);
    // cable_yellow ->queue(insertionYellowUpper); 

    // Second Yellow Cable
    cable_yellow ->queue(grip_task_open_cable);
    cable_yellow ->queue(TransitionMove);
    cable_yellow ->queue(TransitionMove2);
    
    cable_yellow ->queue(cable2ApproachYellow);
    cable_yellow ->queue(cable2ApproachYellow2);
    cable_yellow ->queue(cable2PickApp);
    cable_yellow ->queue(grip_task_pick_cable);
    cable_yellow ->queue(cable2PostPick);
    cable_yellow ->queue(cable2CheckOrientation);
    cable_yellow ->queue(SaveTerminal2Orientation);
    cable_yellow ->queue(cable2CheckOrientationT2);
    cable_yellow ->queue(Setcable2Orientation); 

    cable_black ->queue(grip_task_open_cable);
    cable_black ->queue(cableApproachBlack1);
    cable_black ->queue(cableApproachBlack2);
    cable_black ->queue(cablePickBlack);
    cable_black ->queue(grip_task_pick_cable);

    connector1_pick_up_and_insert->queue(full_open_gripper);
    connector1_pick_up_and_insert->queue(moveHome);
    connector1_pick_up_and_insert->queue(connector1PrePickup);
    connector1_pick_up_and_insert->queue(connector1PullOut);
    connector1_pick_up_and_insert->queue(connector_close_gripper);

    connector1_pick_up_and_insert->queue(connector1HalfPull);
    connector1_pick_up_and_insert->queue(full_open_gripper);
    
    connector1_pick_up_and_insert->queue(connector1PrePickup);
    connector1_pick_up_and_insert->queue(connector_close_gripper);
    connector1_pick_up_and_insert->queue(connector1PreClose);
    connector1_pick_up_and_insert->queue(connector1Close);

    connector1_pick_up_and_insert->queue(connector1PreClose);
    connector1_pick_up_and_insert->queue(connector1Close);
    
    connector1_pick_up_and_insert->queue(connector1PrePickup);
    connector1_pick_up_and_insert->queue(full_open_gripper);

    connector1_pick_up_and_insert->queue(connector1HalfPull);
    connector1_pick_up_and_insert->queue(connector_close_gripper);
    connector1_pick_up_and_insert->queue(connector1Out);
    connector1_pick_up_and_insert->queue(connector1PreInsert);
    connector1_pick_up_and_insert->queue(connector1PreInsert1);

    connector1_pick_up_and_insert->queue(connector1InitialInsert);
    connector1_pick_up_and_insert->queue(connector1InitialInsert1);
    connector1_pick_up_and_insert->queue(full_open_gripper);

    connector1_pick_up_and_insert->queue(connector1PrePush);
    connector1_pick_up_and_insert->queue(half_open_gripper);
    connector1_pick_up_and_insert->queue(connector1Push);

    connector1_pick_up_and_insert->queue(moveHome);

    connector2_pick_up_and_insert->queue(full_open_gripper);
    connector2_pick_up_and_insert->queue(moveHome);
    connector2_pick_up_and_insert->queue(connector2PrePickup);
    connector2_pick_up_and_insert->queue(connector2PullOut);
    connector2_pick_up_and_insert->queue(connector_close_gripper);
    connector2_pick_up_and_insert->queue(connector2Out);

    connector2_pick_up_and_insert->queue(connector2PreInsert);
    connector2_pick_up_and_insert->queue(connector2PreInsert1);

    connector2_pick_up_and_insert->queue(connector2InitialInsert);
    connector2_pick_up_and_insert->queue(full_open_gripper);

    connector2_pick_up_and_insert->queue(connector2PreInsert1);
    connector2_pick_up_and_insert->queue(third_open_gripper);
    connector2_pick_up_and_insert->queue(connector2Push);

    connector2_pick_up_and_insert->queue(moveHome);

    // Tool change Task
    leitungssatz::Task_ptr tool_change_task(new leitungssatz::ToolChangeTask(rob/*, node*/));

    tool_change->queue(moveHome);

    leitungssatz::TaskScheduler sched1;
    sched1.addTask(move_home);   
    sched1.addTask(cable_yellow);
    sched1.addTask(connector1_pick_up_and_insert);
    sched1.addTask(connector2_pick_up_and_insert);

    std::cout << "start? " << std::endl;
    std::cin >> eingabe;
    boost::algorithm::to_lower(eingabe);
    
    if(eingabe == "yes")
    {
        sched1.run();
    }

    // stop executor and cleanup
    exec.cancel();

}
/**
 * @file point_recording.cpp
 * @authors Adrian Müller (adrian.mueller@study.thws.de), 
 *          Maximilian Hornauer (maximilian.hornauer@study.thws.de),
 *          Usama Ali (usama.ali@study.thws.de)
 * @brief Program for recording points by using freedrive
 * @version 0.2
 * @date 2025-01-01
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <filesystem>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <fstream>
#include <termios.h>

#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <boost/array.hpp>

#include <leitungssatz/srv/add_tf2.hpp>
#include <leitungssatz_interfaces/srv/set_freedrive.hpp>
#include <leitungssatz_interfaces/srv/set_cart_target.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <leitungssatz_interfaces/srv/start_jog.hpp>
#include <leitungssatz_interfaces/msg/jog_control.hpp>
#include <leitungssatz_interfaces/srv/set_force_target.hpp>
#include <leitungssatz_interfaces/srv/set_gripper.hpp>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <cmath>


std::array<int64_t, 6> free_axes = {1, 1, 1, 1, 1, 1};

geometry_msgs::msg::TransformStamped tcp_pose;
rclcpp::Client<leitungssatz_interfaces::srv::SetFreedrive>::SharedPtr client_f;
rclcpp::TimerBase::SharedPtr inactivity_timer;
rclcpp::Node::SharedPtr node;

void main_menu();
void free_drive_menu();
void con_menu();
void jog_menu(int frame, double speed, double step);
leitungssatz_interfaces::srv::SetCartTarget::Request::SharedPtr align_robot_to_base_axis(std::string axis);
leitungssatz_interfaces::srv::SetCartTarget::Request::SharedPtr align_robot_to_base_axis_90();
void stopFreedrive();

void tcpCallback(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
{
    tcp_pose = *msg;
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    node = std::make_shared<rclcpp::Node>("point_recorder");

    auto client_g = node->create_client<leitungssatz_interfaces::srv::SetGripper>("/ur_hardware_interface/robotiq/set_gripper");
    client_f = node->create_client<leitungssatz_interfaces::srv::SetFreedrive>("/ur_hardware_interface/set_freedrive");
    auto client_fm = node->create_client<leitungssatz_interfaces::srv::SetForceTarget>("/ur_hardware_interface/set_force_mode");
    auto client_t = node->create_client<leitungssatz_interfaces::srv::SetCartTarget>("/ur_hardware_interface/set_cart_target");
    auto client_tf = node->create_client<leitungssatz::srv::AddTf2>("/store_tf");
    auto jog_client = node->create_client<leitungssatz_interfaces::srv::StartJog>("/ur_hardware_interface/start_jog");
    
    auto sub = node->create_subscription<geometry_msgs::msg::TransformStamped>(
        "/ur_hardware_interface/tcp_pose", 1000, tcpCallback);

    auto jog_control_pub = node->create_publisher<leitungssatz_interfaces::msg::JogControl>("/jog_control", 1000);

    inactivity_timer = node->create_wall_timer(std::chrono::seconds(30), stopFreedrive);
    
    auto tfBuffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    auto tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

    // Start spinning in background thread
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread executor_thread([&executor]() {
        executor.spin();
    });

    auto addTF_srv = std::make_shared<leitungssatz::srv::AddTf2::Request>();
    auto freedrive_srv = std::make_shared<leitungssatz_interfaces::srv::SetFreedrive::Request>();
    freedrive_srv->io = true;
    freedrive_srv->free_axes = free_axes;
    auto target_srv = std::make_shared<leitungssatz_interfaces::srv::SetCartTarget::Request>();
    target_srv->mode = 1;
    target_srv->speed = 0.25;
    target_srv->acceleration = 0.25;
    target_srv->asynchronous = false;
    auto force_srv = std::make_shared<leitungssatz_interfaces::srv::SetForceTarget::Request>();
    force_srv->type = 2;
    force_srv->limits = {1,1,1,1,1,1};
    auto gripper_srv = std::make_shared<leitungssatz_interfaces::srv::SetGripper::Request>();
    gripper_srv->force = 0;
    gripper_srv->asynchronous = false;
    gripper_srv->speed = 50;

    int main_choice;
    bool relation;
    double offset;
    std::string point_ID;
    std::string parent_Name;

    // Wait for services to be available
    // if (!client_f->wait_for_service(std::chrono::seconds(10))) {
    //     RCLCPP_ERROR(node->get_logger(), "Service not available");
    //     return 1;
    // }

    auto freedrive_future = client_f->async_send_request(freedrive_srv);

    do{
        main_menu();
        std::cin >> main_choice;

        switch (main_choice)
        {
        case 1:
            {
                std::cout << "Punktname: ";
                std::cin >> point_ID;
                std::cout << "Add z Offset ? [mm] ";
                std::cin >> offset;
                std::cout << "Is this Pose in relation to a target ? [1/0] ";
                std::cin >> relation;
                addTF_srv->pose = tcp_pose;
                addTF_srv->pose.header.stamp = node->get_clock()->now();
                addTF_srv->pose.header.frame_id = "base";
                addTF_srv->pose.child_frame_id = point_ID;
                addTF_srv->relative = relation;
                if(relation)
                {
                    std::cout << "Which target? ";
                    std::cin >> parent_Name;
                    addTF_srv->pose.header.frame_id = parent_Name;
                }
                auto tf_future = client_tf->async_send_request(addTF_srv);
                if(offset != 0) {
                    auto offset_srv = std::make_shared<leitungssatz::srv::AddTf2::Request>();
                    offset_srv->pose.transform = geometry_msgs::msg::Transform();
                    offset_srv->pose.transform.translation.z = -1 * offset / 1000;
                    offset_srv->pose.transform.rotation.w = 1;
                    offset_srv->pose.header.frame_id = point_ID;
                    offset_srv->pose.child_frame_id = point_ID + "_app";
                    offset_srv->relative = false;
                    auto offset_future = client_tf->async_send_request(offset_srv);
                }
                break;
            }
        case 2:
            {
                free_drive_menu();

                int axis;
                std::cin >> axis;

                freedrive_srv->io = false;
                auto stop_future = client_f->async_send_request(freedrive_srv);

                if(axis < 4 && axis > 0)
                {
                    freedrive_srv->feature = axis-1;
                    if(axis == 3) {
                        try {
                            auto transform = tfBuffer->lookupTransform("base", "task_board", tf2::TimePointZero);
                            freedrive_srv->custom_frame = transform.transform;
                        } catch (tf2::TransformException &ex) {
                            RCLCPP_WARN(node->get_logger(), "%s", ex.what());
                        }
                    }
                }

                freedrive_srv->io = true;
                freedrive_srv->free_axes = free_axes;
                auto start_future = client_f->async_send_request(freedrive_srv);
                break;
            }
        case 3:
            {
                con_menu();
                int con;
                std::cin >> con;
                switch (con)
                {
                case 0:
                    break;
                case 1:
                    std::cout << "Put in values for x,y,z,r,p,y (1/0)" << std::endl;
                    for(int i=0;i<6;i++){std::cin >> free_axes[i];}
                    break;
                case 2:
                    std::cout << "Put in values for x,y,z (1/0)" << std::endl;
                    for(int i=0;i<3;i++){std::cin >> free_axes[i];}
                    break;
                case 3:
                    std::cout << "Put in values for r,p,y (1/0)" << std::endl;
                    for(int i=3;i<6;i++){std::cin >> free_axes[i];}
                    break;
                
                default:
                    break;
                }
                freedrive_srv->io = false;
                auto stop_future = client_f->async_send_request(freedrive_srv);
                freedrive_srv->io = true;
                freedrive_srv->free_axes = free_axes;
                auto start_future = client_f->async_send_request(freedrive_srv);
                break;
            }
        case 4:
            {  
                freedrive_srv->io = false;
                auto stop_future = client_f->async_send_request(freedrive_srv);
                
                bool running = true;
                int feature = 1;
                double step = 0.001;
                double stepIncrement = 0.001;

                int kfd = 0;
                struct termios cooked, raw;
                tcgetattr(kfd, &cooked);
                memcpy(&raw, &cooked, sizeof(struct termios));
                
                raw.c_lflag &=~ (ICANON | ECHO);
                raw.c_cc[VEOL] = 1;
                raw.c_cc[VEOF] = 2;
                tcsetattr(kfd, TCSANOW, &raw);

                jog_menu(feature, step, stepIncrement);

                while(rclcpp::ok() && running){
                    Eigen::Matrix4d deltaP = Eigen::Matrix4d::Identity();
                    
                    char c;
                    if(read(kfd, &c, 1) < 0)
                    {
                      c = '0';
                    }
                    RCLCPP_INFO(node->get_logger(), "value: 0x%02X", c);
                    
                    switch (c)
                    {
                    case '0':
                        tcsetattr(kfd, TCSANOW, &cooked);
                        running = false;
                        break;
                    case 'a':
                        deltaP(0,3) += step;
                        break;
                    case 'd':
                        deltaP(0,3) -= step;
                        break;
                    case 's':
                        deltaP(1,3) -= step;
                        break;
                    case 'w':
                        deltaP(1,3) += step;
                        break;
                    case 'e':
                        deltaP(2,3) -= step;
                        break;
                    case 'q':
                        deltaP(2,3) += step;
                        break;        
                    case 'j':
                        deltaP.block<3,3>(0,0) = Eigen::AngleAxisd(-step * M_PI, Eigen::Vector3d::UnitX()).matrix();
                        break;
                    case 'l':
                        deltaP.block<3,3>(0,0) = Eigen::AngleAxisd(step * M_PI, Eigen::Vector3d::UnitX()).matrix();
                        break;
                    case 'k':
                        deltaP.block<3,3>(0,0) = Eigen::AngleAxisd(-step * M_PI, Eigen::Vector3d::UnitY()).matrix();
                        break;
                    case 'i':
                        deltaP.block<3,3>(0,0) = Eigen::AngleAxisd(step * M_PI, Eigen::Vector3d::UnitY()).matrix();
                        break;
                    case 'o':
                        deltaP.block<3,3>(0,0) = Eigen::AngleAxisd(-step * M_PI, Eigen::Vector3d::UnitZ()).matrix();
                        break;
                    case 'u':
                        deltaP.block<3,3>(0,0) = Eigen::AngleAxisd(step * M_PI, Eigen::Vector3d::UnitZ()).matrix();
                        break;
                    case 'n':
                        step += stepIncrement;
                        break;
                    case 'm':
                        step -= stepIncrement;
                        break;
                    case '1':
                        stepIncrement = 0.0001;
                        break;
                    case '2':
                        stepIncrement = 0.0005;
                        break;
                    case '3':
                        stepIncrement = 0.001;
                        break;
                    case '4':
                        stepIncrement = 0.005;
                        break;
                    case 'b':
                        feature = (feature+1) %2;
                        break;
                    default:
                        break;
                    }
                    
                    geometry_msgs::msg::Transform null;
                    null.rotation.w = 0;
                    null.rotation.x = 0;
                    null.rotation.y = 0;
                    null.rotation.z = 0;
                    
                    Eigen::Isometry3d robotPose_Iso;
                    robotPose_Iso = tf2::transformToEigen(tcp_pose.transform);
                    Eigen::Matrix4d robotPose_Mat = robotPose_Iso.matrix();
                    Eigen::Matrix4d resPose; 
                    
                    if(feature == 0){
                        resPose = robotPose_Mat;
                        resPose(0,3) += deltaP(0, 3);
                        resPose(1,3) += deltaP(1, 3);
                        resPose(2,3) += deltaP(2, 3);
                        resPose.block<3,3>(0,0) = deltaP.block<3,3>(0,0) * resPose.block<3,3>(0,0);
                    }
                    else if(feature == 1){
                        resPose = robotPose_Mat * deltaP;
                    }
                    
                    auto transform_stamped = tf2::eigenToTransform(Eigen::Isometry3d(resPose));
                    target_srv->cartesian_goal = transform_stamped.transform;
                    auto move_future = client_t->async_send_request(target_srv);
                    
                    jog_menu(feature, step, stepIncrement);
                }
                
                force_srv->io = false;
                auto force_future = client_fm->async_send_request(force_srv);
                std::this_thread::sleep_for(std::chrono::seconds(1));
                freedrive_srv->io = true;
                auto restart_future = client_f->async_send_request(freedrive_srv);
                break;
            }
        case 5:
        {
            freedrive_srv->io = false;
            auto stop_future = client_f->async_send_request(freedrive_srv);

            system("clear");
            std::cout << "------------------------" << std::endl;
            std::cout << "Move to TF" << std::endl;
            std::cout << "------------------------" << std::endl;
            std::cout << "Please put name of target frame in" << std::endl;
            std::string tf;
            std::cin >> tf;
            try{
                auto transform = tfBuffer->lookupTransform("base", tf, tf2::TimePointZero);
                target_srv->cartesian_goal = transform.transform;
            }
            catch(tf2::TransformException &ex)
            {
                RCLCPP_WARN(node->get_logger(), "%s", ex.what());
                std::this_thread::sleep_for(std::chrono::seconds(2));
                freedrive_srv->io = true;
                auto restart_future = client_f->async_send_request(freedrive_srv);
                break;
            }
            
            // bool execute = true;
            // if(execute) {
            //     auto move_future = client_t->async_send_request(target_srv);
            // }
            // Above if statement is redundant, kept for possible future use

            auto move_future = client_t->async_send_request(target_srv);

            freedrive_srv->io = true;
            auto restart_future = client_f->async_send_request(freedrive_srv);
            break;
        }
        case 6:
        {
            system("clear");
            std::cout << "------------------------" << std::endl;
            std::cout << "Gripper" << std::endl;
            std::cout << "------------------------" << std::endl;
            std::cout << "1 - Gripper Open" << std::endl;
            std::cout << "2 - Gripper Close" << std::endl;
            std::cout << "0 - break" << std::endl;
            std::cout << "------------------------" << std::endl;
            int input;
            std::cin >> input;

            if (std::cin.fail()) {
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                input = -1;
                std::cout << "Invalid input. Please choose a valid option: - 1 - 2 - 0 -" << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(3));
            }

            switch (input)
            {
            case 0:
                break;
            case 1:
                gripper_srv->position = 100;
                gripper_srv->force = 50;
                break;
            case 2:
                gripper_srv->position = 0;
                gripper_srv->force = 100;
                break;
            
            default:
                break;
            }

            auto gripper_future = client_g->async_send_request(gripper_srv);
            break;
        }
        case 7:
        {
            freedrive_srv->io = false;
            auto stop_future = client_f->async_send_request(freedrive_srv);

            system("clear");
            std::cout << "------------------------" << std::endl;
            std::cout << "Align to axis" << std::endl;
            std::cout << "------------------------" << std::endl;
            std::cout << "Please enter the base axis you want the corresponding TCP axis to align to (x,y,z,-x,-y,-z): ";
            std::string input;
            std::cin >> input;
            if (input != "x" && input != "y" && input != "z" && input != "-x" && input != "-y" && input != "-z"){
                freedrive_srv->io = true;
                auto restart_future = client_f->async_send_request(freedrive_srv);
                break;
            }
            auto target_position = align_robot_to_base_axis(input);
            auto move_future = client_t->async_send_request(target_position);

            freedrive_srv->io = true;
            auto restart_future = client_f->async_send_request(freedrive_srv);
            break;
        }
        case 8:
        {
            freedrive_srv->io = false;
            auto stop_future = client_f->async_send_request(freedrive_srv);

            system("clear");
            std::cout << "------------------------" << std::endl;
            std::cout << "Align 90 °" << std::endl;
            std::cout << "------------------------" << std::endl;
            std::cout << "Start (yes/no): ";
            std::string input;
            std::cin >> input;
            if (input != "yes" && input != "y"){
                freedrive_srv->io = true;
                auto restart_future = client_f->async_send_request(freedrive_srv);
                break;
            }

            auto target_position = align_robot_to_base_axis_90();
            auto move_future = client_t->async_send_request(target_position);

            freedrive_srv->io = true;
            auto restart_future = client_f->async_send_request(freedrive_srv);
            break;
        }
        case 9:
        {
            inactivity_timer->cancel();
            inactivity_timer = node->create_wall_timer(std::chrono::seconds(30), stopFreedrive);
            freedrive_srv->io = true;
            auto restart_future = client_f->async_send_request(freedrive_srv);
            break;
        }
        default:
            break;
        }

    }while(rclcpp::ok() && main_choice != 0);

    freedrive_srv->io = false;
    auto final_stop = client_f->async_send_request(freedrive_srv);
    system("clear");

    executor.cancel();
    executor_thread.join();
    rclcpp::shutdown();
    return 0;
}

void main_menu()
{   
    system("clear");
    std::cout << "------------------------" << std::endl;
    std::cout << "Main Menu" << std::endl;
    std::cout << "------------------------" << std::endl;
    std::cout << "Please make your selection" << std::endl;
    std::cout << "1 - save point" << std::endl;
    std::cout << "2 - set axis" << std::endl;
    std::cout << "3 - set constrains" << std::endl;
    std::cout << "4 - move remotely" << std::endl;
    std::cout << "5 - move to tf" << std::endl;
    std::cout << "6 - move Gripper" << std::endl;
    std::cout << "7 - align to axis" << std::endl;
    std::cout << "8 - align 90°" << std::endl;
    std::cout << "9 - reactivate freedrive" << std::endl;
    std::cout << "0 - Quit" << std::endl;
    std::cout << "------------------------" << std::endl;
    std::cout << "Current Position: " << std::endl;
    std::cout << "\rX: " << tcp_pose.transform.translation.x << "\t Y: " << tcp_pose.transform.translation.y << "\t Z: " << tcp_pose.transform.translation.z << std::endl; 
    std::cout << "\rRX: " << tcp_pose.transform.rotation.x << "\t RY: " << tcp_pose.transform.rotation.y << "\t RZ: "<< tcp_pose.transform.rotation.z << "\t RW: "<< tcp_pose.transform.rotation.w << std::endl;
    std::cout << std::endl;
    std::cout << "Current Constrains: ";
    for(int i=0; i<6; i++){std::cout << free_axes[i] << ", ";}
    std::cout << std::endl;

    std::cout << "------------------------" << std::endl;
    std::cout << "Selection: ";

    if (inactivity_timer) {
        inactivity_timer->cancel();
        inactivity_timer = node->create_wall_timer(std::chrono::seconds(30), stopFreedrive);
    }
}

void free_drive_menu()
{
    system("clear");
    std::cout << "------------------------" << std::endl;
    std::cout << "Freedrive Frame" << std::endl;
    std::cout << "------------------------" << std::endl;
    std::cout << "1 - Base" << std::endl;
    std::cout << "2 - TCP" << std::endl;
    std::cout << "3 - Taskboard" << std::endl;
    std::cout << "0 - back" << std::endl;
    std::cout << "------------------------" << std::endl;
    std::cout << "Selection: ";
}

void con_menu()
{
    system("clear");
    std::cout << "------------------------" << std::endl;
    std::cout << "Freedrive Constrains" << std::endl;
    std::cout << "------------------------" << std::endl;
    std::cout << "Please choose the constrains" << std::endl;
    std::cout << "1 - all" << std::endl;
    std::cout << "2 - position" << std::endl;
    std::cout << "3 - orientaion" << std::endl;
    std::cout << "0 - back" << std::endl;
    std::cout << "------------------------" << std::endl;
    std::cout << "Selection: ";
}

void jog_menu(int feature, double step, double stepIncrement)
{
    std::vector<std::string> frames({"Base", "Tool", "Task Board"});
    system("clear");
    std::cout << "------------------------" << std::endl;
    std::cout << "\rRobot remote control" << std::endl << std::endl;
    std::cout << "------------------------" << std::endl;
    std::cout << "\rControls:" << std::endl;
    std::cout << "\r\tTranslation:" << std::endl;
    std::cout << "\r\t\tX: + a   - d" << std::endl;
    std::cout << "\r\t\tY: + w   - s" << std::endl;
    std::cout << "\r\t\tZ: + q   - e" << std::endl;
    std::cout << "\r\tRotation:" << std::endl;
    std::cout << "\r\t\tX: + j   - l" << std::endl;
    std::cout << "\r\t\tY: + i   - k" << std::endl;
    std::cout << "\r\t\tZ: + u   - o" << std::endl << std::endl;
    std::cout << "\r\tStep-Increment: 0.1mm:1, 0.5mm:2, 1mm:3, 5mm:4" << std::endl;
    std::cout << "\r\tStep: +Inc n, -Inc m" << std::endl;
    std::cout << "\r\tSwitch Frame: b" << std::endl;
    std::cout << "\r\tQuit: 0" << std::endl;
    std::cout << "------------------------" << std::endl;
    std::cout << "\rX: " << tcp_pose.transform.translation.x << "\t Y: " << tcp_pose.transform.translation.y << "\t Z: " << tcp_pose.transform.translation.z << std::endl <<"\rRX: " << tcp_pose.transform.rotation.x << "\t RY: " << tcp_pose.transform.rotation.y << "\t RZ: "<< tcp_pose.transform.rotation.z << "\t RW: "<< tcp_pose.transform.rotation.w << std::endl;
    std::cout << "\rStep: " << step << "\tStep-Increment: "<< stepIncrement << std::endl;
    std::cout << "\rFrame: " << frames[feature] << std::endl;
    std::cout << "------------------------" << std::endl;
}

leitungssatz_interfaces::srv::SetCartTarget::Request::SharedPtr align_robot_to_base_axis(std::string axis){
    
    auto target_srv = std::make_shared<leitungssatz_interfaces::srv::SetCartTarget::Request>();
    target_srv->mode = 1;
    target_srv->speed = 0.25;
    target_srv->acceleration = 0.25;
    target_srv->asynchronous = false;

    geometry_msgs::msg::Transform transform = tcp_pose.transform;
    tf2::Quaternion q(transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    if(axis == "x")
        pitch = yaw = 0;
    else if(axis == "y")
        roll = yaw = 0;
    else if(axis == "z")
        roll = pitch = 0;
    else if(axis == "-z")
        {roll = M_PI; pitch = 0;}
    else if(axis == "-x")
        {pitch = 0; yaw = M_PI;}
    else if(axis == "-y")
        {roll = M_PI; yaw = 0;}

    q.setRPY(roll, pitch, yaw);
    q.normalize();

    transform.rotation.x = q[0];
    transform.rotation.y = q[1];
    transform.rotation.z = q[2];
    transform.rotation.w = q[3];

    target_srv->cartesian_goal = transform;
    return target_srv;
}

leitungssatz_interfaces::srv::SetCartTarget::Request::SharedPtr align_robot_to_base_axis_90(){
    
    auto target_srv = std::make_shared<leitungssatz_interfaces::srv::SetCartTarget::Request>();
    target_srv->mode = 1;
    target_srv->speed = 0.25;
    target_srv->acceleration = 0.25;
    target_srv->asynchronous = false;

    geometry_msgs::msg::Transform transform = tcp_pose.transform;
    tf2::Quaternion q(transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    roll = round(roll / (M_PI / 2)) * (M_PI / 2); 
    pitch = round(pitch / (M_PI / 2)) * (M_PI / 2); 
    yaw = round(yaw / (M_PI / 2)) * (M_PI / 2); 

    q.setRPY(roll, pitch, yaw);
    q.normalize();

    transform.rotation.x = q[0];
    transform.rotation.y = q[1];
    transform.rotation.z = q[2];
    transform.rotation.w = q[3];

    target_srv->cartesian_goal = transform;
    return target_srv;
}

void stopFreedrive(){
    auto freedrive_srv = std::make_shared<leitungssatz_interfaces::srv::SetFreedrive::Request>();
    freedrive_srv->io = false;
    freedrive_srv->free_axes = free_axes;
    auto future = client_f->async_send_request(freedrive_srv);
    if (inactivity_timer) {
        inactivity_timer->cancel();
    }
}
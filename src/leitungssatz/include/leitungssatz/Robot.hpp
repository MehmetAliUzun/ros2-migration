#pragma once

#include "Main.hpp"
#include "Gripper.hpp"

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Geometry>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// ROS2 service types - path_entry and jog_control are not defined in the kopro drivers package
#include <leitungssatz_interfaces/srv/set_cart_target.hpp>
#include <leitungssatz_interfaces/srv/set_trajectory.hpp>
#include <leitungssatz_interfaces/srv/set_joint_target.hpp>
#include <leitungssatz_interfaces/srv/set_contact_target.hpp>
#include <leitungssatz_interfaces/srv/start_jog.hpp>
#include <leitungssatz_interfaces/srv/set_speed_slider.hpp>
#include <leitungssatz_interfaces/srv/set_clip_gun.hpp>
// #include <leitungssatz_interfaces/srv/path_entry.hpp>
// #include <leitungssatz_interfaces/srv/jog_control.hpp>
#include <leitungssatz_interfaces/srv/log.hpp>
#include <leitungssatz_interfaces/srv/get_time_stamp.hpp>
#include <leitungssatz_interfaces/srv/set_payload.hpp>
#include <leitungssatz_interfaces/srv/set_rel_cart_target.hpp>

namespace leitungssatz {

class Robot
{
private:
    std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
    std::shared_ptr<tf2_ros::TransformListener> tfListener_;

    const double vel_multi_ = 1;
    const double acc_multi_ = 1;

    leitungssatz_interfaces::srv::SetTrajectory::Request trajectory_request_; // <-- Add this line
    leitungssatz_interfaces::srv::SetTrajectory::Response trajectory_response_; // <-- Add this line

    //leitungssatz_interfaces::srv::SetTrajectory trajectory;
    std::shared_ptr<leitungssatz_interfaces::srv::SetTrajectory::Request> trajectory;


    // Service clients
    rclcpp::Client<leitungssatz_interfaces::srv::SetCartTarget>::SharedPtr cart_move_;
    rclcpp::Client<leitungssatz_interfaces::srv::SetTrajectory>::SharedPtr path_move_;
    rclcpp::Client<leitungssatz_interfaces::srv::SetJointTarget>::SharedPtr joint_move_;
    rclcpp::Client<leitungssatz_interfaces::srv::SetContactTarget>::SharedPtr contact_move_;
    rclcpp::Client<leitungssatz_interfaces::srv::SetTrajectory>::SharedPtr move_trajectory_;
    rclcpp::Client<leitungssatz_interfaces::srv::Log>::SharedPtr logging_;
    rclcpp::Client<leitungssatz_interfaces::srv::GetTimeStamp>::SharedPtr robot_time_;
    rclcpp::Client<leitungssatz_interfaces::srv::SetSpeedSlider>::SharedPtr _dashboard_speed;
    rclcpp::Client<leitungssatz_interfaces::srv::SetPayload>::SharedPtr _set_payload;
    rclcpp::Client<leitungssatz_interfaces::srv::SetClipGun>::SharedPtr set_clip_gun_;
    rclcpp::Client<leitungssatz_interfaces::srv::SetRelCartTarget>::SharedPtr rel_cart_move_;

          /*Below services are not defined anywhere
      rclcpp::Client<SomeSrvType>::SharedPtr triangle_detection_;
      rclcpp::Client<SomeSrvType>::SharedPtr taskboard_detection_;
      rclcpp::Client<SomeSrvType>::SharedPtr touch_detection_;
      rclcpp::Client<SomeSrvType>::SharedPtr finish_detection_;
        */

        // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr tcp_position_sub_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;

    geometry_msgs::msg::WrenchStamped wrench_;
    std::shared_ptr<rclcpp::Node> node_;

public:
    Robot(std::shared_ptr<rclcpp::Node> node, Gripper& gripper);

    bool init();
    Gripper _gripper;
    geometry_msgs::msg::TransformStamped tcp_position_;
    void tcp_positon_callback(const geometry_msgs::msg::TransformStamped::SharedPtr msg);
    void wrench_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);

    leitungssatz_interfaces::srv::SetCartTarget cart_target(std::string target, int mode = 2, double vel = 1, double acc = 1);
    leitungssatz_interfaces::srv::SetContactTarget contact_target(array6d speed = {0,0,-0.1,0,0,0}, double acc = 0.5);
    leitungssatz_interfaces::srv::SetJointTarget joint_target(array6d target, double vel = 0.5, double acc = 0.5);

    void add_cart_Path_Entry(std::string src, std::string target, int move_type = 1, double blend = 0, double vel = 0.5, double acc = 0.5);
    void add_joint_Path_Entry(array6d target, int move_type = 0, double blend = 0, double vel = 0.5, double acc = 0.5);

    //This function below changed a lot in the definition. Could be error prone. Check later.
    leitungssatz_interfaces::srv::SetCartTarget::Request::SharedPtr 
    createCartTarget(std::string src, std::string target, double vel, double acc, int move_type);

    bool call_cart_target(std::shared_ptr<leitungssatz_interfaces::srv::SetCartTarget::Request> request);
    bool call_joint_target(std::shared_ptr<leitungssatz_interfaces::srv::SetJointTarget::Request> request);
    bool call_contact_target(std::shared_ptr<leitungssatz_interfaces::srv::SetContactTarget::Request> request);
    bool call_speedSlider(std::shared_ptr<leitungssatz_interfaces::srv::SetSpeedSlider::Request> request);

    // Below functions are not defined anywhere
    // bool call_jog_start(bool IO);
    // bool call_finish_detection();

    bool call_trajectory();

    bool setSpeed(float speed);
    bool isTransformExist(const std::string& src, std::string& target);

    void ptpHome();
    void demo_point();

    bool set_payload(double mass, const geometry_msgs::msg::Vector3& center_of_gravity);
    bool setClipGun(bool on);
    bool call_rel_cart_target(std::shared_ptr<leitungssatz_interfaces::srv::SetRelCartTarget::Request> request);

    std::shared_ptr<rclcpp::Node> get_node() { return node_; }
};

} // namespace leitungssatz
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "control_msgs/action/gripper_command.hpp"
#include "leitungssatz_interfaces/srv/set_joint_target.hpp"
#include "leitungssatz_interfaces/srv/set_trajectory.hpp"
#include "leitungssatz_interfaces/srv/set_freedrive.hpp"
#include "leitungssatz_interfaces/srv/set_gripper.hpp"
#include "leitungssatz_interfaces/srv/set_cart_target.hpp"
#include "leitungssatz_interfaces/srv/start_jog.hpp"
#include "leitungssatz_interfaces/srv/set_force_target.hpp"
#include "leitungssatz_interfaces/msg/path_entry.hpp"
#include <trajectory_msgs/msg/joint_trajectory.hpp>

using FollowJT = control_msgs::action::FollowJointTrajectory;
using GripperCmd = control_msgs::action::GripperCommand;

class SimServiceBridge : public rclcpp::Node
{
public:
  SimServiceBridge() : Node("sim_service_bridge")
  {
    joint_action_client_ = rclcpp_action::create_client<FollowJT>(
      this, "joint_trajectory_controller/follow_joint_trajectory");
    
    gripper_action_client_ = rclcpp_action::create_client<GripperCmd>(
      this, "gripper_position_controller/gripper_cmd");

    srv_set_joint_ = create_service<leitungssatz_interfaces::srv::SetJointTarget>(
      "ur_hardware_interface/set_joint_target",
      std::bind(&SimServiceBridge::handle_set_joint, this, std::placeholders::_1, std::placeholders::_2));

    srv_set_traj_ = create_service<leitungssatz_interfaces::srv::SetTrajectory>(
      "ur_hardware_interface/set_trajectory",
      std::bind(&SimServiceBridge::handle_set_trajectory, this, std::placeholders::_1, std::placeholders::_2));

    srv_set_cart_ = create_service<leitungssatz_interfaces::srv::SetCartTarget>(
      "ur_hardware_interface/set_cart_target",
      std::bind(&SimServiceBridge::handle_set_cart, this, std::placeholders::_1, std::placeholders::_2));

    srv_set_freedrive_ = create_service<leitungssatz_interfaces::srv::SetFreedrive>(
      "ur_hardware_interface/set_freedrive",
      std::bind(&SimServiceBridge::handle_set_freedrive, this, std::placeholders::_1, std::placeholders::_2));

    srv_set_gripper_ = create_service<leitungssatz_interfaces::srv::SetGripper>(
      "ur_hardware_interface/robotiq/set_gripper",
      std::bind(&SimServiceBridge::handle_set_gripper, this, std::placeholders::_1, std::placeholders::_2));

    srv_start_jog_ = create_service<leitungssatz_interfaces::srv::StartJog>(
      "ur_hardware_interface/start_jog",
      std::bind(&SimServiceBridge::handle_start_jog, this, std::placeholders::_1, std::placeholders::_2));

    srv_set_force_ = create_service<leitungssatz_interfaces::srv::SetForceTarget>(
      "ur_hardware_interface/set_force_mode",
      std::bind(&SimServiceBridge::handle_set_force, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "Sim service bridge ready");
  }

private:
  void handle_set_joint(
    const std::shared_ptr<leitungssatz_interfaces::srv::SetJointTarget::Request> req,
    std::shared_ptr<leitungssatz_interfaces::srv::SetJointTarget::Response> res)
  {
    if (!joint_action_client_->wait_for_action_server(std::chrono::seconds(1))) {
      res->success = false;
      return;
    }

    auto goal = FollowJT::Goal();
    goal.trajectory.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                   "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
    
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.assign(req->joint_goal.begin(), req->joint_goal.end());
    point.velocities = std::vector<double>(6, 0.0);
    point.time_from_start = rclcpp::Duration::from_seconds(3.0);
    goal.trajectory.points.push_back(point);

    auto send_goal_options = rclcpp_action::Client<FollowJT>::SendGoalOptions();
    joint_action_client_->async_send_goal(goal, send_goal_options);
    res->success = true;
  }

  void handle_set_trajectory(
    const std::shared_ptr<leitungssatz_interfaces::srv::SetTrajectory::Request> req,
    std::shared_ptr<leitungssatz_interfaces::srv::SetTrajectory::Response> res)
  {
    if (!joint_action_client_->wait_for_action_server(std::chrono::seconds(1))) {
      res->success = false;
      return;
    }

    auto goal = FollowJT::Goal();
    goal.trajectory.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                   "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

    double cumulative_time = 0.0;
    for (const auto& entry : req->path) {
      if (entry.position_type == 1) {
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions.assign(entry.joint_goal.begin(), entry.joint_goal.end());
        point.velocities = std::vector<double>(6, 0.0);
        cumulative_time += 2.0;
        point.time_from_start = rclcpp::Duration::from_seconds(cumulative_time);
        goal.trajectory.points.push_back(point);
      }
    }

    if (goal.trajectory.points.empty()) {
      res->success = false;
      return;
    }
    
    joint_action_client_->async_send_goal(goal);
    res->success = true;
  }  

  void handle_set_cart(
    const std::shared_ptr<leitungssatz_interfaces::srv::SetCartTarget::Request>,
    std::shared_ptr<leitungssatz_interfaces::srv::SetCartTarget::Response> res)
  {
    res->success = false;
  }

  void handle_set_freedrive(
    const std::shared_ptr<leitungssatz_interfaces::srv::SetFreedrive::Request>,
    std::shared_ptr<leitungssatz_interfaces::srv::SetFreedrive::Response> res)
  {
    res->success = true;
    res->freedrive_status = 0;
    res->message = "Sim stub";
  }

  void handle_start_jog(
    const std::shared_ptr<leitungssatz_interfaces::srv::StartJog::Request>,
    std::shared_ptr<leitungssatz_interfaces::srv::StartJog::Response> res)
  {
    res->success = true;
  }

  void handle_set_force(
    const std::shared_ptr<leitungssatz_interfaces::srv::SetForceTarget::Request>,
    std::shared_ptr<leitungssatz_interfaces::srv::SetForceTarget::Response> res)
  {
    res->success = true;
  }

  void handle_set_gripper(
    const std::shared_ptr<leitungssatz_interfaces::srv::SetGripper::Request> req,
    std::shared_ptr<leitungssatz_interfaces::srv::SetGripper::Response> res)
  {
    if (!gripper_action_client_->wait_for_action_server(std::chrono::seconds(1))) {
      res->success = false;
      res->e_object_status = 0;
      return;
    }

    auto goal = GripperCmd::Goal();
    double normalized_position = 0.0;
    
    if (req->position_unit == 0) {
      normalized_position = (req->position / 100.0) * 0.8;
    } else if (req->position_unit == 1) {
      normalized_position = 0.8 - (req->position / 85.0) * 0.8;
    } else if (req->position_unit == 2) {
      normalized_position = (req->position / 255.0) * 0.8;
    }
    
    goal.command.position = normalized_position;
    goal.command.max_effort = (req->force / 100.0) * 100.0;
    
    gripper_action_client_->async_send_goal(goal);
    res->success = true;
    res->e_object_status = 0;
  }

  rclcpp_action::Client<FollowJT>::SharedPtr joint_action_client_;
  rclcpp_action::Client<GripperCmd>::SharedPtr gripper_action_client_;
  rclcpp::Service<leitungssatz_interfaces::srv::SetJointTarget>::SharedPtr srv_set_joint_;
  rclcpp::Service<leitungssatz_interfaces::srv::SetTrajectory>::SharedPtr srv_set_traj_;
  rclcpp::Service<leitungssatz_interfaces::srv::SetCartTarget>::SharedPtr srv_set_cart_;
  rclcpp::Service<leitungssatz_interfaces::srv::SetFreedrive>::SharedPtr srv_set_freedrive_;
  rclcpp::Service<leitungssatz_interfaces::srv::SetGripper>::SharedPtr srv_set_gripper_;
  rclcpp::Service<leitungssatz_interfaces::srv::StartJog>::SharedPtr srv_start_jog_;
  rclcpp::Service<leitungssatz_interfaces::srv::SetForceTarget>::SharedPtr srv_set_force_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimServiceBridge>());
  rclcpp::shutdown();
  return 0;
}
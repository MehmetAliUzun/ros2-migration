#pragma once

#include <string>
#include <vector>
#include <array>
#include <memory>
#include <mutex>
#include <thread>
#include <atomic>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/handle.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include "leitungssatz_interfaces/srv/set_joint_target.hpp"
#include "leitungssatz_interfaces/srv/set_trajectory.hpp"
#include "leitungssatz_interfaces/srv/set_cart_target.hpp"

using FollowJT = control_msgs::action::FollowJointTrajectory;
using GoalHandleFollowJT = rclcpp_action::ClientGoalHandle<FollowJT>;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace leitungssatz_interfaces
{

class URHardwareInterfaceSim : public hardware_interface::SystemInterface
{
public:
  URHardwareInterfaceSim();
  ~URHardwareInterfaceSim() override;

  // SystemInterface overrides
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // minimal joint storage (6 DOF)
  std::array<double,6> hw_positions_;
  std::array<double,6> hw_velocities_;
  std::array<double,6> hw_efforts_;
  std::array<double,6> hw_commands_;

  std::vector<std::string> joint_names_;

  // non-RT node for services and action client
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<FollowJT>::SharedPtr traj_action_client_;

  // service servers exposing same API as real hw_interface
  rclcpp::Service<leitungssatz_interfaces::srv::SetJointTarget>::SharedPtr srv_set_joint_target_;
  rclcpp::Service<leitungssatz_interfaces::srv::SetTrajectory>::SharedPtr srv_set_trajectory_;
  rclcpp::Service<leitungssatz_interfaces::srv::SetCartTarget>::SharedPtr srv_set_cart_target_;

  // helper to send a simple single-point trajectory goal
  bool send_joint_goal(const std::vector<double> & positions, double time_s, bool asynchronous, bool & accepted, int timeout_sec = 30);

  // lifecycle helpers
  std::string node_name_{"ur_hardware_interface_sim"};
  std::string action_name_{"joint_trajectory_controller/follow_joint_trajectory"};
};

} // namespace leitungssatz_interfaces
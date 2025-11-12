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
#include "rcutils/logging_macros.h"

#include <realtime_tools/realtime_buffer.h>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include "leitungssatz/srv/add_tf2.hpp"
#include "leitungssatz_interfaces/srv/set_freedrive.hpp"
#include "leitungssatz_interfaces/srv/set_cart_target.hpp"
#include "leitungssatz_interfaces/srv/start_jog.hpp"
#include "leitungssatz_interfaces/srv/set_force_target.hpp"
#include "leitungssatz_interfaces/srv/set_gripper.hpp"
#include "leitungssatz_interfaces/msg/jog_control.hpp"

#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_control_interface.h>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace leitungssatz_interfaces
{

class URHardwareInterface : public hardware_interface::SystemInterface
{
public:
  URHardwareInterface();
  ~URHardwareInterface() override;

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
  // Joint data (6 DOF UR)
  std::array<double, 6> hw_positions_;
  std::array<double, 6> hw_velocities_;
  std::array<double, 6> hw_efforts_;
  std::array<double, 6> hw_position_commands_;

  // Realtime buffers populated by non-RTDE thread
  realtime_tools::RealtimeBuffer<std::array<double, 6>> rtde_positions_buffer_;
  realtime_tools::RealtimeBuffer<std::array<double, 6>> rtde_velocities_buffer_;
  realtime_tools::RealtimeBuffer<std::array<double, 6>> rtde_efforts_buffer_;

  // TCP pose/wrench/tcp_info buffers
  realtime_tools::RealtimeBuffer<geometry_msgs::msg::TransformStamped> tcp_pose_buffer_;
  realtime_tools::RealtimeBuffer<geometry_msgs::msg::WrenchStamped> wrench_buffer_;
  realtime_tools::RealtimeBuffer<geometry_msgs::msg::TwistStamped> tcp_info_buffer_;

  // Non-realtime rclcpp node for services/publishers
  rclcpp::Node::SharedPtr node_;

  // Publishers (non-realtime)
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr tcp_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr tcp_info_pub_;

  // Service servers (names match your point_recording)
  rclcpp::Service<leitungssatz_interfaces::srv::SetFreedrive>::SharedPtr srv_set_freedrive_;
  rclcpp::Service<leitungssatz_interfaces::srv::SetCartTarget>::SharedPtr srv_set_cart_target_;
  rclcpp::Service<leitungssatz_interfaces::srv::StartJog>::SharedPtr srv_start_jog_;
  rclcpp::Service<leitungssatz_interfaces::srv::SetForceTarget>::SharedPtr srv_set_force_target_;
  rclcpp::Service<leitungssatz_interfaces::srv::SetGripper>::SharedPtr srv_set_gripper_;
  rclcpp::Service<leitungssatz::srv::AddTf2>::SharedPtr srv_add_tf2_;

  // RTDE / UR communication objects (real types)
  std::unique_ptr<ur_rtde::RTDEReceiveInterface> rtde_receive_;
  std::unique_ptr<ur_rtde::RTDEControlInterface> rtde_control_;

  // RTDE thread
  std::thread rtde_thread_;
  std::atomic<bool> rtde_thread_running_{false};

  // Executor & thread to spin the non-realtime node so services work
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread executor_thread_;

  // Mutex protecting rtde_control_ usage from service callbacks
  std::mutex rtde_control_mutex_;

  // Jog subscription (on-demand)
  rclcpp::Subscription<leitungssatz_interfaces::msg::JogControl>::SharedPtr jog_control_sub_;

  // Convert geometry_msgs::msg::Transform to UR angle-axis vector [x,y,z,rx,ry,rz]
  void transform_to_angelaxis(const geometry_msgs::msg::Transform & transform, std::vector<double>& angleaxis);

  // Service handlers (non-realtime)
  void handle_set_freedrive(
    const std::shared_ptr<leitungssatz_interfaces::srv::SetFreedrive::Request> request,
    std::shared_ptr<leitungssatz_interfaces::srv::SetFreedrive::Response> response);

  void handle_set_cart_target(
    const std::shared_ptr<leitungssatz_interfaces::srv::SetCartTarget::Request> request,
    std::shared_ptr<leitungssatz_interfaces::srv::SetCartTarget::Response> response);

  void handle_start_jog(
    const std::shared_ptr<leitungssatz_interfaces::srv::StartJog::Request> request,
    std::shared_ptr<leitungssatz_interfaces::srv::StartJog::Response> response);

  void handle_set_force_target(
    const std::shared_ptr<leitungssatz_interfaces::srv::SetForceTarget::Request> request,
    std::shared_ptr<leitungssatz_interfaces::srv::SetForceTarget::Response> response);

  void handle_set_gripper(
    const std::shared_ptr<leitungssatz_interfaces::srv::SetGripper::Request> request,
    std::shared_ptr<leitungssatz_interfaces::srv::SetGripper::Response> response);

  void handle_add_tf2(
    const std::shared_ptr<leitungssatz::srv::AddTf2::Request> request,
    std::shared_ptr<leitungssatz::srv::AddTf2::Response> response);

  // jog callback
  void jog_control_callback(const leitungssatz_interfaces::msg::JogControl::SharedPtr msg);

  // connect / disconnect helpers
  bool connect_to_robot();
  void disconnect_from_robot();

  void rtde_thread_loop();

  // internal state
  std::vector<std::string> joint_names_;
  std::string robot_ip_;

  // last motion (same semantics as ROS1)
  int last_motion_{0};
};

} // namespace leitungssatz_interfaces
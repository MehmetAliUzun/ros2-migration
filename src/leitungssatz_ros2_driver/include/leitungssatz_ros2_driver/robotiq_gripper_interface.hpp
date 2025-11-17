#pragma once

#include <memory>
#include <string>
#include <thread>
#include <atomic>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "leitungssatz_interfaces/msg/gripper_info.hpp"
#include "leitungssatz_interfaces/srv/set_gripper.hpp"
#include "leitungssatz_interfaces/srv/get_gripper_calib.hpp"

#include <ur_rtde/robotiq_gripper.h>

class RobotiqGripperInterface
{
public:
  RobotiqGripperInterface();
  ~RobotiqGripperInterface();

  bool init(const rclcpp::Node::SharedPtr & node, const std::string & robot_ip);
  void shutdown();

  // service handler wrapper (can be called by hw_interface)
  bool set_gripper(const std::shared_ptr<leitungssatz_interfaces::srv::SetGripper::Request> req,
                   std::shared_ptr<leitungssatz_interfaces::srv::SetGripper::Response> res);

private:
  void run();
  void read_gripper_infos();
  void publish_gripper_infos();

  rclcpp::Node::SharedPtr node_;
  std::string robot_ip_;

  std::unique_ptr<ur_rtde::RobotiqGripper> robotiq_gripper_;

  rclcpp::Publisher<leitungssatz_interfaces::msg::GripperInfo>::SharedPtr gripper_info_pub_;
  rclcpp::Service<leitungssatz_interfaces::srv::SetGripper>::SharedPtr set_gripper_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calib_gripper_srv_;
  rclcpp::Service<leitungssatz_interfaces::srv::GetGripperCalib>::SharedPtr get_gripper_calib_srv_;

  leitungssatz_interfaces::msg::GripperInfo gripper_info_msg_;

  std::thread thread_;
  std::atomic<bool> running_{false};
};
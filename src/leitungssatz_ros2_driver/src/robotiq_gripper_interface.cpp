#include "leitungssatz_ros2_driver/robotiq_gripper_interface.hpp"
#include <chrono>

using namespace std::chrono_literals;

RobotiqGripperInterface::RobotiqGripperInterface() = default;
RobotiqGripperInterface::~RobotiqGripperInterface() { shutdown(); }

bool RobotiqGripperInterface::init(const rclcpp::Node::SharedPtr & node, const std::string & robot_ip)
{
  if (!node) return false;
  node_ = node;
  robot_ip_ = robot_ip;

  try {
    robotiq_gripper_ = std::make_unique<ur_rtde::RobotiqGripper>(robot_ip_, 63352, true);
    robotiq_gripper_->connect();
    robotiq_gripper_->activate();
    robotiq_gripper_->close();
    robotiq_gripper_->setUnit(ur_rtde::RobotiqGripper::POSITION, ur_rtde::RobotiqGripper::UNIT_MM);
    robotiq_gripper_->setUnit(ur_rtde::RobotiqGripper::SPEED, ur_rtde::RobotiqGripper::UNIT_PERCENT);
    robotiq_gripper_->setUnit(ur_rtde::RobotiqGripper::FORCE, ur_rtde::RobotiqGripper::UNIT_PERCENT);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Robotiq init exception: %s", e.what());
    return false;
  }

  gripper_info_pub_ = node_->create_publisher<leitungssatz_interfaces::msg::GripperInfo>("gripper_infos", 10);

  // services
  set_gripper_srv_ = node_->create_service<leitungssatz_interfaces::srv::SetGripper>("set_gripper",
    [this](const std::shared_ptr<leitungssatz_interfaces::srv::SetGripper::Request> req,
           std::shared_ptr<leitungssatz_interfaces::srv::SetGripper::Response> res){
      (void)set_gripper(req, res);
    });

  calib_gripper_srv_ = node_->create_service<std_srvs::srv::Trigger>("calib_gripper",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res){
      try {
        if (!robotiq_gripper_->isConnected()) robotiq_gripper_->connect();
        if (!robotiq_gripper_->isActive()) robotiq_gripper_->activate();
        robotiq_gripper_->autoCalibrate();
        res->success = true;
      } catch (...) { res->success = false; }
    });

  get_gripper_calib_srv_ = node_->create_service<leitungssatz_interfaces::srv::GetGripperCalib>("get_gripper_calib",
    [this](const std::shared_ptr<leitungssatz_interfaces::srv::GetGripperCalib::Request>, std::shared_ptr<leitungssatz_interfaces::srv::GetGripperCalib::Response> res){
      try {
        if (!robotiq_gripper_->isConnected()) robotiq_gripper_->connect();
        if (!robotiq_gripper_->isActive()) robotiq_gripper_->activate();

        robotiq_gripper_->setUnit(ur_rtde::RobotiqGripper::POSITION, ur_rtde::RobotiqGripper::UNIT_MM);
        res->mm_max = robotiq_gripper_->getOpenPosition();
        res->mm_current = robotiq_gripper_->getCurrentPosition();
        res->mm_min = robotiq_gripper_->getClosedPosition();

        robotiq_gripper_->setUnit(ur_rtde::RobotiqGripper::POSITION, ur_rtde::RobotiqGripper::UNIT_PERCENT);
        res->per_max = robotiq_gripper_->getOpenPosition();
        res->per_current = robotiq_gripper_->getCurrentPosition();
        res->per_min = robotiq_gripper_->getClosedPosition();

        robotiq_gripper_->setUnit(ur_rtde::RobotiqGripper::POSITION, ur_rtde::RobotiqGripper::UNIT_DEVICE);
        res->dev_max = robotiq_gripper_->getOpenPosition();
        res->dev_current = robotiq_gripper_->getCurrentPosition();
        res->dev_min = robotiq_gripper_->getClosedPosition();
      } catch (...) {
        RCLCPP_ERROR(node_->get_logger(), "get_gripper_calib failed");
      }
    });

  running_ = true;
  thread_ = std::thread(&RobotiqGripperInterface::run, this);
  RCLCPP_INFO(node_->get_logger(), "RobotiqGripperInterface initialized");
  return true;
}

void RobotiqGripperInterface::shutdown()
{
  running_ = false;
  if (thread_.joinable()) thread_.join();
  robotiq_gripper_.reset();
}

bool RobotiqGripperInterface::set_gripper(const std::shared_ptr<leitungssatz_interfaces::srv::SetGripper::Request> req,
                                          std::shared_ptr<leitungssatz_interfaces::srv::SetGripper::Response> res)
{
  try {
    if (!robotiq_gripper_->isConnected()) robotiq_gripper_->connect();
    if (!robotiq_gripper_->isActive()) robotiq_gripper_->activate();

    bool error = false;
    switch (req->position_unit) {
      case 0:
        robotiq_gripper_->setUnit(ur_rtde::RobotiqGripper::POSITION, ur_rtde::RobotiqGripper::UNIT_PERCENT);
        if (req->position > 100.0 || req->position < 0.0) error = true;
        break;
      case 1:
        robotiq_gripper_->setUnit(ur_rtde::RobotiqGripper::POSITION, ur_rtde::RobotiqGripper::UNIT_MM);
        break;
      case 2:
        robotiq_gripper_->setUnit(ur_rtde::RobotiqGripper::POSITION, ur_rtde::RobotiqGripper::UNIT_DEVICE);
        if (req->position > 255 || req->position < 0) error = true;
        break;
      default:
        error = true;
        break;
    }

    if (!error) {
      if (req->asynchronous) {
        res->success = robotiq_gripper_->move(req->position, req->speed, req->force, ur_rtde::RobotiqGripper::START_MOVE);
      } else {
        res->success = robotiq_gripper_->move(req->position, req->speed, req->force, ur_rtde::RobotiqGripper::WAIT_FINISHED);
      }
      res->eObjectStatus = robotiq_gripper_->objectDetectionStatus();
    } else {
      res->success = false;
      RCLCPP_ERROR(node_->get_logger(), "Gripper: position values out of range");
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "set_gripper exception: %s", e.what());
    res->success = false;
    return false;
  }
  return true;
}

void RobotiqGripperInterface::read_gripper_infos()
{
  gripper_info_msg_.stamp = node_->get_clock()->now();
  robotiq_gripper_->setUnit(ur_rtde::RobotiqGripper::POSITION, ur_rtde::RobotiqGripper::UNIT_MM);
  gripper_info_msg_.position_mm = robotiq_gripper_->getCurrentPosition();
  robotiq_gripper_->setUnit(ur_rtde::RobotiqGripper::POSITION, ur_rtde::RobotiqGripper::UNIT_PERCENT);
  gripper_info_msg_.position_per = robotiq_gripper_->getCurrentPosition();
  robotiq_gripper_->setUnit(ur_rtde::RobotiqGripper::POSITION, ur_rtde::RobotiqGripper::UNIT_DEVICE);
  gripper_info_msg_.position_device = robotiq_gripper_->getCurrentPosition();

  gripper_info_msg_.eFaultCode = robotiq_gripper_->faultStatus();
  gripper_info_msg_.eObjectStatus = robotiq_gripper_->objectDetectionStatus();

  gripper_info_msg_.connected = robotiq_gripper_->isConnected();
  gripper_info_msg_.activated = robotiq_gripper_->isActive();
}

void RobotiqGripperInterface::publish_gripper_infos()
{
  if (gripper_info_pub_) gripper_info_pub_->publish(gripper_info_msg_);
}

void RobotiqGripperInterface::run()
{
  RCLCPP_INFO(node_->get_logger(), "Gripper thread started");
  rclcpp::Rate rate(100);
  while (running_ && rclcpp::ok()) {
    try {
      if (robotiq_gripper_->isConnected()) {
        read_gripper_infos();
        publish_gripper_infos();
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(node_->get_logger(), "Gripper thread exception: %s", e.what());
    }
    rate.sleep();
  }
  RCLCPP_INFO(node_->get_logger(), "Gripper thread stopping");
}
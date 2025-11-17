#include "leitungssatz_ros2_driver/dashboard_client.hpp"
#include <chrono>

using namespace std::chrono_literals;

DashboardClient::DashboardClient() = default;

DashboardClient::~DashboardClient()
{
  shutdown();
}

bool DashboardClient::init(const rclcpp::Node::SharedPtr & node, const std::string & robot_ip)
{
  if (!node) return false;
  node_ = node;
  robot_ip_ = robot_ip;

  // instantiate RTDE IO and dashboard client
  try {
    rtde_io_ = std::make_unique<ur_rtde::RTDEIOInterface>(robot_ip_);
    rtde_dashboard_ = std::make_unique<ur_rtde::DashboardClient>(robot_ip_);
    rtde_dashboard_->connect(2000U);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "DashboardClient connect exception: %s", e.what());
    return false;
  }

  // publishers & services on the provided node (namespace of the node)
  dashboard_info_pub_ = node_->create_publisher<leitungssatz_interfaces::msg::DashboardInfo>("dashboard_info", 10);

  power_off_srv_ = node_->create_service<std_srvs::srv::Trigger>("powerOff",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res){
      try {
        rtde_dashboard_->powerOff();
        res->success = true;
      } catch (...) { res->success = false; }
    });

  brake_release_srv_ = node_->create_service<std_srvs::srv::Trigger>("brake_release",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res){
      try {
        rtde_dashboard_->brakeRelease();
        res->success = true;
      } catch (...) { res->success = false; }
    });

  unlock_protective_stop_srv_ = node_->create_service<std_srvs::srv::Trigger>("unlock_protective_stop",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res){
      try {
        rtde_dashboard_->unlockProtectiveStop();
        res->success = true;
      } catch (...) { res->success = false; }
    });

  restart_safety_srv_ = node_->create_service<std_srvs::srv::Trigger>("restart_safety",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res){
      try {
        rtde_dashboard_->closeSafetyPopup();
        rtde_dashboard_->brakeRelease();
        res->success = true;
      } catch (...) { res->success = false; }
    });

  speed_slider_srv_ = node_->create_service<leitungssatz_interfaces::srv::SetSpeedSlider>("set_speed_slider",
    [this](const std::shared_ptr<leitungssatz_interfaces::srv::SetSpeedSlider::Request> req,
           std::shared_ptr<leitungssatz_interfaces::srv::SetSpeedSlider::Response> res){
      try {
        if (req->slider >= 0 && req->slider <= 2) {
          res->success = rtde_io_->setSpeedSlider(req->slider);
        } else {
          res->success = false;
        }
      } catch (...) { res->success = false; }
    });

  // start background thread to poll dashboard info
  running_ = true;
  thread_ = std::thread(&DashboardClient::run, this);

  RCLCPP_INFO(node_->get_logger(), "DashboardClient initialized (robot_ip=%s)", robot_ip_.c_str());
  return true;
}

void DashboardClient::shutdown()
{
  running_ = false;
  if (thread_.joinable()) thread_.join();
  rtde_dashboard_.reset();
  rtde_io_.reset();
}

bool DashboardClient::isInRemoteControl()
{
  try {
    return rtde_dashboard_ && rtde_dashboard_->isInRemoteControl();
  } catch (...) { return false; }
}

void DashboardClient::powerOn()
{
  try { if (rtde_dashboard_) rtde_dashboard_->powerOn(); } catch (...) {}
}

void DashboardClient::powerOff()
{
  try { if (rtde_dashboard_) rtde_dashboard_->powerOff(); } catch (...) {}
}

void DashboardClient::brakeRelease()
{
  try { if (rtde_dashboard_) rtde_dashboard_->brakeRelease(); } catch (...) {}
}

void DashboardClient::play()
{
  try { if (rtde_dashboard_) rtde_dashboard_->play(); } catch (...) {}
}

void DashboardClient::unlockProtectiveStop()
{
  try { if (rtde_dashboard_) rtde_dashboard_->unlockProtectiveStop(); } catch (...) {}
}

void DashboardClient::restartSafety()
{
  try {
    if (rtde_dashboard_) {
      rtde_dashboard_->closeSafetyPopup();
      rtde_dashboard_->brakeRelease();
    }
  } catch (...) {}
}

void DashboardClient::run()
{
  RCLCPP_INFO(node_->get_logger(), "DashboardClient: thread started");
  rclcpp::Rate rate(50);
  while (running_ && rclcpp::ok()) {
    try {
      // popup text and connected state
      if (rtde_dashboard_) {
        try {
          std::string popup;
          rtde_dashboard_->popup(popup);
          dashboard_info_msg_.popup = popup;
          dashboard_info_msg_.isConnected = rtde_dashboard_->isConnected();
        } catch (...) {
          dashboard_info_msg_.isConnected = false;
        }
      } else {
        dashboard_info_msg_.isConnected = false;
      }

      if (dashboard_info_pub_) dashboard_info_pub_->publish(dashboard_info_msg_);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(node_->get_logger(), "DashboardClient run exception: %s", e.what());
    } catch (...) {
      RCLCPP_ERROR(node_->get_logger(), "DashboardClient run unknown exception");
    }
    rate.sleep();
  }
  RCLCPP_INFO(node_->get_logger(), "DashboardClient: thread stopping");
}
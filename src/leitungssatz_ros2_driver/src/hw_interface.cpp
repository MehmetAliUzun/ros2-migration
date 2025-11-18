#include "leitungssatz_ros2_driver/hw_interface.hpp"

#include <chrono>
#include <thread>
#include <algorithm>

#include <Eigen/Geometry>

#include <tf2/LinearMath/Quaternion.h>
// #include <tf2_eigen/tf2_eigen.hpp>

#include "pluginlib/class_list_macros.hpp"

using namespace std::chrono_literals;
using leitungssatz_interfaces::URHardwareInterface;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// ------------------ Lifecycle / SystemInterface ------------------

URHardwareInterface::URHardwareInterface()
{
}

URHardwareInterface::~URHardwareInterface()
{
  // stop executor
  if (executor_) {
    executor_->cancel();
  }
  if (executor_thread_.joinable()) {
    executor_thread_.join();
  }

  // stop RTDE thread & disconnect
  rtde_thread_running_ = false;
  if (rtde_thread_.joinable()) {
    rtde_thread_.join();
  }
  disconnect_from_robot();

  if (dashboard_) {
    dashboard_->shutdown();
  }
  if (gripper_) {
    gripper_->shutdown();
  }
}

CallbackReturn URHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    RCUTILS_LOG_ERROR_NAMED("URHardwareInterface", "Base class on_init failed");
    return CallbackReturn::ERROR;
  }

  // Read joint names from hardware info
  joint_names_.clear();
  for (const auto & j : info.joints) {
    joint_names_.push_back(j.name);
  }
  if (joint_names_.size() != 6) {
    RCUTILS_LOG_WARN_NAMED(
      "URHardwareInterface", "Expected 6 joints, got %zu", joint_names_.size());
  }

  hw_positions_.fill(0.0);
  hw_velocities_.fill(0.0);
  hw_efforts_.fill(0.0);
  hw_position_commands_.fill(0.0);

  // initialize realtime buffers
  rtde_positions_buffer_.writeFromNonRT(hw_positions_);
  rtde_velocities_buffer_.writeFromNonRT(hw_velocities_);
  rtde_efforts_buffer_.writeFromNonRT(hw_efforts_);

  // default robot IP (override via hardware_parameters in ros2_control YAML if present)
  robot_ip_ = "172.31.1.200";
  for (auto & p : info_.hardware_parameters) {
    if (p.first == "robot_ip") {
      robot_ip_ = p.second;
    } else if (p.first == "use_dashboard") {
      use_dashboard_ = (p.second == "true" || p.second == "1");
    } else if (p.first == "use_robotiq") {
      use_robotiq_ = (p.second == "true" || p.second == "1");
    }
  }

  RCUTILS_LOG_INFO_NAMED(
    "URHardwareInterface", "on_init successful; robot_ip=%s", robot_ip_.c_str());
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
URHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(joint_names_[i], "position", &hw_positions_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(joint_names_[i], "velocity", &hw_velocities_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(joint_names_[i], "effort", &hw_efforts_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
URHardwareInterface::export_command_interfaces()
{
  // Mode B: we do NOT actually actuate via ros2_control, but we still expose position commands
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(joint_names_[i], "position", &hw_position_commands_[i]));
  }
  return command_interfaces;
}

CallbackReturn URHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
{
  RCUTILS_LOG_INFO_NAMED("URHardwareInterface", "Configuring hardware plugin...");

  // Create non-realtime rclcpp node for services/publishers
  node_ = std::make_shared<rclcpp::Node>("ur_hardware_interface_node");

  // Set up an executor to spin this node in background so services process requests
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  executor_thread_ = std::thread([this]() {
    try {
      executor_->spin();
    } catch (const std::exception & e) {
      RCUTILS_LOG_ERROR_NAMED(
        "URHardwareInterface", "Executor thread exception: %s", e.what());
    }
  });

  // Publishers (same topic names as ROS1, with node namespace /ur_hardware_interface)
  tcp_pose_pub_ =
    node_->create_publisher<geometry_msgs::msg::TransformStamped>("tcp_pose", 10);
  wrench_pub_ =
    node_->create_publisher<geometry_msgs::msg::WrenchStamped>("wrench", 10);
  tcp_info_pub_ =
    node_->create_publisher<geometry_msgs::msg::TwistStamped>("tcp_info", 10);

  // Service servers (same API as ROS1, but using leitungssatz_interfaces)
  srv_set_cart_target_ =
    node_->create_service<leitungssatz_interfaces::srv::SetCartTarget>(
      "set_cart_target",
      std::bind(
        &URHardwareInterface::handle_set_cart_target, this,
        std::placeholders::_1, std::placeholders::_2));

  srv_set_rel_cart_target_ =
    node_->create_service<leitungssatz_interfaces::srv::SetRelCartTarget>(
      "set_rel_cart_target",
      std::bind(
        &URHardwareInterface::handle_set_rel_cart_target, this,
        std::placeholders::_1, std::placeholders::_2));

  srv_set_joint_target_ =
    node_->create_service<leitungssatz_interfaces::srv::SetJointTarget>(
      "set_joint_target",
      std::bind(
        &URHardwareInterface::handle_set_joint_target, this,
        std::placeholders::_1, std::placeholders::_2));

  srv_set_rel_joint_target_ =
    node_->create_service<leitungssatz_interfaces::srv::SetJointTarget>(
      "set_rel_joint_target",
      std::bind(
        &URHardwareInterface::handle_set_rel_joint_target, this,
        std::placeholders::_1, std::placeholders::_2));

  srv_set_trajectory_ =
    node_->create_service<leitungssatz_interfaces::srv::SetTrajectory>(
      "set_trajectory",
      std::bind(
        &URHardwareInterface::handle_set_trajectory, this,
        std::placeholders::_1, std::placeholders::_2));

  srv_set_freedrive_ =
    node_->create_service<leitungssatz_interfaces::srv::SetFreedrive>(
      "set_freedive",
      std::bind(
        &URHardwareInterface::handle_set_freedrive, this,
        std::placeholders::_1, std::placeholders::_2));

  srv_set_force_target_ =
    node_->create_service<leitungssatz_interfaces::srv::SetForceTarget>(
      "set_force_mode",
      std::bind(
        &URHardwareInterface::handle_set_force_target, this,
        std::placeholders::_1, std::placeholders::_2));

  srv_set_contact_target_ =
    node_->create_service<leitungssatz_interfaces::srv::SetContactTarget>(
      "set_contact_target",
      std::bind(
        &URHardwareInterface::handle_set_contact_target, this,
        std::placeholders::_1, std::placeholders::_2));

  srv_start_jog_ =
    node_->create_service<leitungssatz_interfaces::srv::StartJog>(
      "start_jog",
      std::bind(
        &URHardwareInterface::handle_start_jog, this,
        std::placeholders::_1, std::placeholders::_2));

  srv_zero_ftsensor_ =
    node_->create_service<std_srvs::srv::Trigger>(
      "zero_ftsensor",
      std::bind(
        &URHardwareInterface::handle_zero_ftsensor, this,
        std::placeholders::_1, std::placeholders::_2));

  srv_log_ =
    node_->create_service<leitungssatz_interfaces::srv::Log>(
      "logging",
      std::bind(
        &URHardwareInterface::handle_log, this,
        std::placeholders::_1, std::placeholders::_2));

  srv_set_payload_ =
    node_->create_service<leitungssatz_interfaces::srv::SetPayload>(
      "set_payload",
      std::bind(
        &URHardwareInterface::handle_set_payload, this,
        std::placeholders::_1, std::placeholders::_2));

  srv_get_timestamp_ =
    node_->create_service<leitungssatz_interfaces::srv::GetTimeStamp>(
      "get_robot_timestamp",
      std::bind(
        &URHardwareInterface::handle_get_timestamp, this,
        std::placeholders::_1, std::placeholders::_2));

  srv_stop_motion_ =
    node_->create_service<leitungssatz_interfaces::srv::StopMotion>(
      "stop_motion",
      std::bind(
        &URHardwareInterface::handle_stop_motion, this,
        std::placeholders::_1, std::placeholders::_2));

  srv_set_clip_gun_ =
    node_->create_service<leitungssatz_interfaces::srv::SetClipGun>(
      "set_clip_gun",
      std::bind(
        &URHardwareInterface::handle_set_clip_gun, this,
        std::placeholders::_1, std::placeholders::_2));

  // Additional AddTf2 service (like ROS1 /store_tf)
  srv_add_tf2_ =
    node_->create_service<leitungssatz::srv::AddTf2>(
      "/store_tf",
      std::bind(
        &URHardwareInterface::handle_add_tf2, this,
        std::placeholders::_1, std::placeholders::_2));

  // Jog subscription (created when jog is active)
  jog_control_sub_.reset();

  // Connect to robot (instantiate RTDE objects)
  if (!connect_to_robot()) {
    RCUTILS_LOG_ERROR_NAMED("URHardwareInterface", "Failed to connect to robot");
    return CallbackReturn::ERROR;
  }

  // initialize optional helpers
  if (use_dashboard_) {
    dashboard_ = std::make_unique<DashboardClient>();
    if (!dashboard_->init(node_, robot_ip_)) {
      RCUTILS_LOG_WARN_NAMED(
        "URHardwareInterface",
        "Dashboard init failed (continuing without dashboard)");
      dashboard_.reset();
    } else {
      // wait until robot is in remote control (like ROS1 initInterfaces())
      size_t attempts = 0;
      while (
        rclcpp::ok() && !dashboard_->isInRemoteControl() && attempts++ < 40) {
        RCUTILS_LOG_WARN_NAMED(
          "URHardwareInterface",
          "Please set Robot to remote control (dashboard)");
        std::this_thread::sleep_for(500ms);
      }
    }
  }

  if (use_robotiq_) {
    gripper_ = std::make_unique<RobotiqGripperInterface>();
    if (!gripper_->init(node_, robot_ip_)) {
      RCUTILS_LOG_WARN_NAMED(
        "URHardwareInterface",
        "Robotiq init failed (continuing without gripper)");
      gripper_.reset();
    }
  }

  // start RTDE thread
  rtde_thread_running_ = true;
  rtde_thread_ = std::thread(&URHardwareInterface::rtde_thread_loop, this);

  RCUTILS_LOG_INFO_NAMED("URHardwareInterface", "Configured");
  return CallbackReturn::SUCCESS;
}

CallbackReturn URHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
  RCUTILS_LOG_INFO_NAMED("URHardwareInterface", "Activating hardware...");
  return CallbackReturn::SUCCESS;
}

CallbackReturn URHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCUTILS_LOG_INFO_NAMED("URHardwareInterface", "Deactivating hardware...");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type URHardwareInterface::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  // Copy latest RTDE values from realtime buffers into hw_* arrays (real-time safe)
  auto pos = rtde_positions_buffer_.readFromRT();
  auto vel = rtde_velocities_buffer_.readFromRT();
  auto eff = rtde_efforts_buffer_.readFromRT();

  if (pos) hw_positions_ = *pos;
  if (vel) hw_velocities_ = *vel;
  if (eff) hw_efforts_ = *eff;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type URHardwareInterface::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  // Mode B: we do NOT send commands via ros2_control; all motion goes through services.
  (void)hw_position_commands_;
  return hardware_interface::return_type::OK;
}

// ------------------ RTDE / robot comm helpers ------------------

bool URHardwareInterface::connect_to_robot()
{
  try {
    // instantiate ur_rtde receive/control objects
    rtde_receive_ = std::make_unique<ur_rtde::RTDEReceiveInterface>(robot_ip_);
    rtde_control_ = std::make_unique<ur_rtde::RTDEControlInterface>(robot_ip_);

    // optional: zero force-torque sensor at startup (as in ROS1)
    if (rtde_control_ && rtde_control_->isConnected()) {
      try {
        rtde_control_->zeroFtSensor();
      } catch (...) {
      }
    }

    RCUTILS_LOG_INFO_NAMED(
      "URHardwareInterface", "Connected to RTDE at %s", robot_ip_.c_str());
    return true;
  } catch (const std::exception & e) {
    RCUTILS_LOG_ERROR_NAMED(
      "URHardwareInterface", "connect_to_robot exception: %s", e.what());
    return false;
  }
}

void URHardwareInterface::disconnect_from_robot()
{
  std::lock_guard<std::mutex> lock(rtde_control_mutex_);
  try {
    if (rtde_control_) {
      // Stop any ongoing motions based on last_motion_, like ROS1 shutdown()
      switch (last_motion_) {
        case 1:
          try { rtde_control_->stopL(); } catch (...) {}
          break;
        case 2:
          try { rtde_control_->stopJ(); } catch (...) {}
          break;
        case 3:
          try { rtde_control_->forceModeStop(); } catch (...) {}
          break;
        case 4:
          try { rtde_control_->endFreedriveMode(); } catch (...) {}
          break;
        default:
          break;
      }
      rtde_control_.reset();
    }
    if (rtde_receive_) {
      rtde_receive_.reset();
    }
  } catch (...) {
  }
}

// RTDE thread: non-realtime loop that polls receive interface and updates realtime buffers
void URHardwareInterface::rtde_thread_loop()
{
  RCUTILS_LOG_INFO_NAMED("URHardwareInterface", "RTDE thread started");

  while (rtde_thread_running_ && rclcpp::ok()) {
    try {
      // Ensure receive/control are connected
      if (!rtde_receive_ || !rtde_receive_->isConnected()) {
        try {
          if (rtde_receive_) {
            rtde_receive_->reconnect();
          }
        } catch (...) {
        }
      }
      if (!rtde_control_ || !rtde_control_->isConnected()) {
        try {
          if (rtde_control_) {
            rtde_control_->reconnect();
          }
        } catch (...) {
        }
      }

      // If not connected, write zeros and sleep
      std::array<double, 6> q{};
      std::array<double, 6> qd{};
      std::array<double, 6> force{};

      if (rtde_receive_ && rtde_receive_->isConnected()) {
        auto qvec = rtde_receive_->getActualQ();
        auto qdvec = rtde_receive_->getActualQd();
        auto fvec = rtde_receive_->getActualTCPForce();
        auto tcp_pose_vec = rtde_receive_->getActualTCPPose();   // [x,y,z,rx,ry,rz]
        auto tcp_speed_vec = rtde_receive_->getActualTCPSpeed(); // [vx,vy,vz,wx,wy,wz]

        for (size_t i = 0; i < 6 && i < qvec.size(); ++i) q[i] = qvec[i];
        for (size_t i = 0; i < 6 && i < qdvec.size(); ++i) qd[i] = qdvec[i];
        for (size_t i = 0; i < 6 && i < fvec.size(); ++i) force[i] = fvec[i];

        // prepare tcp_pose message (similar to ROS1 read_tcp_pose())
        geometry_msgs::msg::TransformStamped tcp_msg;
        tcp_msg.header.stamp = node_->get_clock()->now();
        tcp_msg.header.frame_id = "base";
        tcp_msg.child_frame_id = "tool0";

        if (tcp_pose_vec.size() >= 6) {
          tcp_msg.transform.translation.x = tcp_pose_vec[0];
          tcp_msg.transform.translation.y = tcp_pose_vec[1];
          tcp_msg.transform.translation.z = tcp_pose_vec[2];

          Eigen::Vector3d rotVec(tcp_pose_vec[3], tcp_pose_vec[4], tcp_pose_vec[5]);
          double theta = rotVec.norm();
          Eigen::Quaterniond qrot = Eigen::Quaterniond::Identity();
          if (theta > 1e-9) {
            Eigen::Vector3d axis = rotVec.normalized();
            qrot = Eigen::Quaterniond(Eigen::AngleAxisd(theta, axis));
          }
          tf2::Quaternion qt(qrot.x(), qrot.y(), qrot.z(), qrot.w());
          tcp_msg.transform.rotation.x = qt.x();
          tcp_msg.transform.rotation.y = qt.y();
          tcp_msg.transform.rotation.z = qt.z();
          tcp_msg.transform.rotation.w = qt.w();        
        }

        // write joint / wrench buffers
        rtde_positions_buffer_.writeFromNonRT(q);
        rtde_velocities_buffer_.writeFromNonRT(qd);
        rtde_efforts_buffer_.writeFromNonRT(force);

        tcp_pose_buffer_.writeFromNonRT(tcp_msg);

        // publish non-RT topics
        if (tcp_pose_pub_) tcp_pose_pub_->publish(tcp_msg);

        if (wrench_pub_) {
          geometry_msgs::msg::WrenchStamped wmsg;
          wmsg.header = tcp_msg.header;
          wmsg.wrench.force.x = force[0];
          wmsg.wrench.force.y = force[1];
          wmsg.wrench.force.z = force[2];
          wmsg.wrench.torque.x = force[3];
          wmsg.wrench.torque.y = force[4];
          wmsg.wrench.torque.z = force[5];
          wrench_pub_->publish(wmsg);
          wrench_buffer_.writeFromNonRT(wmsg);
        }

        if (tcp_info_pub_) {
          geometry_msgs::msg::TwistStamped tmsg;
          tmsg.header = tcp_msg.header;
          if (tcp_speed_vec.size() >= 6) {
            tmsg.twist.linear.x = tcp_speed_vec[0];
            tmsg.twist.linear.y = tcp_speed_vec[1];
            tmsg.twist.linear.z = tcp_speed_vec[2];
            tmsg.twist.angular.x = tcp_speed_vec[3];
            tmsg.twist.angular.y = tcp_speed_vec[4];
            tmsg.twist.angular.z = tcp_speed_vec[5];
          }
          tcp_info_pub_->publish(tmsg);
          tcp_info_buffer_.writeFromNonRT(tmsg);
        }
      } else {
        // not connected: write zeros
        rtde_positions_buffer_.writeFromNonRT(q);
        rtde_velocities_buffer_.writeFromNonRT(qd);
        rtde_efforts_buffer_.writeFromNonRT(force);
      }
    } catch (const std::exception & e) {
      RCUTILS_LOG_ERROR_NAMED(
        "URHardwareInterface", "RTDE thread exception: %s", e.what());
    }
    std::this_thread::sleep_for(10ms);
  }

  RCUTILS_LOG_INFO_NAMED("URHardwareInterface", "RTDE thread stopping");
}

// ------------------ Helper: Transform -> angle-axis ------------------

void URHardwareInterface::transform_to_angelaxis(
  const geometry_msgs::msg::Transform & transform,
  std::vector<double> & angleaxis)
{
  Eigen::Quaterniond q(
    transform.rotation.w,
    transform.rotation.x,
    transform.rotation.y,
    transform.rotation.z);

  Eigen::AngleAxisd angleAxis(q);
  Eigen::Vector3d angles = angleAxis.axis() * angleAxis.angle();

  angleaxis.clear();
  angleaxis.reserve(6);
  angleaxis.push_back(transform.translation.x);
  angleaxis.push_back(transform.translation.y);
  angleaxis.push_back(transform.translation.z);
  angleaxis.insert(angleaxis.end(), angles.data(), angles.data() + 3);
}

// ------------------ Service handlers (motion & control) ------------------

// Freedrive (SetFreedrive)
void URHardwareInterface::handle_set_freedrive(
  const std::shared_ptr<leitungssatz_interfaces::srv::SetFreedrive::Request> request,
  std::shared_ptr<leitungssatz_interfaces::srv::SetFreedrive::Response> response)
{
  std::lock_guard<std::mutex> lock(rtde_control_mutex_);

  if (!rtde_control_ || !rtde_control_->isConnected()) {
    response->success = false;
    return;
  }

  try {
    if (request->io) {
      last_motion_ = 4;
      std::vector<double> frame;
      switch (request->feature) {
        case 0:
          frame = {0, 0, 0, 0, 0, 0};
          break;
        case 1:
          frame = rtde_receive_->getActualTCPPose();
          break;
        case 2:
          transform_to_angelaxis(request->custom_frame, frame);
          break;
        default:
          frame = {0, 0, 0, 0, 0, 0};
          break;
      }
      response->success = rtde_control_->freedriveMode(
        std::vector<int>(
          request->free_axes.begin(), request->free_axes.end()),
        frame);
      response->freedrive_status = rtde_control_->getFreedriveStatus();
    } else {
      response->success = rtde_control_->endFreedriveMode();
      if (response->success) {
        last_motion_ = 0;
      }
      response->freedrive_status = rtde_control_->getFreedriveStatus();
    }
  } catch (const std::exception & e) {
    RCUTILS_LOG_ERROR_NAMED(
      "URHardwareInterface", "set_freedrive exception: %s", e.what());
    response->success = false;
  }
}

// Cart target (absolute)
void URHardwareInterface::handle_set_cart_target(
  const std::shared_ptr<leitungssatz_interfaces::srv::SetCartTarget::Request> request,
  std::shared_ptr<leitungssatz_interfaces::srv::SetCartTarget::Response> response)
{
  std::vector<double> pose;
  transform_to_angelaxis(request->cartesian_goal, pose);

  bool success = false;
  bool error = false;

  std::lock_guard<std::mutex> lock(rtde_control_mutex_);

  if (!rtde_control_ || !rtde_control_->isConnected()) {
    response->success = false;
    return;
  }

  try {
    if (!rtde_control_->isPoseWithinSafetyLimits(pose)) {
      RCUTILS_LOG_WARN_NAMED(
        "URHardwareInterface", "Target is out of range or safety limits");
      response->success = false;
      return;
    }

    switch (request->mode) {
      case 1:
        last_motion_ = 1;
        success = rtde_control_->moveL(
          pose, request->speed, request->acceleration, request->asynchronous);
        break;
      case 2:
        last_motion_ = 1;
        success = rtde_control_->moveJ_IK(
          pose, request->speed, request->acceleration, request->asynchronous);
        break;
      default:
        error = true;
        success = false;
        break;
    }

    if (request->asynchronous && !error) {
      response->success = true;
    } else {
      response->success = success;
    }
  } catch (const std::exception & e) {
    RCUTILS_LOG_ERROR_NAMED(
      "URHardwareInterface", "set_cart_target exception: %s", e.what());
    response->success = false;
  }
}

// Cart target (relative)
void URHardwareInterface::handle_set_rel_cart_target(
  const std::shared_ptr<leitungssatz_interfaces::srv::SetRelCartTarget::Request> request,
  std::shared_ptr<leitungssatz_interfaces::srv::SetRelCartTarget::Response> response)
{
  // reconstruct current TCP pose from buffer
  auto tcp_pose_msg = tcp_pose_buffer_.readFromRT();
  geometry_msgs::msg::Transform trans;
  if (tcp_pose_msg) {
    trans = tcp_pose_msg->transform;
  } else {
    trans.translation.x = 0.0;
    trans.translation.y = 0.0;
    trans.translation.z = 0.0;
    trans.rotation.x = 0.0;
    trans.rotation.y = 0.0;
    trans.rotation.z = 0.0;
    trans.rotation.w = 1.0;
  }

  // apply relative translation
  trans.translation.x += request->rel_goal[0];
  trans.translation.y += request->rel_goal[1];
  trans.translation.z += request->rel_goal[2];

  // relative rotation (XYZ RPY style)
  Eigen::Quaterniond quat_rel =
    Eigen::AngleAxisd(request->rel_goal[3], Eigen::Vector3d::UnitX()) *
    Eigen::AngleAxisd(request->rel_goal[4], Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(request->rel_goal[5], Eigen::Vector3d::UnitZ());

  Eigen::Quaterniond quat(
    trans.rotation.w,
    trans.rotation.x,
    trans.rotation.y,
    trans.rotation.z);

  quat = quat * quat_rel;
  trans.rotation.x = quat.x();
  trans.rotation.y = quat.y();
  trans.rotation.z = quat.z();
  trans.rotation.w = quat.w();

  std::vector<double> pose;
  transform_to_angelaxis(trans, pose);

  bool success = false;
  bool error = false;

  std::lock_guard<std::mutex> lock(rtde_control_mutex_);

  if (!rtde_control_ || !rtde_control_->isConnected()) {
    response->success = false;
    return;
  }

  try {
    if (!rtde_control_->isPoseWithinSafetyLimits(pose)) {
      RCUTILS_LOG_WARN_NAMED(
        "URHardwareInterface", "Rel target is out of range or safety limits");
      response->success = false;
      return;
    }

    switch (request->mode) {
      case 1:
        last_motion_ = 1;
        success = rtde_control_->moveL(
          pose, request->speed, request->acceleration, request->asynchronous);
        break;
      case 2:
        last_motion_ = 1;
        success = rtde_control_->moveJ_IK(
          pose, request->speed, request->acceleration, request->asynchronous);
        break;
      default:
        error = true;
        success = false;
        break;
    }

    if (request->asynchronous && !error) {
      response->success = true;
    } else {
      response->success = success;
    }
  } catch (const std::exception & e) {
    RCUTILS_LOG_ERROR_NAMED(
      "URHardwareInterface", "set_rel_cart_target exception: %s", e.what());
    response->success = false;
  }
}

// Joint target (absolute)
void URHardwareInterface::handle_set_joint_target(
  const std::shared_ptr<leitungssatz_interfaces::srv::SetJointTarget::Request> request,
  std::shared_ptr<leitungssatz_interfaces::srv::SetJointTarget::Response> response)
{
  std::lock_guard<std::mutex> lock(rtde_control_mutex_);

  if (!rtde_control_ || !rtde_control_->isConnected()) {
    response->success = false;
    return;
  }

  try {
    std::vector<double> joints(
      request->joint_goal.begin(), request->joint_goal.end());

    if (!rtde_control_->isJointsWithinSafetyLimits(joints)) {
      RCUTILS_LOG_WARN_NAMED(
        "URHardwareInterface", "Joint target is out of joint or safety limits");
      response->success = false;
      return;
    }

    last_motion_ = 2;
    response->success = rtde_control_->moveJ(
      joints, request->speed, request->acceleration, request->asynchronous);
  } catch (const std::exception & e) {
    RCUTILS_LOG_ERROR_NAMED(
      "URHardwareInterface", "set_joint_target exception: %s", e.what());
    response->success = false;
  }
}

// Joint target (relative)
void URHardwareInterface::handle_set_rel_joint_target(
  const std::shared_ptr<leitungssatz_interfaces::srv::SetJointTarget::Request> request,
  std::shared_ptr<leitungssatz_interfaces::srv::SetJointTarget::Response> response)
{
  std::lock_guard<std::mutex> lock(rtde_control_mutex_);

  if (!rtde_control_ || !rtde_control_->isConnected()) {
    response->success = false;
    return;
  }

  try {
    // current joint positions from last read (hw_positions_)
    std::vector<double> joint_goal(hw_positions_.begin(), hw_positions_.end());
    std::vector<double> rel_goal(
      request->joint_goal.begin(), request->joint_goal.end());

    if (rel_goal.size() < joint_goal.size()) {
      rel_goal.resize(joint_goal.size(), 0.0);
    }

    for (size_t i = 0; i < joint_goal.size(); ++i) {
      joint_goal[i] += rel_goal[i];
    }

    if (!rtde_control_->isJointsWithinSafetyLimits(joint_goal)) {
      RCUTILS_LOG_WARN_NAMED(
        "URHardwareInterface", "Rel joint target is out of joint or safety limits");
      response->success = false;
      return;
    }

    last_motion_ = 2;
    response->success = rtde_control_->moveJ(
      joint_goal, request->speed, request->acceleration, request->asynchronous);
  } catch (const std::exception & e) {
    RCUTILS_LOG_ERROR_NAMED(
      "URHardwareInterface", "set_rel_joint_target exception: %s", e.what());
    response->success = false;
  }
}

// Trajectory (Path)
void URHardwareInterface::handle_set_trajectory(
  const std::shared_ptr<leitungssatz_interfaces::srv::SetTrajectory::Request> request,
  std::shared_ptr<leitungssatz_interfaces::srv::SetTrajectory::Response> response)
{
  std::lock_guard<std::mutex> lock(rtde_control_mutex_);

  if (!rtde_control_ || !rtde_control_->isConnected()) {
    response->success = false;
    return;
  }

  try {
    ur_rtde::Path path;

    for (const auto & entry : request->path) {
      std::vector<double> parameter;

      geometry_msgs::msg::Quaternion temp;
      // if rotation not default, treat as cartesian
      if (
        entry.cartesian_goal.rotation.x != temp.x ||
        entry.cartesian_goal.rotation.y != temp.y ||
        entry.cartesian_goal.rotation.z != temp.z ||
        entry.cartesian_goal.rotation.w != temp.w) {
        transform_to_angelaxis(entry.cartesian_goal, parameter);
      } else {
        parameter.assign(
          entry.joint_goal.begin(), entry.joint_goal.end());
      }

      parameter.push_back(entry.velocity);
      parameter.push_back(entry.acceleration);
      parameter.push_back(entry.blend);

      ur_rtde::PathEntry pe(
      ur_rtde::PathEntry::eMoveType(entry.move_type),
      ur_rtde::PathEntry::ePositionType(entry.position_type),
      parameter);

      path.addEntry(pe);
    }

    response->success = rtde_control_->movePath(path, request->asynchronous);
  } catch (const std::exception & e) {
    RCUTILS_LOG_ERROR_NAMED(
      "URHardwareInterface", "set_trajectory exception: %s", e.what());
    response->success = false;
  }
}

// Force mode
void URHardwareInterface::handle_set_force_target(
  const std::shared_ptr<leitungssatz_interfaces::srv::SetForceTarget::Request> request,
  std::shared_ptr<leitungssatz_interfaces::srv::SetForceTarget::Response> response)
{
  static bool force_mode_active = false;

  std::lock_guard<std::mutex> lock(rtde_control_mutex_);

  if (!rtde_control_ || !rtde_control_->isConnected()) {
    response->success = false;
    return;
  }

  try {
    if (request->io) {
      if (!force_mode_active) {
        while (!rtde_control_->isSteady()) {
          std::this_thread::sleep_for(10ms);
        }
        rtde_control_->zeroFtSensor();
        std::this_thread::sleep_for(100ms);
        RCUTILS_LOG_INFO_NAMED(
          "URHardwareInterface", "Force mode activated");
      }

      std::vector<double> task_frame;
      geometry_msgs::msg::Quaternion temp;
      if (
        request->frame.rotation.x != temp.x ||
        request->frame.rotation.y != temp.y ||
        request->frame.rotation.z != temp.z ||
        request->frame.rotation.w != temp.w) {
        transform_to_angelaxis(request->frame, task_frame);
      } else {
        task_frame = {0, 0, 0, 0, 0, 0};
      }

      last_motion_ = 3;
      response->success = rtde_control_->forceMode(
        task_frame,
        std::vector<int>(
          request->selection_vector.begin(),
          request->selection_vector.end()),
        std::vector<double>(
          request->wrench.begin(), request->wrench.end()),
        request->type,
        std::vector<double>(
          request->limits.begin(), request->limits.end()));

      force_mode_active = response->success;
    } else {
      if (force_mode_active) {
        response->success = rtde_control_->forceModeStop();
        if (response->success) {
          last_motion_ = 0;
          force_mode_active = false;
          RCUTILS_LOG_INFO_NAMED(
            "URHardwareInterface", "Force mode deactivated");
        }
      } else {
        response->success = true;
      }
    }
  } catch (const std::exception & e) {
    RCUTILS_LOG_ERROR_NAMED(
      "URHardwareInterface", "set_force_target exception: %s", e.what());
    response->success = false;
  }
}

// Contact target
void URHardwareInterface::handle_set_contact_target(
  const std::shared_ptr<leitungssatz_interfaces::srv::SetContactTarget::Request> request,
  std::shared_ptr<leitungssatz_interfaces::srv::SetContactTarget::Response> response)
{
  std::lock_guard<std::mutex> lock(rtde_control_mutex_);

  if (!rtde_control_ || !rtde_control_->isConnected()) {
    response->success = false;
    return;
  }

  try {
    response->success = rtde_control_->moveUntilContact(
      std::vector<double>(request->xd.begin(), request->xd.end()),
      std::vector<double>(request->direction.begin(), request->direction.end()),
      request->acceleration);
  } catch (const std::exception & e) {
    RCUTILS_LOG_ERROR_NAMED(
      "URHardwareInterface", "set_contact_target exception: %s", e.what());
    response->success = false;
  }
}

// Start / stop jog
void URHardwareInterface::handle_start_jog(
  const std::shared_ptr<leitungssatz_interfaces::srv::StartJog::Request> request,
  std::shared_ptr<leitungssatz_interfaces::srv::StartJog::Response> response)
{
  std::lock_guard<std::mutex> lock(rtde_control_mutex_);

  if (!rtde_control_ || !rtde_control_->isConnected()) {
    response->success = false;
    return;
  }

  try {
    if (request->io) {
      last_motion_ = 5;
      if (!jog_control_sub_) {
        jog_control_sub_ = node_->create_subscription<leitungssatz_interfaces::msg::JogControl>(
          "jog_control", 1000,
          std::bind(
            &URHardwareInterface::jog_control_callback, this,
            std::placeholders::_1));
      }
    } else {
      if (jog_control_sub_) {
        jog_control_sub_.reset();
      }
      rtde_control_->jogStop();
      last_motion_ = 0;
    }
    response->success = true;
  } catch (const std::exception & e) {
    RCUTILS_LOG_ERROR_NAMED(
      "URHardwareInterface", "start_jog exception: %s", e.what());
    response->success = false;
  }
}

// Zero FT sensor
void URHardwareInterface::handle_zero_ftsensor(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  std::lock_guard<std::mutex> lock(rtde_control_mutex_);

  if (!rtde_control_ || !rtde_control_->isConnected()) {
    response->success = false;
    response->message = "RTDE control not connected";
    return;
  }

  try {
    while (!rtde_control_->isSteady()) {
      std::this_thread::sleep_for(10ms);
    }
    rtde_control_->zeroFtSensor();
    response->success = true;
    response->message = "FT sensor zeroed";
  } catch (const std::exception & e) {
    RCUTILS_LOG_ERROR_NAMED(
      "URHardwareInterface", "zero_ftsensor exception: %s", e.what());
    response->success = false;
    response->message = e.what();
  }
}

// Logging
void URHardwareInterface::handle_log(
  const std::shared_ptr<leitungssatz_interfaces::srv::Log::Request> request,
  std::shared_ptr<leitungssatz_interfaces::srv::Log::Response> response)
{
  if (!rtde_receive_ || !rtde_receive_->isConnected()) {
    response->success = false;
    return;
  }

  try {
    if (request->io) {
      if (request->file_name.empty()) {
        RCUTILS_LOG_WARN_NAMED(
          "URHardwareInterface", "log filename is missing");
        response->success = false;
        return;
      }
      response->success =
        rtde_receive_->startFileRecording(request->file_name + ".csv");
    } else {
      response->success = rtde_receive_->stopFileRecording();
    }
  } catch (const std::exception & e) {
    RCUTILS_LOG_ERROR_NAMED(
      "URHardwareInterface", "log exception: %s", e.what());
    response->success = false;
  }
}

// Payload
void URHardwareInterface::handle_set_payload(
  const std::shared_ptr<leitungssatz_interfaces::srv::SetPayload::Request> request,
  std::shared_ptr<leitungssatz_interfaces::srv::SetPayload::Response> response)
{
  std::lock_guard<std::mutex> lock(rtde_control_mutex_);

  if (!rtde_control_ || !rtde_control_->isConnected()) {
    response->success = false;
    return;
  }

  try {
    response->success = rtde_control_->setPayload(
      request->mass,
      std::vector<double>(request->cog.begin(), request->cog.end()));
  } catch (const std::exception & e) {
    RCUTILS_LOG_ERROR_NAMED(
      "URHardwareInterface", "set_payload exception: %s", e.what());
    response->success = false;
  }
}

// Get timestamp
void URHardwareInterface::handle_get_timestamp(
  const std::shared_ptr<leitungssatz_interfaces::srv::GetTimeStamp::Request>,
  std::shared_ptr<leitungssatz_interfaces::srv::GetTimeStamp::Response> response)
{
  if (!rtde_receive_ || !rtde_receive_->isConnected()) {
    response->time_stamp = 0.0;
    return;
  }

  try {
    response->time_stamp = rtde_receive_->getTimestamp();
  } catch (const std::exception & e) {
    RCUTILS_LOG_ERROR_NAMED(
      "URHardwareInterface", "get_timestamp exception: %s", e.what());
    response->time_stamp = 0.0;
  }
}

// Stop motion
void URHardwareInterface::handle_stop_motion(
  const std::shared_ptr<leitungssatz_interfaces::srv::StopMotion::Request> request,
  std::shared_ptr<leitungssatz_interfaces::srv::StopMotion::Response> response)
{
  std::lock_guard<std::mutex> lock(rtde_control_mutex_);

  if (!rtde_control_ || !rtde_control_->isConnected()) {
    response->success = false;
    return;
  }

  try {
    if (last_motion_ == 2) {
      rtde_control_->stopJ(request->acceleration);
      response->success = true;
    } else if (last_motion_ == 1) {
      rtde_control_->stopL(request->acceleration);
      response->success = true;
    } else {
      response->success = false;
    }
  } catch (const std::exception & e) {
    RCUTILS_LOG_ERROR_NAMED(
      "URHardwareInterface", "stop_motion exception: %s", e.what());
    response->success = false;
  }
}

// Clip gun IO script
void URHardwareInterface::handle_set_clip_gun(
  const std::shared_ptr<leitungssatz_interfaces::srv::SetClipGun::Request> request,
  std::shared_ptr<leitungssatz_interfaces::srv::SetClipGun::Response> response)
{
  std::lock_guard<std::mutex> lock(rtde_control_mutex_);

  if (!rtde_control_ || !rtde_control_->isConnected()) {
    RCUTILS_LOG_ERROR_NAMED(
      "URHardwareInterface", "set_clip_gun: RTDE control interface not connected");
    response->success = false;
    return;
  }

  try {
    std::string script =
      "def set_io():\n"
      "  set_standard_digital_out(" + std::to_string(4) + ", " +
      (request->io ? std::string("True") : std::string("False")) + ")\n"
      "end\n"
      "set_io()\n";

    RCUTILS_LOG_INFO_NAMED(
      "URHardwareInterface", "Sending script to robot: %s", script.c_str());

    response->success =
      rtde_control_->sendCustomScriptFunction("set_io", script);

    if (response->success) {
      RCUTILS_LOG_INFO_NAMED(
        "URHardwareInterface", "Clip gun set to %s",
        request->io ? "True" : "False");
    } else {
      RCUTILS_LOG_ERROR_NAMED(
        "URHardwareInterface", "Failed to set clip gun state");
    }
  } catch (const std::exception & e) {
    RCUTILS_LOG_ERROR_NAMED(
      "URHardwareInterface", "set_clip_gun exception: %s", e.what());
    response->success = false;
  }
}

// Add TF2 (stub: store_tf equivalent – fill in as needed)
void URHardwareInterface::handle_add_tf2(
  const std::shared_ptr<leitungssatz::srv::AddTf2::Request>,
  std::shared_ptr<leitungssatz::srv::AddTf2::Response> response)
{
  // Implement if you had equivalent functionality in ROS1 (e.g. store frames)
  response->success = true;
}

// ------------------ Jog callback ------------------

void URHardwareInterface::jog_control_callback(
  const leitungssatz_interfaces::msg::JogControl::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(rtde_control_mutex_);

  if (!rtde_control_ || !rtde_control_->isConnected()) {
    return;
  }

  try {
    // timeout: if message is older than 3 seconds, stop
    rclcpp::Time now = node_->get_clock()->now();
    rclcpp::Time stamp = msg->stamp;
    if ((now - stamp) > rclcpp::Duration(3, 0)) {
      rtde_control_->jogStop();
      return;
    }

    std::vector<double> speeds(msg->speeds.begin(), msg->speeds.end());
    double acc = 0.5;  // same as default in header, or tune

    if (msg->feature == 1) {
      rtde_control_->jogStart(
        speeds, ur_rtde::RTDEControlInterface::FEATURE_BASE, acc);
    } else if (msg->feature == 2) {
      rtde_control_->jogStart(
        speeds, ur_rtde::RTDEControlInterface::FEATURE_TOOL, acc);
    } else if (msg->feature == 3) {
      std::vector<double> custom_frame(
        msg->custom_frame.begin(), msg->custom_frame.end());
      rtde_control_->jogStart(
        speeds,
        ur_rtde::RTDEControlInterface::FEATURE_CUSTOM,
        acc,
        custom_frame);
    } else {
      rtde_control_->jogStop();
    }
  } catch (const std::exception & e) {
    RCUTILS_LOG_ERROR_NAMED(
      "URHardwareInterface", "jog_control_callback exception: %s", e.what());
  }
}

PLUGINLIB_EXPORT_CLASS(leitungssatz_interfaces::URHardwareInterface, hardware_interface::SystemInterface)

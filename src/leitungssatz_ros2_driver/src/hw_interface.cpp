#include "leitungssatz_ros2_driver/hw_interface.hpp"

#include <chrono>
#include <thread>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.hpp>

using namespace std::chrono_literals;
using leitungssatz_interfaces::URHardwareInterface;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

URHardwareInterface::URHardwareInterface()
{
}

URHardwareInterface::~URHardwareInterface()
{
  // stop executor
  if (executor_) {
    executor_->cancel();
  }
  if (executor_thread_.joinable()) executor_thread_.join();

  // stop RTDE thread & disconnect
  rtde_thread_running_ = false;
  if (rtde_thread_.joinable()) rtde_thread_.join();
  disconnect_from_robot();

  if (dashboard_) dashboard_->shutdown();
  if (gripper_) gripper_->shutdown();
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
    RCUTILS_LOG_WARN_NAMED("URHardwareInterface", "Expected 6 joints, got %zu", joint_names_.size());
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
    if (p.first == "robot_ip") robot_ip_ = p.second;
    if (p.first == "use_dashboard") use_dashboard_ = (p.second == "true" || p.second == "1");
    if (p.first == "use_robotiq") use_robotiq_ = (p.second == "true" || p.second == "1");
  }

  RCUTILS_LOG_INFO_NAMED("URHardwareInterface", "on_init successful; robot_ip=%s", robot_ip_.c_str());
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> URHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(joint_names_[i], "position", &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(joint_names_[i], "velocity", &hw_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(joint_names_[i], "effort", &hw_efforts_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> URHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(joint_names_[i], "position", &hw_position_commands_[i]));
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
  executor_thread_ = std::thread([this]() { executor_->spin(); });

  // Publishers use same topic names as your migrated point_recording expects
  tcp_pose_pub_ = node_->create_publisher<geometry_msgs::msg::TransformStamped>("/ur_hardware_interface/tcp_pose", 10);
  wrench_pub_ = node_->create_publisher<geometry_msgs::msg::WrenchStamped>("/ur_hardware_interface/wrench", 10);
  tcp_info_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>("/ur_hardware_interface/tcp_info", 10);

  // Service servers with same names as point_recording expects
  srv_set_freedrive_ = node_->create_service<leitungssatz_interfaces::srv::SetFreedrive>("/ur_hardware_interface/set_freedrive",
    std::bind(&URHardwareInterface::handle_set_freedrive, this, std::placeholders::_1, std::placeholders::_2));
  srv_set_cart_target_ = node_->create_service<leitungssatz_interfaces::srv::SetCartTarget>("/ur_hardware_interface/set_cart_target",
    std::bind(&URHardwareInterface::handle_set_cart_target, this, std::placeholders::_1, std::placeholders::_2));
  srv_start_jog_ = node_->create_service<leitungssatz_interfaces::srv::StartJog>("/ur_hardware_interface/start_jog",
    std::bind(&URHardwareInterface::handle_start_jog, this, std::placeholders::_1, std::placeholders::_2));
  srv_set_force_target_ = node_->create_service<leitungssatz_interfaces::srv::SetForceTarget>("/ur_hardware_interface/set_force_mode",
    std::bind(&URHardwareInterface::handle_set_force_target, this, std::placeholders::_1, std::placeholders::_2));
  srv_set_gripper_ = node_->create_service<leitungssatz_interfaces::srv::SetGripper>("/ur_hardware_interface/robotiq/set_gripper",
    std::bind(&URHardwareInterface::handle_set_gripper, this, std::placeholders::_1, std::placeholders::_2));
  srv_add_tf2_ = node_->create_service<leitungssatz::srv::AddTf2>("/store_tf",
    std::bind(&URHardwareInterface::handle_add_tf2, this, std::placeholders::_1, std::placeholders::_2));

  // Newly added service servers (restore ROS1 parity)
  srv_set_rel_cart_target_ = node_->create_service<leitungssatz_interfaces::srv::SetRelCartTarget>("/ur_hardware_interface/set_rel_cart_target",
    std::bind(&URHardwareInterface::handle_set_rel_cart_target, this, std::placeholders::_1, std::placeholders::_2));
  srv_set_joint_target_ = node_->create_service<leitungssatz_interfaces::srv::SetJointTarget>("/ur_hardware_interface/set_joint_target",
    std::bind(&URHardwareInterface::handle_set_joint_target, this, std::placeholders::_1, std::placeholders::_2));
  srv_set_rel_joint_target_ = node_->create_service<leitungssatz_interfaces::srv::SetJointTarget>("/ur_hardware_interface/set_rel_joint_target",
    std::bind(&URHardwareInterface::handle_set_rel_joint_target, this, std::placeholders::_1, std::placeholders::_2));
  srv_set_trajectory_ = node_->create_service<leitungssatz_interfaces::srv::SetTrajectory>("/ur_hardware_interface/set_trajectory",
    std::bind(&URHardwareInterface::handle_set_trajectory, this, std::placeholders::_1, std::placeholders::_2));
  srv_set_contact_target_ = node_->create_service<leitungssatz_interfaces::srv::SetContactTarget>("/ur_hardware_interface/set_contact_target",
    std::bind(&URHardwareInterface::handle_set_contact_target, this, std::placeholders::_1, std::placeholders::_2));
  srv_log_ = node_->create_service<leitungssatz_interfaces::srv::Log>("/ur_hardware_interface/logging",
    std::bind(&URHardwareInterface::handle_log, this, std::placeholders::_1, std::placeholders::_2));
  srv_get_timestamp_ = node_->create_service<leitungssatz_interfaces::srv::GetTimeStamp>("/ur_hardware_interface/get_robot_timestamp",
    std::bind(&URHardwareInterface::handle_get_timestamp, this, std::placeholders::_1, std::placeholders::_2));
  srv_set_payload_ = node_->create_service<leitungssatz_interfaces::srv::SetPayload>("/ur_hardware_interface/set_payload",
    std::bind(&URHardwareInterface::handle_set_payload, this, std::placeholders::_1, std::placeholders::_2));
  srv_stop_motion_ = node_->create_service<leitungssatz_interfaces::srv::StopMotion>("/ur_hardware_interface/stop_motion",
    std::bind(&URHardwareInterface::handle_stop_motion, this, std::placeholders::_1, std::placeholders::_2));
  srv_set_clip_gun_ = node_->create_service<leitungssatz_interfaces::srv::SetClipGun>("/ur_hardware_interface/set_clip_gun",
    std::bind(&URHardwareInterface::handle_set_clip_gun, this, std::placeholders::_1, std::placeholders::_2));
  srv_zero_ftsensor_ = node_->create_service<std_srvs::srv::Trigger>("/ur_hardware_interface/zero_ftsensor",
    std::bind(&URHardwareInterface::handle_zero_ftsensor, this, std::placeholders::_1, std::placeholders::_2));

  // Connect to robot (instantiate RTDE objects)
  if (!connect_to_robot()) {
    RCUTILS_LOG_ERROR_NAMED("URHardwareInterface", "Failed to connect to robot");
    return CallbackReturn::ERROR;
  }

  // initialize optional helpers
  if (use_dashboard_) {
    dashboard_ = std::make_unique<DashboardClient>();
    if (!dashboard_->init(node_, robot_ip_)) {
      RCUTILS_LOG_WARN_NAMED("URHardwareInterface", "Dashboard init failed (continuing without dashboard)");
      dashboard_.reset();
    } else {
      // wait until robot is in remote control (like ROS1)
      size_t attempts = 0;
      while (rclcpp::ok() && dashboard_->isInRemoteControl() == false && attempts++ < 20) {
        RCUTILS_LOG_WARN_NAMED("URHardwareInterface", "Please set Robot to remote control (dashboard)");
        std::this_thread::sleep_for(500ms);
      }
    }
  }

  if (use_robotiq_) {
    gripper_ = std::make_unique<RobotiqGripperInterface>();
    if (!gripper_->init(node_, robot_ip_)) {
      RCUTILS_LOG_WARN_NAMED("URHardwareInterface", "Robotiq init failed (continuing without gripper)");
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

hardware_interface::return_type URHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
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

hardware_interface::return_type URHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Commands should be handed off to RTDE thread via a queue or protected call.
  // For now this is a placeholder; implement a command queue or protected non-blocking call if controllers will send joint commands.
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
      } catch (...) {}
    }

    RCUTILS_LOG_INFO_NAMED("URHardwareInterface", "Connected to RTDE at %s", robot_ip_.c_str());
    return true;
  } catch (const std::exception & e) {
    RCUTILS_LOG_ERROR_NAMED("URHardwareInterface", "connect_to_robot exception: %s", e.what());
    return false;
  }
}

void URHardwareInterface::disconnect_from_robot()
{
  // stop RTDE thread (already signalled by destructor)
  try {
    if (rtde_control_) {
      rtde_control_.reset();
    }
    if (rtde_receive_) {
      rtde_receive_.reset();
    }
  } catch (...) {}
}

// RTDE thread: non-realtime loop that polls receive interface and updates realtime buffers
void URHardwareInterface::rtde_thread_loop()
{
  RCUTILS_LOG_INFO_NAMED("URHardwareInterface", "RTDE thread started");
  using namespace std::chrono_literals;

  while (rtde_thread_running_ && rclcpp::ok()) {
    try {
      // Ensure receive/control are connected
      if (!rtde_receive_ || !rtde_receive_->isConnected()) {
        try {
          if (rtde_receive_) rtde_receive_->reconnect();
        } catch (...) {}
      }
      if (!rtde_control_ || !rtde_control_->isConnected()) {
        try {
          if (rtde_control_) rtde_control_->reconnect();
        } catch (...) {}
      }

      // Read data from robot (if available)
      std::array<double,6> q{};
      std::array<double,6> qd{};
      std::array<double,6> force{};

      if (rtde_receive_ && rtde_receive_->isConnected()) {
        auto qvec = rtde_receive_->getActualQ();
        auto qdvec = rtde_receive_->getActualQd();
        auto fvec = rtde_receive_->getActualTCPForce();
        auto tcp_pose_vec = rtde_receive_->getActualTCPPose(); // [x,y,z,rx,ry,rz]
        auto tcp_speed_vec = rtde_receive_->getActualTCPSpeed();

        for (size_t i=0;i<6 && i<qvec.size(); ++i) q[i] = qvec[i];
        for (size_t i=0;i<6 && i<qdvec.size(); ++i) qd[i] = qdvec[i];
        for (size_t i=0;i<6 && i<fvec.size(); ++i) force[i] = fvec[i];

        // prepare and publish tcp_pose
        geometry_msgs::msg::TransformStamped tcp_msg;
        tcp_msg.header.stamp = node_->get_clock()->now();
        tcp_msg.header.frame_id = "base";
        tcp_msg.child_frame_id = "tool0";

        // tcp_pose_vec is [x,y,z, rx,ry,rz] where rx,ry,rz is rotation vector (axis * angle)
        if (tcp_pose_vec.size() >= 6) {
          tcp_msg.transform.translation.x = tcp_pose_vec[0];
          tcp_msg.transform.translation.y = tcp_pose_vec[1];
          tcp_msg.transform.translation.z = tcp_pose_vec[2];
          // convert rotation-vector to quaternion
          Eigen::Vector3d rotVec(tcp_pose_vec[3], tcp_pose_vec[4], tcp_pose_vec[5]);
          double theta = rotVec.norm();
          Eigen::Quaterniond qrot = Eigen::Quaterniond::Identity();
          if (theta > 1e-9) {
            Eigen::Vector3d axis = rotVec / theta;
            qrot = Eigen::Quaterniond(Eigen::AngleAxisd(theta, axis));
          }
          tf2::Quaternion qt;
          qt.setX(qrot.x());
          qt.setY(qrot.y());
          qt.setZ(qrot.z());
          qt.setW(qrot.w());
          tcp_msg.transform.rotation.x = qt.x();
          tcp_msg.transform.rotation.y = qt.y();
          tcp_msg.transform.rotation.z = qt.z();
          tcp_msg.transform.rotation.w = qt.w();
        }

        // write to realtime buffers
        rtde_positions_buffer_.writeFromNonRT(q);
        rtde_velocities_buffer_.writeFromNonRT(qd);
        rtde_efforts_buffer_.writeFromNonRT(force);

        tcp_pose_buffer_.writeFromNonRT(tcp_msg);

        // publish non-RT topics
        if (tcp_pose_pub_) tcp_pose_pub_->publish(tcp_msg);

        // publish wrench if needed
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
        }

        // publish tcp_info if needed (uses tcp speed)
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
        }
      } else {
        // not connected: write zeros to buffers so read() sees something
        rtde_positions_buffer_.writeFromNonRT(q);
        rtde_velocities_buffer_.writeFromNonRT(qd);
        rtde_efforts_buffer_.writeFromNonRT(force);
      }
    } catch (const std::exception & e) {
      RCUTILS_LOG_ERROR_NAMED("URHardwareInterface", "RTDE thread exception: %s", e.what());
    }
    std::this_thread::sleep_for(10ms);
  }
  RCUTILS_LOG_INFO_NAMED("URHardwareInterface", "RTDE thread stopping");
}

// ------------------ Service handlers (non-realtime) ------------------

// Freedrive handler (existing)
void URHardwareInterface::handle_set_freedrive(
  const std::shared_ptr<leitungssatz_interfaces::srv::SetFreedrive::Request> request,
  std::shared_ptr<leitungssatz_interfaces::srv::SetFreedrive::Response> response)
{
  std::lock_guard<std::mutex> lock(rtde_control_mutex_);
  try {
    if (!rtde_control_ || !rtde_control_->isConnected()) {
      response->success = false;
      response->freedrive_status = 0;
      response->message = "not connected";
      return;
    }

    if (request->io) { // start freedrive
      last_motion_ = 4;
      std::vector<double> frame;
      switch (request->feature) {
        case 0:
          frame = {0,0,0,0,0,0};
          break;
        case 1:
          frame = rtde_receive_->getActualTCPPose();
          break;
        case 2:
          transform_to_angelaxis(request->custom_frame, frame);
          break;
        default:
          frame = {0,0,0,0,0,0};
      }

      std::vector<int> free_axes_vec(request->free_axes.begin(), request->free_axes.end());
      response->success = rtde_control_->freedriveMode(free_axes_vec, frame);
      if (response->success) RCUTILS_LOG_INFO_NAMED("URHardwareInterface", "Freedrive Mode Activated");
    } else { // end freedrive
      response->success = rtde_control_->endFreedriveMode();
      if (response->success) {
        last_motion_ = 0;
        RCUTILS_LOG_INFO_NAMED("URHardwareInterface", "Freedrive Mode Deactivated");
      }
    }

    // Provide freedrive status
    if (rtde_control_) {
      response->freedrive_status = rtde_control_->getFreedriveStatus();
    } else {
      response->freedrive_status = 0;
    }
    response->message = "";
  } catch (const std::exception & e) {
    RCUTILS_LOG_ERROR_NAMED("URHardwareInterface", "set_freedrive exception: %s", e.what());
    response->success = false;
    response->freedrive_status = 0;
    response->message = e.what();
  }
}

// set_cart_target (existing)
void URHardwareInterface::handle_set_cart_target(
  const std::shared_ptr<leitungssatz_interfaces::srv::SetCartTarget::Request> request,
  std::shared_ptr<leitungssatz_interfaces::srv::SetCartTarget::Response> response)
{
  std::lock_guard<std::mutex> lock(rtde_control_mutex_);
  try {
    if (!rtde_control_ || !rtde_control_->isConnected()) {
      response->success = false;
      return;
    }

    std::vector<double> pose;
    transform_to_angelaxis(request->cartesian_goal, pose);

    if (!rtde_control_->isPoseWithinSafetyLimits(pose)) {
      RCUTILS_LOG_WARN_NAMED("URHardwareInterface", "Target is out of range or safety limits");
      response->success = false;
      return;
    }

    bool success = false;
    switch (request->mode) {
      case 1:
        last_motion_ = 1;
        success = rtde_control_->moveL(pose, request->speed, request->acceleration, request->asynchronous);
        break;
      case 2:
        last_motion_ = 1;
        success = rtde_control_->moveJ_IK(pose, request->speed, request->acceleration, request->asynchronous);
        break;
      default:
        success = false;
        break;
    }

    // keep ROS1 asynchronous semantics: accept async requests as success if accepted
    if (request->asynchronous) {
      response->success = true;
    } else {
      response->success = success;
    }
  } catch (const std::exception & e) {
    RCUTILS_LOG_ERROR_NAMED("URHardwareInterface", "set_cart_target exception: %s", e.what());
    response->success = false;
  }
}

// start_jog (existing)
void URHardwareInterface::handle_start_jog(
  const std::shared_ptr<leitungssatz_interfaces::srv::StartJog::Request> request,
  std::shared_ptr<leitungssatz_interfaces::srv::StartJog::Response> response)
{
  std::lock_guard<std::mutex> lock(rtde_control_mutex_);
  try {
    if (!rtde_control_ || !rtde_control_->isConnected()) {
      response->success = false;
      return;
    }

    if (request->io) {
      last_motion_ = 5;
      // create subscription if not present
      if (!jog_control_sub_) {
        jog_control_sub_ = node_->create_subscription<leitungssatz_interfaces::msg::JogControl>(
          "/jog_control", rclcpp::QoS(10),
          std::bind(&URHardwareInterface::jog_control_callback, this, std::placeholders::_1));
      }
    } else {
      // stop jogging
      if (jog_control_sub_) {
        jog_control_sub_.reset();
      }
      rtde_control_->jogStop();
      last_motion_ = 0;
    }
    response->success = true;
  } catch (const std::exception & e) {
    RCUTILS_LOG_ERROR_NAMED("URHardwareInterface", "start_jog exception: %s", e.what());
    response->success = false;
  }
}

// set_force_target (existing)
void URHardwareInterface::handle_set_force_target(
  const std::shared_ptr<leitungssatz_interfaces::srv::SetForceTarget::Request> request,
  std::shared_ptr<leitungssatz_interfaces::srv::SetForceTarget::Response> response)
{
  std::lock_guard<std::mutex> lock(rtde_control_mutex_);
  try {
    if (!rtde_control_ || !rtde_control_->isConnected()) {
      response->success = false;
      return;
    }

    if (request->io) {
      std::vector<double> task_frame;
      geometry_msgs::msg::Quaternion tmp{};
      if (request->frame.rotation.x != tmp.x ||
          request->frame.rotation.y != tmp.y ||
          request->frame.rotation.z != tmp.z ||
          request->frame.rotation.w != tmp.w) {
        transform_to_angelaxis(request->frame, task_frame);
      } else {
        task_frame = {0,0,0,0,0,0};
      }

      last_motion_ = 3;
      response->success = rtde_control_->forceMode(
        task_frame,
        std::vector<int>(request->selection_vector.begin(), request->selection_vector.end()),
        std::vector<double>(request->wrench.begin(), request->wrench.end()),
        static_cast<int>(request->type),
        std::vector<double>(request->limits.begin(), request->limits.end()));
    } else {
      response->success = rtde_control_->forceModeStop();
      if (response->success) last_motion_ = 0;
    }
  } catch (const std::exception & e) {
    RCUTILS_LOG_ERROR_NAMED("URHardwareInterface", "set_force_target exception: %s", e.what());
    response->success = false;
  }
}

// set_gripper (existing; forwards to Robotiq if available)
void URHardwareInterface::handle_set_gripper(
  const std::shared_ptr<leitungssatz_interfaces::srv::SetGripper::Request> request,
  std::shared_ptr<leitungssatz_interfaces::srv::SetGripper::Response> response)
{
  std::lock_guard<std::mutex> lock(rtde_control_mutex_);
  try {
    if (gripper_) {
      // forward to gripper object (it will ensure connection)
      (void)gripper_->set_gripper(request, response);
    } else {
      // stub behavior (as before)
      response->e_object_status = 0;
      response->success = true;
    }
  } catch (const std::exception & e) {
    RCUTILS_LOG_ERROR_NAMED("URHardwareInterface", "set_gripper exception: %s", e.what());
    response->success = false;
    response->e_object_status = 0;
  }
}

// store_tf (existing)
void URHardwareInterface::handle_add_tf2(
  const std::shared_ptr<leitungssatz::srv::AddTf2::Request> request,
  std::shared_ptr<leitungssatz::srv::AddTf2::Response> response)
{
  (void)request;
  response->success = true;
}

// ---------------- Newly added handler implementations ----------------

// set_rel_cart_target
void URHardwareInterface::handle_set_rel_cart_target(
  const std::shared_ptr<leitungssatz_interfaces::srv::SetRelCartTarget::Request> request,
  std::shared_ptr<leitungssatz_interfaces::srv::SetRelCartTarget::Response> response)
{
  std::lock_guard<std::mutex> lock(rtde_control_mutex_);
  try {
    if (!rtde_control_ || !rtde_control_->isConnected() || !rtde_receive_ || !rtde_receive_->isConnected()) {
      response->success = false;
      return;
    }

    // Get current TCP pose from RTDE and convert to Transform
    auto tcp_vec = rtde_receive_->getActualTCPPose();
    geometry_msgs::msg::Transform current_tf;
    if (tcp_vec.size() >= 6) {
      current_tf.translation.x = tcp_vec[0];
      current_tf.translation.y = tcp_vec[1];
      current_tf.translation.z = tcp_vec[2];
      Eigen::Vector3d rotVec(tcp_vec[3], tcp_vec[4], tcp_vec[5]);
      double theta = rotVec.norm();
      Eigen::Quaterniond qrot = Eigen::Quaterniond::Identity();
      if (theta > 1e-9) {
        qrot = Eigen::Quaterniond(Eigen::AngleAxisd(theta, rotVec.normalized()));
      }
      current_tf.rotation.x = qrot.x();
      current_tf.rotation.y = qrot.y();
      current_tf.rotation.z = qrot.z();
      current_tf.rotation.w = qrot.w();
    }

    // apply relative translation
    geometry_msgs::msg::Transform new_tf = current_tf;
    if (request->rel_goal.size() >= 6) {
      new_tf.translation.x += request->rel_goal[0];
      new_tf.translation.y += request->rel_goal[1];
      new_tf.translation.z += request->rel_goal[2];
      // relative rotation: build quaternion from small-angle vector
      Eigen::Quaterniond quat_current;
      quat_current.w() = current_tf.rotation.w;
      quat_current.x() = current_tf.rotation.x;
      quat_current.y() = current_tf.rotation.y;
      quat_current.z() = current_tf.rotation.z;
      Eigen::Quaterniond quat_rel = Eigen::AngleAxisd(request->rel_goal[3], Eigen::Vector3d::UnitX())
                                 * Eigen::AngleAxisd(request->rel_goal[4], Eigen::Vector3d::UnitY())
                                 * Eigen::AngleAxisd(request->rel_goal[5], Eigen::Vector3d::UnitZ());
      Eigen::Quaterniond quat_new = quat_current * quat_rel;
      new_tf.rotation.x = quat_new.x();
      new_tf.rotation.y = quat_new.y();
      new_tf.rotation.z = quat_new.z();
      new_tf.rotation.w = quat_new.w();
    }

    std::vector<double> pose;
    transform_to_angelaxis(new_tf, pose);

    if (!rtde_control_->isPoseWithinSafetyLimits(pose)) {
      RCUTILS_LOG_WARN_NAMED("URHardwareInterface", "Relative target out of safety limits");
      response->success = false;
      return;
    }

    bool success = false;
    switch (request->mode) {
      case 1:
        last_motion_ = 1;
        success = rtde_control_->moveL(pose, request->speed, request->acceleration, request->asynchronous);
        break;
      case 2:
        last_motion_ = 1;
        success = rtde_control_->moveJ_IK(pose, request->speed, request->acceleration, request->asynchronous);
        break;
      default:
        success = false;
        break;
    }

    response->success = request->asynchronous ? true : success;
  } catch (const std::exception & e) {
    RCUTILS_LOG_ERROR_NAMED("URHardwareInterface", "set_rel_cart_target exception: %s", e.what());
    response->success = false;
  }
}

// set_joint_target
void URHardwareInterface::handle_set_joint_target(
  const std::shared_ptr<leitungssatz_interfaces::srv::SetJointTarget::Request> request,
  std::shared_ptr<leitungssatz_interfaces::srv::SetJointTarget::Response> response)
{
  std::lock_guard<std::mutex> lock(rtde_control_mutex_);
  try {
    if (!rtde_control_ || !rtde_control_->isConnected()) {
      response->success = false;
      return;
    }

    std::vector<double> joints(request->joint_goal.begin(), request->joint_goal.end());
    if (!rtde_control_->isJointsWithinSafetyLimits(joints)) {
      RCUTILS_LOG_WARN_NAMED("URHardwareInterface", "Joint target out of limits");
      response->success = false;
      return;
    }

    last_motion_ = 2;
    bool ok = rtde_control_->moveJ(joints, request->speed, request->acceleration, request->asynchronous);
    response->success = request->asynchronous ? true : ok;
  } catch (const std::exception & e) {
    RCUTILS_LOG_ERROR_NAMED("URHardwareInterface", "set_joint_target exception: %s", e.what());
    response->success = false;
  }
}

// set_rel_joint_target
void URHardwareInterface::handle_set_rel_joint_target(
  const std::shared_ptr<leitungssatz_interfaces::srv::SetJointTarget::Request> request,
  std::shared_ptr<leitungssatz_interfaces::srv::SetJointTarget::Response> response)
{
  std::lock_guard<std::mutex> lock(rtde_control_mutex_);
  try {
    if (!rtde_control_ || !rtde_control_->isConnected() || !rtde_receive_ || !rtde_receive_->isConnected()) {
      response->success = false;
      return;
    }

    auto current_q = rtde_receive_->getActualQ();
    std::vector<double> target(6, 0.0);
    for (size_t i=0;i<6 && i<current_q.size() && i<request->joint_goal.size(); ++i) {
      target[i] = current_q[i] + request->joint_goal[i];
    }

    if (!rtde_control_->isJointsWithinSafetyLimits(target)) {
      RCUTILS_LOG_WARN_NAMED("URHardwareInterface", "Relative joint target out of limits");
      response->success = false;
      return;
    }

    last_motion_ = 2;
    bool ok = rtde_control_->moveJ(target, request->speed, request->acceleration, request->asynchronous);
    response->success = request->asynchronous ? true : ok;
  } catch (const std::exception & e) {
    RCUTILS_LOG_ERROR_NAMED("URHardwareInterface", "set_rel_joint_target exception: %s", e.what());
    response->success = false;
  }
}

// set_trajectory (Path handling)
void URHardwareInterface::handle_set_trajectory(
  const std::shared_ptr<leitungssatz_interfaces::srv::SetTrajectory::Request> request,
  std::shared_ptr<leitungssatz_interfaces::srv::SetTrajectory::Response> response)
{
  std::lock_guard<std::mutex> lock(rtde_control_mutex_);
  try {
    // Build ur_rtde Path similar to ROS1
    ur_rtde::Path path;
    for (size_t i=0;i<request->path.size(); ++i) {
      std::vector<double> parameter;
      // if cartesian goal present (using rotation default check)
      if (request->path[i].cartesian_goal.rotation.w != 0.0 ||
          request->path[i].cartesian_goal.rotation.x != 0.0 ||
          request->path[i].cartesian_goal.rotation.y != 0.0 ||
          request->path[i].cartesian_goal.rotation.z != 0.0) {
        geometry_msgs::msg::Transform tf{};
        tf.translation.x = request->path[i].cartesian_goal.translation.x;
        tf.translation.y = request->path[i].cartesian_goal.translation.y;
        tf.translation.z = request->path[i].cartesian_goal.translation.z;
        tf.rotation = request->path[i].cartesian_goal.rotation;
        transform_to_angelaxis(tf, parameter);
      } else {
        parameter = std::vector<double>(request->path[i].joint_goal.begin(), request->path[i].joint_goal.end());
      }
      parameter.push_back(request->path[i].velocity);
      parameter.push_back(request->path[i].acceleration);
      parameter.push_back(request->path[i].blend);

      ur_rtde::PathEntry entry(ur_rtde::PathEntry::eMoveType(request->path[i].move_type),
                              ur_rtde::PathEntry::ePositionType(request->path[i].position_type),
                              parameter);
      path.addEntry(entry);
    }

    // call movePath
    response->success = rtde_control_->movePath(path, request->asynchronous);
  } catch (const std::exception & e) {
    RCUTILS_LOG_ERROR_NAMED("URHardwareInterface", "set_trajectory exception: %s", e.what());
    response->success = false;
  }
}

// set_contact_target
void URHardwareInterface::handle_set_contact_target(
  const std::shared_ptr<leitungssatz_interfaces::srv::SetContactTarget::Request> request,
  std::shared_ptr<leitungssatz_interfaces::srv::SetContactTarget::Response> response)
{
  std::lock_guard<std::mutex> lock(rtde_control_mutex_);
  try {
    response->success = rtde_control_->moveUntilContact(
      std::vector<double>(request->xd.begin(), request->xd.end()),
      std::vector<double>(request->direction.begin(), request->direction.end()),
      request->acceleration);
  } catch (const std::exception & e) {
    RCUTILS_LOG_ERROR_NAMED("URHardwareInterface", "set_contact_target exception: %s", e.what());
    response->success = false;
  }
}

// log (start/stop file recording)
void URHardwareInterface::handle_log(
  const std::shared_ptr<leitungssatz_interfaces::srv::Log::Request> request,
  std::shared_ptr<leitungssatz_interfaces::srv::Log::Response> response)
{
  std::lock_guard<std::mutex> lock(rtde_control_mutex_);
  try {
    if (request->io) {
      if (request->file_name.empty())
      {
        RCUTILS_LOG_WARN_NAMED("URHardwareInterface","HW: log filename is missing");
        response->success = false;
        return;
      }
      response->success = rtde_receive_->startFileRecording(request->file_name + ".csv");
    } else {
      response->success = rtde_receive_->stopFileRecording();
    }
  } catch (const std::exception & e) {
    RCUTILS_LOG_ERROR_NAMED("URHardwareInterface", "log exception: %s", e.what());
    response->success = false;
  }
}

// zero_ftsensor (trigger)
void URHardwareInterface::handle_zero_ftsensor(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  std::lock_guard<std::mutex> lock(rtde_control_mutex_);
  try {
    // wait until steady
    while (rtde_control_ && !rtde_control_->isSteady()) {
      std::this_thread::sleep_for(10ms);
    }
    if (rtde_control_) {
      rtde_control_->zeroFtSensor();
      response->success = true;
      response->message = "zeroed";
      return;
    }
  } catch (const std::exception & e) {
    RCUTILS_LOG_ERROR_NAMED("URHardwareInterface", "zero_ftsensor exception: %s", e.what());
  }
  response->success = false;
  response->message = "failed";
}

// set_payload
void URHardwareInterface::handle_set_payload(
  const std::shared_ptr<leitungssatz_interfaces::srv::SetPayload::Request> request,
  std::shared_ptr<leitungssatz_interfaces::srv::SetPayload::Response> response)
{
  std::lock_guard<std::mutex> lock(rtde_control_mutex_);
  try {
    response->success = rtde_control_->setPayload(request->mass, std::vector<double>(request->cog.begin(), request->cog.end()));
  } catch (const std::exception & e) {
    RCUTILS_LOG_ERROR_NAMED("URHardwareInterface", "set_payload exception: %s", e.what());
    response->success = false;
  }
}

// get_timestamp
void URHardwareInterface::handle_get_timestamp(
  const std::shared_ptr<leitungssatz_interfaces::srv::GetTimeStamp::Request> /*request*/,
  std::shared_ptr<leitungssatz_interfaces::srv::GetTimeStamp::Response> response)
{
  try {
    if (rtde_receive_) {
      // note: service field is 'time_stamp' (see GetTimeStamp.srv)
      response->time_stamp = rtde_receive_->getTimestamp();
    } else {
      response->time_stamp = 0.0;
    }
  } catch (const std::exception & e) {
    RCUTILS_LOG_ERROR_NAMED("URHardwareInterface", "get_timestamp exception: %s", e.what());
    response->time_stamp = 0.0;
  }
}

// stop_motion
void URHardwareInterface::handle_stop_motion(
  const std::shared_ptr<leitungssatz_interfaces::srv::StopMotion::Request> request,
  std::shared_ptr<leitungssatz_interfaces::srv::StopMotion::Response> response)
{
  std::lock_guard<std::mutex> lock(rtde_control_mutex_);
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
    RCUTILS_LOG_ERROR_NAMED("URHardwareInterface", "stop_motion exception: %s", e.what());
    response->success = false;
  }
}

// set_clip_gun
void URHardwareInterface::handle_set_clip_gun(
  const std::shared_ptr<leitungssatz_interfaces::srv::SetClipGun::Request> request,
  std::shared_ptr<leitungssatz_interfaces::srv::SetClipGun::Response> response)
{
  std::lock_guard<std::mutex> lock(rtde_control_mutex_);
  try {
    if (!rtde_control_ || !rtde_control_->isConnected()) {
      response->success = false;
      return;
    }

    const int CLIP_GUN_PIN = 4;
    std::string script = "def set_io():\n"
                         "  set_standard_digital_out(" + std::to_string(CLIP_GUN_PIN) + ", " +
                         (request->io ? "True" : "False") + ")\n"
                         "end\n"
                         "set_io()\n";
    response->success = rtde_control_->sendCustomScriptFunction("set_io", script);
  } catch (const std::exception & e) {
    RCUTILS_LOG_ERROR_NAMED("URHardwareInterface", "set_clip_gun exception: %s", e.what());
    response->success = false;
  }
}

// jog callback: forward messages to rtde_control_->jogStart / jogStop
void URHardwareInterface::jog_control_callback(const leitungssatz_interfaces::msg::JogControl::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(rtde_control_mutex_);
  try {
    // check age
    if ((node_->get_clock()->now() - msg->stamp).seconds() > 3.0) {
      if (rtde_control_) rtde_control_->jogStop();
      return;
    }

    std::vector<double> speeds(6, 0.0);
    for (size_t i=0;i<6 && i<msg->speeds.size(); ++i) speeds[i] = msg->speeds[i];

    if (!rtde_control_) return;

    if (msg->feature == 1) {
      rtde_control_->jogStart(speeds, ur_rtde::RTDEControlInterface::FEATURE_BASE);
    } else if (msg->feature == 2) {
      rtde_control_->jogStart(speeds, ur_rtde::RTDEControlInterface::FEATURE_TOOL);
    } else if (msg->feature == 3) {
      std::vector<double> custom_frame(6, 0.0);
      for (size_t i=0;i<6 && i<msg->custom_frame.size(); ++i) custom_frame[i] = msg->custom_frame[i];
      rtde_control_->jogStart(speeds, ur_rtde::RTDEControlInterface::FEATURE_CUSTOM, 0.5, custom_frame);
    } else {
      rtde_control_->jogStop();
    }
  } catch (const std::exception & e) {
    RCUTILS_LOG_ERROR_NAMED("URHardwareInterface", "jog_control_callback exception: %s", e.what());
  }
}

// Helper: convert geometry_msgs Transform to angle-axis vector used by UR RTDE APIs
void URHardwareInterface::transform_to_angelaxis(const geometry_msgs::msg::Transform & transform, std::vector<double>& angleaxis)
{
  Eigen::Quaterniond q;
  q.w() = transform.rotation.w;
  q.x() = transform.rotation.x;
  q.y() = transform.rotation.y;
  q.z() = transform.rotation.z;
  Eigen::AngleAxisd angleAxis(q);
  Eigen::Vector3d axis = angleAxis.axis();
  double angle = angleAxis.angle();
  Eigen::Vector3d rotvec = axis * angle;

  angleaxis.clear();
  angleaxis.push_back(transform.translation.x);
  angleaxis.push_back(transform.translation.y);
  angleaxis.push_back(transform.translation.z);
  angleaxis.push_back(rotvec.x());
  angleaxis.push_back(rotvec.y());
  angleaxis.push_back(rotvec.z());
}
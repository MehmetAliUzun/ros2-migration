#include "leitungssatz_ros2_driver/hw_interface_sim.hpp"

#include <chrono>
#include <cmath>

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(leitungssatz_interfaces::URHardwareInterfaceSim, hardware_interface::SystemInterface)

using namespace std::chrono_literals;
using leitungssatz_interfaces::URHardwareInterfaceSim;

static builtin_interfaces::msg::Duration duration_from_seconds(double seconds)
{
  // Convert double seconds -> builtin_interfaces::msg::Duration (sec + nanosec)
  int64_t total_ns = static_cast<int64_t>(std::lround(seconds * 1e9));
  builtin_interfaces::msg::Duration d;
  d.sec = static_cast<int32_t>(total_ns / 1000000000LL);
  d.nanosec = static_cast<uint32_t>(total_ns % 1000000000LL);
  return d;
}

URHardwareInterfaceSim::URHardwareInterfaceSim()
{
  hw_positions_.fill(0.0);
  hw_velocities_.fill(0.0);
  hw_efforts_.fill(0.0);
  hw_commands_.fill(0.0);
}

URHardwareInterfaceSim::~URHardwareInterfaceSim()
{
}

CallbackReturn URHardwareInterfaceSim::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("URHardwareInterfaceSim"), "Base on_init failed");
    return CallbackReturn::ERROR;
  }
  joint_names_.clear();
  for (const auto & j : info.joints) joint_names_.push_back(j.name);
  if (joint_names_.size() != 6) {
    RCLCPP_WARN(rclcpp::get_logger("URHardwareInterfaceSim"), "Expected 6 joints, got %zu", joint_names_.size());
  }
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> URHardwareInterfaceSim::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> ifaces;
  for (size_t i=0;i<joint_names_.size();++i) {
    ifaces.emplace_back(hardware_interface::StateInterface(joint_names_[i],"position",&hw_positions_[i]));
    ifaces.emplace_back(hardware_interface::StateInterface(joint_names_[i],"velocity",&hw_velocities_[i]));
    ifaces.emplace_back(hardware_interface::StateInterface(joint_names_[i],"effort",&hw_efforts_[i]));
  }
  return ifaces;
}

std::vector<hardware_interface::CommandInterface> URHardwareInterfaceSim::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ifaces;
  for (size_t i=0;i<joint_names_.size();++i) {
    ifaces.emplace_back(hardware_interface::CommandInterface(joint_names_[i],"position",&hw_commands_[i]));
  }
  return ifaces;
}

CallbackReturn URHardwareInterfaceSim::on_configure(const rclcpp_lifecycle::State &)
{
  // create a non-RT node to host services and action client
  node_ = std::make_shared<rclcpp::Node>(node_name_);
  traj_action_client_ = rclcpp_action::create_client<FollowJT>(node_, action_name_);

  // services (only motion-relevant ones for sim)
  srv_set_joint_target_ = node_->create_service<leitungssatz_interfaces::srv::SetJointTarget>(
    "ur_hardware_interface/set_joint_target",
    [this](const std::shared_ptr<leitungssatz_interfaces::srv::SetJointTarget::Request> req,
           std::shared_ptr<leitungssatz_interfaces::srv::SetJointTarget::Response> res)
    {
      // convert request->joint_goal -> trajectory goal
      std::vector<double> positions(req->joint_goal.begin(), req->joint_goal.end());
      if (positions.size() < joint_names_.size()) {
        res->success = false;
        return;
      }
      // compute a nominal time based on speed if provided, else 2s
      double t = 2.0;
      if (req->speed > 1e-6) {
        // rough: use max joint displacement / speed as duration (approx)
        double maxd = 0.0;
        for (size_t i=0;i<joint_names_.size();++i) maxd = std::max(maxd, std::abs(positions[i] - hw_positions_[i]));
        t = std::max(0.5, maxd / req->speed);
      }
      bool accepted=false;
      bool ok = send_joint_goal(positions, t, req->asynchronous, accepted);
      res->success = ok && (req->asynchronous ? accepted : ok);
    });

  srv_set_trajectory_ = node_->create_service<leitungssatz_interfaces::srv::SetTrajectory>(
    "ur_hardware_interface/set_trajectory",
    [this](const std::shared_ptr<leitungssatz_interfaces::srv::SetTrajectory::Request> req,
           std::shared_ptr<leitungssatz_interfaces::srv::SetTrajectory::Response> res)
    {
      // build joint trajectory from path entries that contain joint_goal
      trajectory_msgs::msg::JointTrajectory traj;
      traj.joint_names = joint_names_;
      double time = 0.0;
      for (const auto & entry : req->path) {
        if (entry.joint_goal.size() >= joint_names_.size()) {
          trajectory_msgs::msg::JointTrajectoryPoint pt;
          pt.positions.assign(entry.joint_goal.begin(), entry.joint_goal.end());
          // compute an approximate time increment: if velocity present use it, else fallback
          double dt = (entry.velocity > 1e-6) ? std::max(0.01, 1.0 / entry.velocity) : 0.5;
          time += dt;
          // set time_from_start using msg fields
          pt.time_from_start = duration_from_seconds(time);
          traj.points.push_back(pt);
        }
      }
      if (traj.points.empty()) {
        res->success = false;
        return;
      }
      // send as single goal (use last point as target)
      const auto & last_tf = traj.points.back().time_from_start;
      double last_time_s = static_cast<double>(last_tf.sec) + static_cast<double>(last_tf.nanosec) * 1e-9;
      std::vector<double> lastpos = traj.points.back().positions;
      bool accepted=false;
      bool ok = send_joint_goal(lastpos, last_time_s, req->asynchronous, accepted);
      res->success = ok;
    });

  srv_set_cart_target_ = node_->create_service<leitungssatz_interfaces::srv::SetCartTarget>(
    "ur_hardware_interface/set_cart_target",
    [this](const std::shared_ptr<leitungssatz_interfaces::srv::SetCartTarget::Request> /*req*/,
           std::shared_ptr<leitungssatz_interfaces::srv::SetCartTarget::Response> res)
    {
      // cartesian To IK is not implemented in this sim adapter
      RCLCPP_WARN(node_->get_logger(), "set_cart_target not implemented in sim adapter (use joint targets)");
      res->success = false;
    });

  RCLCPP_INFO(node_->get_logger(), "URHardwareInterfaceSim configured; action client='%s'", action_name_.c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn URHardwareInterfaceSim::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("URHardwareInterfaceSim"), "Activated");
  return CallbackReturn::SUCCESS;
}

CallbackReturn URHardwareInterfaceSim::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("URHardwareInterfaceSim"), "Deactivated");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type URHardwareInterfaceSim::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // In sim we don't poll real robot here; controllers provide joint_states
  // Keep last known hw_positions_ (they remain unchanged by this adapter)
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type URHardwareInterfaceSim::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Commands are sent via action client; nothing to do in write()
  (void)hw_commands_;
  return hardware_interface::return_type::OK;
}

bool URHardwareInterfaceSim::send_joint_goal(const std::vector<double> & positions, double time_s, bool asynchronous, bool & accepted, int timeout_sec)
{
  if (!traj_action_client_) {
    RCLCPP_ERROR(rclcpp::get_logger("URHardwareInterfaceSim"), "Action client not created");
    accepted = false;
    return false;
  }

  // wait for server
  if (!traj_action_client_->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_ERROR(rclcpp::get_logger("URHardwareInterfaceSim"), "Trajectory action server not available");
    accepted = false;
    return false;
  }

  FollowJT::Goal goal_msg;
  goal_msg.trajectory.joint_names = joint_names_;
  trajectory_msgs::msg::JointTrajectoryPoint pt;
  pt.positions = positions;
  // store as message duration
  pt.time_from_start = duration_from_seconds(time_s);
  goal_msg.trajectory.points.push_back(pt);

  // send goal options
  auto send_goal_options = rclcpp_action::Client<FollowJT>::SendGoalOptions();
  send_goal_options.result_callback = [](const GoalHandleFollowJT::WrappedResult & /*result*/) {
    // no-op; synchronous wait below handles results when needed
  };

  // async send
  auto goal_handle_future = traj_action_client_->async_send_goal(goal_msg, send_goal_options);

  // if asynchronous, return once accepted
  if (asynchronous) {
    // check if goal handle was created (accepted)
    auto status = goal_handle_future.wait_for(1s);
    if (status == std::future_status::ready) {
      auto gh = goal_handle_future.get();
      accepted = (gh != nullptr);
      return accepted;
    } else {
      accepted = false;
      return false;
    }
  }

  // synchronous: wait for result up to timeout_sec
  auto gh = goal_handle_future.get();
  if (!gh) {
    RCLCPP_ERROR(rclcpp::get_logger("URHardwareInterfaceSim"), "Goal rejected by action server");
    accepted = false;
    return false;
  }
  accepted = true;

  auto result_future = traj_action_client_->async_get_result(gh);
  auto status = result_future.wait_for(std::chrono::seconds(timeout_sec));
  if (status != std::future_status::ready) {
    RCLCPP_ERROR(rclcpp::get_logger("URHardwareInterfaceSim"), "Goal result timeout");
    return false;
  }
  auto result = result_future.get();
  // check result code (SUCCEEDED)
  return (result.code == rclcpp_action::ResultCode::SUCCEEDED);
}
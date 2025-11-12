#include "leitungssatz/Robot.hpp"

using namespace leitungssatz;

Robot::Robot(std::shared_ptr<rclcpp::Node> node, Gripper& gripper)
    : _gripper(gripper), node_(node)
{
    tfBuffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

    trajectory_request_.asynchronous = false;

    _set_payload = node_->create_client<leitungssatz_interfaces::srv::SetPayload>("/ur_hardware_interface/set_payload");
    cart_move_ = node_->create_client<leitungssatz_interfaces::srv::SetCartTarget>("/ur_hardware_interface/set_cart_target");
    rel_cart_move_ = node_->create_client<leitungssatz_interfaces::srv::SetRelCartTarget>("/ur_hardware_interface/set_rel_cart_target");
    joint_move_ = node_->create_client<leitungssatz_interfaces::srv::SetJointTarget>("/ur_hardware_interface/set_joint_target");
    contact_move_ = node_->create_client<leitungssatz_interfaces::srv::SetContactTarget>("/ur_hardware_interface/set_contact_target");
    move_trajectory_ = node_->create_client<leitungssatz_interfaces::srv::SetTrajectory>("/ur_hardware_interface/set_trajectory");
    _dashboard_speed = node_->create_client<leitungssatz_interfaces::srv::SetSpeedSlider>("/ur_hardware_interface/dashboard/set_speed_slider");
    set_clip_gun_ = node_->create_client<leitungssatz_interfaces::srv::SetClipGun>("/ur_hardware_interface/set_clip_gun");

    logging_ = node_->create_client<leitungssatz_interfaces::srv::Log>("/ur_hardware_interface/logging");
    robot_time_ = node_->create_client<leitungssatz_interfaces::srv::GetTimeStamp>("/ur_hardware_interface/get_robot_timestamp");

    tcp_position_sub_ = node_->create_subscription<geometry_msgs::msg::TransformStamped>(
        "/ur_hardware_interface/tcp_pose", 100,
        std::bind(&Robot::tcp_positon_callback, this, std::placeholders::_1));
    //wrench_sub_ = node_->create_subscription<geometry_msgs::msg::WrenchStamped>(
    //    "/ur_hardware_interface/wrench", 100,
    //    std::bind(&Robot::wrench_callback, this, std::placeholders::_1));
}

void Robot::tcp_positon_callback(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
{
    tcp_position_ = *msg;
}

void Robot::wrench_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
    wrench_ = *msg;
}

leitungssatz_interfaces::srv::SetCartTarget::Request::SharedPtr Robot::createCartTarget(
    std::string src, std::string target, double vel, double acc, int move_type)
{
    /*This function was returning a service object in the ros1 implementation
    Now it is returning a service request object. Take extensive care when using*/
    auto request = std::make_shared<leitungssatz_interfaces::srv::SetCartTarget::Request>();
    if (move_type == 1)
        request->mode = 1;
    else if (move_type == 0)
        request->mode = 2;

    request->speed = vel * vel_multi_;
    request->acceleration = acc * acc_multi_;
    request->asynchronous = false;

    try {
        auto tf = tfBuffer_->lookupTransform(src, target, tf2::TimePointZero);
        request->cartesian_goal = tf.transform;
    } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(node_->get_logger(), "Transform lookup failed: %s", ex.what());
    }
    return request;
}

bool Robot::call_cart_target(std::shared_ptr<leitungssatz_interfaces::srv::SetCartTarget::Request> request)
{
    if (!cart_move_->wait_for_service(std::chrono::seconds(2))) {
        //The logging here is not correct, it should use the node_ logger.
        //Commented out because it is also commented out in original code.
        //Kept for reference. 

        //RCLCPP_ERROR(this->get_logger(), "Service /cart_move not available");
        return false;
    }

    auto future = cart_move_->async_send_request(request);
    // Block until result arrives (in a node context, use node_)
    auto ret = rclcpp::spin_until_future_complete(node_, future);

    if (ret != rclcpp::FutureReturnCode::SUCCESS) {
        //Same as above, this should use node_ logger.

        //RCLCPP_ERROR(this->get_logger(), "Failed to call service /cart_move");
        return false;
    }

    auto response = future.get();
    if (!response->success) {
        //Same as above, this should use node_ logger.
        
        //RCLCPP_ERROR(this->get_logger(), "Service /cart_move reported failure to move to cart_target");
        return false;
    }
    return true;
}

bool Robot::call_joint_target(std::shared_ptr<leitungssatz_interfaces::srv::SetJointTarget::Request> request)
{
    if (!joint_move_->wait_for_service(std::chrono::seconds(2))) {
        return false;
    }

    auto future = joint_move_->async_send_request(request);

    auto ret = rclcpp::spin_until_future_complete(node_, future);

    if (ret != rclcpp::FutureReturnCode::SUCCESS) {
        return false;
    }

    auto response = future.get();
    return response->success;
}


bool Robot::call_contact_target(std::shared_ptr<leitungssatz_interfaces::srv::SetContactTarget::Request> request)
{
    if (!contact_move_->wait_for_service(std::chrono::seconds(2))) {
        return false;
    }

    auto future = contact_move_->async_send_request(request);

    auto ret = rclcpp::spin_until_future_complete(node_, future);
    if (ret != rclcpp::FutureReturnCode::SUCCESS) {
        return false;
    }

    auto response = future.get();
    return response->success;
}


bool Robot::call_speedSlider(std::shared_ptr<leitungssatz_interfaces::srv::SetSpeedSlider::Request> request)
{
    if (!_dashboard_speed->wait_for_service(std::chrono::seconds(2))) {
        return false;
    }

    auto future = _dashboard_speed->async_send_request(request);

    auto ret = rclcpp::spin_until_future_complete(node_, future);
    if (ret != rclcpp::FutureReturnCode::SUCCESS) {
        return false;
    }

    auto response = future.get();
    return response->success;
}


void Robot::add_cart_Path_Entry(std::string src, std::string target, int move_type, double blend, double vel, double acc)
{
    leitungssatz_interfaces::msg::PathEntry entry;
    //tf_buffer_ will be defined later in the tf2_publisher.cpp file.
    //entry.cartesian_goal = tf_buffer_->lookupTransform(src, target, tf2::TimePointZero).transform;
    entry.velocity = vel * vel_multi_;
    entry.acceleration = acc * acc_multi_;
    entry.blend = blend;
    entry.position_type = 0;
    entry.move_type = move_type;

    trajectory->path.push_back(entry);
}


void Robot::add_joint_Path_Entry(array6d target, int move_type, double blend, double vel, double acc)
{
    // Implement as needed for your application
}

bool Robot::call_trajectory()
{
    if (!move_trajectory_->wait_for_service(std::chrono::seconds(2))) {
        trajectory->path.clear();
        return false;
    }

    auto future = move_trajectory_->async_send_request(trajectory);

    auto ret = rclcpp::spin_until_future_complete(node_, future);
    if (ret != rclcpp::FutureReturnCode::SUCCESS) {
        trajectory->path.clear();
        return false;
    }

    auto response = future.get();
    if (!response->success) {
        trajectory->path.clear();
        return false;
    }

    trajectory->path.clear();
    return true;
}


bool Robot::setSpeed(float speed)
{
    auto request = std::make_shared<leitungssatz_interfaces::srv::SetSpeedSlider::Request>();
    request->slider = speed;
    return call_speedSlider(request);
}


bool Robot::isTransformExist(const std::string& src, std::string& target)
{
    try {
        // tf_buffer_ will be defined later in the tf2_publisher.cpp file.
        //auto transform = tf_buffer_->lookupTransform(src, target, tf2::TimePointZero);
        return true;
    }
    catch (tf2::TransformException &ex) {
        return false;
    }
}


void Robot::ptpHome()
{
    // Implement as needed
}

void Robot::demo_point()
{
    add_cart_Path_Entry("base", "home", 1, 0.01, vel_multi_, acc_multi_);
    call_trajectory();

    add_cart_Path_Entry("base", "buchse_grip", 1, 0.01, vel_multi_, acc_multi_);
    call_trajectory();

    std::string test;
    std::cout << "continue? " << std::endl;
    std::cin >> test;

    auto srv = createCartTarget("base", "buchse_grip", 1, 0.5, 0.5);
    call_cart_target(srv);
}

bool Robot::set_payload(double mass, const geometry_msgs::msg::Vector3& cog) {
    auto request = std::make_shared<leitungssatz_interfaces::srv::SetPayload::Request>();
    request->mass = mass;
    request->cog = {cog.x, cog.y, cog.z};

    if (!_set_payload->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call SetPayload service: service unavailable");
        return false;
    }

    auto future = _set_payload->async_send_request(request);
    auto ret = rclcpp::spin_until_future_complete(node_, future);

    if (ret != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call SetPayload service");
        return false;
    }

    auto response = future.get();
    if (!response->success) {
        RCLCPP_ERROR(node_->get_logger(), "SetPayload service call succeeded but returned failure");
        return false;
    }

    RCLCPP_INFO(node_->get_logger(), "Setting payload with mass: %f, center_of_gravity: [%f, %f, %f]", mass, cog.x, cog.y, cog.z);
    return true;
}



bool Robot::setClipGun(bool on) {
    auto request = std::make_shared<leitungssatz_interfaces::srv::SetClipGun::Request>();
    request->io = on;

    if (!set_clip_gun_->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call SetClipGun service: service unavailable");
        return false;
    }

    auto future = set_clip_gun_->async_send_request(request);
    auto ret = rclcpp::spin_until_future_complete(node_, future);

    if (ret != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call SetClipGun service");
        return false;
    }

    auto response = future.get();
    if (!response->success) {
        RCLCPP_ERROR(node_->get_logger(), "SetClipGun service call succeeded but returned failure");
        return false;
    }

    RCLCPP_INFO(node_->get_logger(), "SetClipGun service called: IO set to %s", on ? "ON" : "OFF");
    return true;
}



bool Robot::call_rel_cart_target(std::shared_ptr<leitungssatz_interfaces::srv::SetRelCartTarget::Request> request)
{
    if (!rel_cart_move_->wait_for_service(std::chrono::seconds(2))) {
        return false;
    }

    auto future = rel_cart_move_->async_send_request(request);
    auto ret = rclcpp::spin_until_future_complete(node_, future);

    if (ret != rclcpp::FutureReturnCode::SUCCESS) {
        return false;
    }

    auto response = future.get();
    return response->success;
}

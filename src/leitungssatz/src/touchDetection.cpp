#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <fstream>
#include <leitungssatz/eigen_json.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <boost/array.hpp>

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Plane_3.h>
#include <CGAL/Point_3.h>
#include <CGAL/Line_3.h>
#include <CGAL/intersections.h>
#include <CGAL/Aff_transformation_3.h>

#include "leitungssatz/srv/add_tf2.hpp"
#include "leitungssatz/srv/get_board_location.hpp"
#include <leitungssatz_interfaces/srv/set_freedrive.hpp>
#include <leitungssatz_interfaces/srv/set_cart_target.hpp>
#include <leitungssatz_interfaces/srv/set_gripper.hpp>

using json = nlohmann::json;
typedef CGAL::Exact_predicates_exact_constructions_kernel K;

geometry_msgs::msg::TransformStamped tcp_pose;

void tcpCallback(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
{
    tcp_pose = *msg;
}

K::Aff_transformation_3 eigen2CGAL(Eigen::Matrix4d pose)
{
    return K::Aff_transformation_3(pose.coeff(0, 0), pose.coeff(0, 1), pose.coeff(0, 2), pose.coeff(0, 3),
                                   pose.coeff(1, 0), pose.coeff(1, 1), pose.coeff(1, 2), pose.coeff(1, 3),
                                   pose.coeff(2, 0), pose.coeff(2, 1), pose.coeff(2, 2), pose.coeff(2, 3));
}

void calc_trans(const std::string &name, const geometry_msgs::msg::Transform &trans)
{
    Eigen::Isometry3d iso = tf2::transformToEigen(trans);

    Eigen::Quaterniond quat(iso.matrix().topLeftCorner<3, 3>());
    //auto euler = quat.toRotationMatrix().eulerAngles(0, 1, 2);

    RCLCPP_INFO(rclcpp::get_logger("calc_trans"), "%s: pos: %f, %f, %f", name.c_str(),
                trans.translation.x, trans.translation.y, trans.translation.z);
    RCLCPP_INFO(rclcpp::get_logger("calc_trans"), "quat: %f, %f, %f, %f",
                trans.rotation.x, trans.rotation.y, trans.rotation.z, trans.rotation.w);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("touch_localizer_node");

    // Create service clients
    auto client_freedrive = node->create_client<leitungssatz_interfaces::srv::SetFreedrive>("/ur_hardware_interface/set_freedive");
    auto client_move = node->create_client<leitungssatz_interfaces::srv::SetCartTarget>("/ur_hardware_interface/set_cart_target");
    auto client_store_tf = node->create_client<leitungssatz::srv::AddTf2>("/store_tf");
    auto gripper_move = node->create_client<leitungssatz_interfaces::srv::SetGripper>("/ur_hardware_interface/robotiq/set_gripper");
    auto taskboard_detection = node->create_client<leitungssatz::srv::GetBoardLocation>("/board_detection");

    // Create subscriber
    auto tcp_subscriber = node->create_subscription<geometry_msgs::msg::TransformStamped>(
        "/ur_hardware_interface/tcp_pose", 10, tcpCallback);

    RCLCPP_INFO(node->get_logger(), "Touch Detection Node Initialized");

    // Wait for TCP pose
    while (tcp_pose.header.stamp.sec == 0)
    {
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    // Align TCP to Z-axis
    auto target_srv = std::make_shared<leitungssatz_interfaces::srv::SetCartTarget::Request>();
    target_srv->mode = 1;
    target_srv->speed = 0.5;
    target_srv->acceleration = 0.1;
    target_srv->asynchronous = false;
    target_srv->cartesian_goal.translation = tcp_pose.transform.translation;
    target_srv->cartesian_goal.rotation = tf2::toMsg(Eigen::Quaterniond(0, 1, 0, 0));

    client_move->async_send_request(target_srv);

    // Open Gripper
    auto gripper_srv = std::make_shared<leitungssatz_interfaces::srv::SetGripper::Request>();
    gripper_srv->position = 100;
    gripper_srv->force = 1;
    gripper_srv->speed = 100;
    gripper_srv->asynchronous = true;

    gripper_move->async_send_request(gripper_srv);

    // Turn on freedrive
    auto freedrive_srv = std::make_shared<leitungssatz_interfaces::srv::SetFreedrive::Request>();
    freedrive_srv->io = true;
    freedrive_srv->free_axes = {1, 1, 1, 0, 0, 1};
    client_freedrive->async_send_request(freedrive_srv);

    RCLCPP_INFO(node->get_logger(), "Move the robot to the long edge and press enter");
    std::cin.ignore();

    // Get long edge plane
    Eigen::Isometry3d robotPose = tf2::transformToEigen(tcp_pose.transform);
    Eigen::Matrix4d longPose = robotPose.matrix();
    K::Plane_3 xzPlane(K::Point_3(0, -0.01175, 1), K::Point_3(1, -0.01175, 0), K::Point_3(-1, -0.01175, 0));
    K::Plane_3 xyPlane(K::Point_3(0, 0, 0), K::Point_3(1, 0, 0), K::Point_3(0, 1, 0));
    auto longSidePlane = xzPlane.transform(eigen2CGAL(longPose));

    // ---------------------------------------------------------
    // Get short edge plane
    Eigen::Matrix4d sidePose = robotPose.matrix();
    auto shortSidePlane = xzPlane.transform(eigen2CGAL(sidePose)); // Transform plane to TCP pose

    RCLCPP_INFO(node->get_logger(), "Move the robot to the top of the board and press enter");
    std::cin.ignore(); // Wait for enter press

    // Get top plane
    Eigen::Matrix4d topPose = robotPose.matrix();
    auto topPlane = xyPlane.transform(eigen2CGAL(topPose)); // Transform plane to TCP pose

    // Calculate intersection of planes
    auto firstIntersect = CGAL::intersection(longSidePlane, shortSidePlane); // Intersection of sides (returns a line in 3D space)
    auto finalIntersect = CGAL::intersection(*boost::get<K::Line_3>(&*firstIntersect), topPlane); // Intersection of resulting line and top plane
    K::Point_3 *resPoint = boost::get<K::Point_3>(&*finalIntersect); // Extract point from boosted result
    RCLCPP_INFO(node->get_logger(), "Intersection Point: %f, %f, %f",
                CGAL::to_double(resPoint->x()), CGAL::to_double(resPoint->y()), CGAL::to_double(resPoint->z()));

    // Take orientation of long edge and use the calculated position
    Eigen::Matrix4d resultPose = longPose;
    resultPose(0, 3) = CGAL::to_double(resPoint->x());
    resultPose(1, 3) = CGAL::to_double(resPoint->y());
    resultPose(2, 3) = CGAL::to_double(resPoint->z());

    // Rotate result around X because Z of TCP points down
    resultPose.block<3, 3>(0, 0) = resultPose.block<3, 3>(0, 0) * Eigen::Quaterniond(0, 1, 0, 0).matrix();

    RCLCPP_INFO(node->get_logger(), "Taskboard Pose: \n%f, %f, %f\n%f, %f, %f, %f",
                resultPose(0, 3), resultPose(1, 3), resultPose(2, 3),
                resultPose(0, 0), resultPose(1, 1), resultPose(2, 2), resultPose(3, 3));

    // Send TF to the server
    auto addTF_srv = std::make_shared<leitungssatz::srv::AddTf2::Request>();

    Eigen::Isometry3d iso(resultPose);

    // ✅ Convert Eigen → geometry_msgs::Transform
    addTF_srv->pose.transform = tf2::eigenToTransform(iso).transform;
    addTF_srv->pose.header.stamp = node->get_clock()->now();
    addTF_srv->pose.header.frame_id = "base";
    addTF_srv->pose.child_frame_id = "task_board";
    addTF_srv->relative = false;

    auto result = client_store_tf->async_send_request(addTF_srv);
    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(node->get_logger(), "TF sent successfully");
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to send TF");
    }

    RCLCPP_INFO(node->get_logger(), "Press enter to end the Freedrive...");
    std::cin.ignore(); // Wait for enter press

    // Turn off freedrive
    freedrive_srv->io = false;
    auto freedrive_result = client_freedrive->async_send_request(freedrive_srv);
    if (rclcpp::spin_until_future_complete(node, freedrive_result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(node->get_logger(), "Freedrive mode ended");
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to end Freedrive mode");
    }
    rclcpp::shutdown();
    return 0;
}
/**
 * @file touchDetection_service.cpp
 * @authors Adrian Müller (adrian.mueller@study.thws.de), 
 *          Maximilian Hornauer (maximilian.hornauer@study.thws.de),
 *          Usama Ali (usama.ali@study.thws.de)
 * @brief Program to manually localize the taskboard
 * @version 0.1
 * @date 2023-03-29
 * 
 * @copyright Copyright (c) 2023
 * 
 */

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

#include <leitungssatz/srv/add_tf2.hpp>
#include <leitungssatz/srv/get_board_location.hpp>
#include <leitungssatz_interfaces/srv/set_freedrive.hpp>
#include <leitungssatz_interfaces/srv/set_cart_target.hpp>
#include <leitungssatz_interfaces/srv/set_gripper.hpp>

using json = nlohmann::json;
typedef CGAL::Exact_predicates_exact_constructions_kernel K;

geometry_msgs::msg::TransformStamped tcp_pose;

class TouchDetectionService : public rclcpp::Node
{
public:
    TouchDetectionService() : Node("touch_localizer_node")
    {
        // Service
        service_ = this->create_service<leitungssatz::srv::GetBoardLocation>(
            "/touch_detection",
            std::bind(&TouchDetectionService::service_cb, this, std::placeholders::_1, std::placeholders::_2));

        // Subscriber
        tcp_subscriber_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
            "/ur_hardware_interface/tcp_pose", 10,
            std::bind(&TouchDetectionService::tcpCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Touch Detection Service Node Initialized");
    }

private:
    rclcpp::Service<leitungssatz::srv::GetBoardLocation>::SharedPtr service_;
    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr tcp_subscriber_;

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
        auto euler = quat.toRotationMatrix().eulerAngles(0, 1, 2);

        RCLCPP_INFO(this->get_logger(), "%s: pos: %f, %f, %f", name.c_str(),
                    trans.translation.x, trans.translation.y, trans.translation.z);
        RCLCPP_INFO(this->get_logger(), "quat: %f, %f, %f, %f",
                    trans.rotation.x, trans.rotation.y, trans.rotation.z, trans.rotation.w);
    }

    bool service_cb(const std::shared_ptr<leitungssatz::srv::GetBoardLocation::Request> req,
                    std::shared_ptr<leitungssatz::srv::GetBoardLocation::Response> res)
    {
        auto client_f = this->create_client<leitungssatz_interfaces::srv::SetFreedrive>("/ur_hardware_interface/set_freedive");
        auto client_move = this->create_client<leitungssatz_interfaces::srv::SetCartTarget>("/ur_hardware_interface/set_cart_target");
        auto client_store_tf = this->create_client<leitungssatz::srv::AddTf2>("/store_tf");
        auto gripper_move = this->create_client<leitungssatz_interfaces::srv::SetGripper>("/ur_hardware_interface/robotiq/set_gripper");

        leitungssatz_interfaces::srv::SetGripper::Request gripper_srv;
        gripper_srv.position = 100;
        gripper_srv.force = 1;
        gripper_srv.speed = 100;
        gripper_srv.asynchronous = true;

        // Wait for TCP pose
        while (tcp_pose.header.stamp.sec == 0)
        {
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }

        // Align TCP to Z-axis
        leitungssatz_interfaces::srv::SetCartTarget::Request target_srv;
        target_srv.mode = 1;
        target_srv.speed = 0.5;
        target_srv.acceleration = 0.1;
        target_srv.asynchronous = false;
        target_srv.cartesian_goal.translation = tcp_pose.transform.translation;
        target_srv.cartesian_goal.rotation = tf2::toMsg(Eigen::Quaterniond(0, 1, 0, 0));

        client_move->async_send_request(std::make_shared<leitungssatz_interfaces::srv::SetCartTarget::Request>(target_srv));

        // Open Gripper
        gripper_move->async_send_request(std::make_shared<leitungssatz_interfaces::srv::SetGripper::Request>(gripper_srv));

        // Turn on freedrive
        leitungssatz_interfaces::srv::SetFreedrive::Request freedrive_srv;
        freedrive_srv.io = true;
        freedrive_srv.free_axes = {1, 1, 1, 0, 0, 1};
        client_f->async_send_request(std::make_shared<leitungssatz_interfaces::srv::SetFreedrive::Request>(freedrive_srv));

        RCLCPP_INFO(this->get_logger(), "Move the robot to the long edge and press enter");
        std::cin.ignore();

        // Get long edge plane
        Eigen::Isometry3d robotPose = tf2::transformToEigen(tcp_pose.transform);
        Eigen::Matrix4d longPose = robotPose.matrix();
        K::Plane_3 xzPlane(K::Point_3(0, -0.01175, 1), K::Point_3(1, -0.01175, 0), K::Point_3(-1, -0.01175, 0));
        auto longSidePlane = xzPlane.transform(eigen2CGAL(longPose));

        // Continue with similar logic for short edge and top plane...

        res->success = true;
        res->config = 0;
        return true;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TouchDetectionService>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
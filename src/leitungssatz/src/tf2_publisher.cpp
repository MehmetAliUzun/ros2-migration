#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <fstream>
#include <leitungssatz/eigen_json.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include "leitungssatz/srv/add_tf2.hpp"

#include <visualization_msgs/msg/marker.hpp>

using json = nlohmann::json;

class tf2_publisher : public rclcpp::Node
{
private:
    rclcpp::Service<leitungssatz::srv::AddTf2>::SharedPtr server_tf_;

    std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
    std::shared_ptr<tf2_ros::TransformListener> tfListener_;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vis_pub_;
    std::string filename;
    std::map<std::string, geometry_msgs::msg::TransformStamped> transforms;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> br;
    
    void add_tf(const std::shared_ptr<leitungssatz::srv::AddTf2::Request> req,
                std::shared_ptr<leitungssatz::srv::AddTf2::Response> res);
    void writeJson();
    void readJson();
    void publishTf();
    void visual_taskboard();

public:
    tf2_publisher(const std::string& filename_);
    ~tf2_publisher();
};

tf2_publisher::tf2_publisher(const std::string& filename_)
    : Node("tf2_publisher")
{
    this->tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    this->tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

    this->server_tf_ = this->create_service<leitungssatz::srv::AddTf2>(
        "/store_tf",
        std::bind(&tf2_publisher::add_tf, this, std::placeholders::_1, std::placeholders::_2));
    
    this->vis_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker", 1);
    
    this->br = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    this->filename = filename_;

    this->readJson();
    this->publishTf();
}

tf2_publisher::~tf2_publisher()
{
    this->writeJson();
}

void tf2_publisher::visual_taskboard()
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "buchse";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "leitungssatz";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.001;
    marker.scale.y = 0.001;
    marker.scale.z = 0.001;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.mesh_resource = "package://leitungssatz/config/buchse.dae";
    marker.mesh_use_embedded_materials = true;
    vis_pub_->publish(marker);
}

void tf2_publisher::publishTf() 
{
    for(auto i : this->transforms) {
        this->br->sendTransform(i.second);
    }
    this->visual_taskboard();
}

void tf2_publisher::readJson() 
{
    std::string path = ament_index_cpp::get_package_share_directory("leitungssatz");

    std::ifstream f(path + "/config/" + this->filename);/*InputFileStream*/
    if (!f.is_open()) {
        RCLCPP_WARN(this->get_logger(), "Could not open file: %s", (path + "/config/" + this->filename).c_str());
        return;
    }
    
    json j = json::parse(f);

    for(auto element : j.items())
    {   
        RCLCPP_INFO(this->get_logger(), "Importing %s", element.key().c_str());
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = this->get_clock()->now();
        transformStamped.header.frame_id = element.value()["parent"];
        transformStamped.child_frame_id = element.value()["name"];
        
        // Eigen::Matrix4d mat = element.value()["transform"];
        
        auto transform_json = element.value()["transform"];

        Eigen::Matrix4d mat;
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                mat(i, j) = transform_json[i][j];
            }
        }


        Eigen::Isometry3d iso(mat);
        transformStamped.transform = tf2::eigenToTransform(iso).transform;
        
        transforms[element.value()["name"]] = transformStamped;
    }
}

void tf2_publisher::writeJson()
{   
    RCLCPP_INFO(this->get_logger(), "Writing file...");
    json j;

    for(auto i : this->transforms) {
        geometry_msgs::msg::TransformStamped transf = i.second;
        Eigen::Isometry3d iso = tf2::transformToEigen(transf.transform);
        j[i.first]["parent"] = transf.header.frame_id;
        j[i.first]["name"] = transf.child_frame_id;
        j[i.first]["transform"] = iso.matrix();
    }

    std::string path = ament_index_cpp::get_package_share_directory("leitungssatz");
    std::ofstream file(path + "/config/" + this->filename);
    file << std::setw(4) << j;
}

/**
 * @brief Service call to add a tf to the transforms buffer
 * 
 * @param req the request with the transform and the information if the pose should be recalculated in relation to the taskboard
 * @param res the response
 */
void tf2_publisher::add_tf(const std::shared_ptr<leitungssatz::srv::AddTf2::Request> req,
                          std::shared_ptr<leitungssatz::srv::AddTf2::Response> res)
{
    RCLCPP_INFO_STREAM(this->get_logger(), "Callback called with " << req->pose.child_frame_id);
    geometry_msgs::msg::TransformStamped temp = req->pose;
    
    /* The pose needs to be retransformed because it is given in relation to the base_link but we want it in relation to the taskboard*/
    if(req->relative) {
        Eigen::Isometry3d iso = tf2::transformToEigen(temp.transform);
        Eigen::Matrix4d mat_f = iso.matrix();

        try {
            auto tb = tfBuffer_->lookupTransform("base", temp.header.frame_id, tf2::TimePointZero);
            Eigen::Isometry3d iso_tb = tf2::transformToEigen(tb.transform);
            Eigen::Matrix4d mat_tb = iso_tb.matrix();

            Eigen::Matrix4d mat_rel = mat_tb.inverse() * mat_f; //calculate target in reference to task_board
            temp.transform = tf2::eigenToTransform(Eigen::Isometry3d(mat_rel)).transform;
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "Transform lookup failed: %s", ex.what());
            res->success = false;
            return;
        }
    }
    else{
        temp = req->pose; // we dont want to change anything in the transform
    }

    transforms[temp.child_frame_id] = temp; // write the transform into buffer
    this->publishTf(); // republish everything
    res->success = true;
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<rclcpp::Node>("tf2_publisher_main");
    
    std::string filename;
    node->declare_parameter("json_filename", "");
    
    if (node->get_parameter("json_filename", filename) && !filename.empty())
    {
        RCLCPP_INFO(node->get_logger(), "filename: %s", filename.c_str());
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "FAILED to get json_filename parameter");
        return 1;
    }
    
    auto tf2_pub = std::make_shared<tf2_publisher>(filename);
    
    rclcpp::spin(tf2_pub);
    rclcpp::shutdown();
    
    return 0;
}
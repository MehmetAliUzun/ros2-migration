#pragma once

#include "leitungssatz/Task.hpp"
#include "leitungssatz/Tasks/GripperTask.hpp"
#include "leitungssatz/Tasks/MoveTask.hpp"

#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>

namespace leitungssatz {

class CableCheckTask : public Task {
public:
    CableCheckTask(Robot_ptr robot, std::shared_ptr<rclcpp::Node> node, std::string cameratopic, CableType cableType, double vel, double acc);

    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
    void load_image(std::string fileName);

    bool check_black_cable();
    bool check_first_blue_cable();
    bool check_second_blue_cable();
    bool fallback_routine();
    TaskStatus execute() override;

private:

    //black cable
    const int roi_black_x_edge = 210;
    const int roi_black_y_edge = 230;
    const int roi_black_height = 70; 
    const int roi_black_width = 110;

    const int threshold_canny_black_low = 80;
    const int threshold_canny_black_high = 240;
    const int threshold_hough_black = 40;

    //first blue cable
    const int roi_first_blue_x_edge = 290;
    const int roi_first_blue_y_edge = 215;
    const int roi_first_blue_height = 45; 
    const int roi_first_blue_width = 75; 

    const int threshold_canny_first_blue_low = 80;
    const int threshold_canny_first_blue_high = 240;
    const int threshold_hough_first_blue = 20;

    //second blue cable
    const int roi_second_blue_left_x_edge = 290;
    const int roi_second_blue_left_y_edge = 195;
    const int roi_second_blue_left_height = 20; 
    const int roi_second_blue_left_width = 25; 

    const int roi_second_blue_right_x_edge = 332;
    const int roi_second_blue_right_y_edge = 195;
    const int roi_second_blue_right_height = 20; 
    const int roi_second_blue_right_width = 23;

    const int threshold_canny_second_blue_low = 80;
    const int threshold_canny_second_blue_high = 240;
    const int threshold_hough_second_blue = 5;

    image_transport::Subscriber sub_;
    image_transport::ImageTransport it_;
    cv::Mat img;
    CableType cableType;
    double vel, acc;
    std::shared_ptr<rclcpp::Node> node_;
};

}
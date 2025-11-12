#include <leitungssatz/Utils/SimpleImagePublisher.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("imgPub");
    auto pub = node->create_publisher<sensor_msgs::msg::Image>("camSim/image", 1);

    if(argc != 2){
        RCLCPP_ERROR(node->get_logger(), "No Image provided");
        return 1;
    }
    cv::Mat image = cv::imread(argv[1]);
    cv::waitKey(30);
    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();

    rclcpp::Rate loop_rate(1);
    RCLCPP_INFO(node->get_logger(), "Publishing..");
    while (rclcpp::ok()) {
        pub->publish(*msg);
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
} 
//Checked for consistency with the original ROS1 code
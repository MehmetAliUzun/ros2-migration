#include <leitungssatz/Tasks/CableCheckTask.hpp>

using namespace leitungssatz;

CableCheckTask::CableCheckTask(Robot_ptr robot, std::shared_ptr<rclcpp::Node> node, std::string cameratopic, CableType cableType, double vel, double acc)
    : Task(robot), it_(node), vel(vel), acc(acc), node_(node)
{
    this->cableType = cableType;
    this->sub_ = this->it_.subscribe(
        cameratopic, 1,
        std::bind(&CableCheckTask::imageCallback, this, std::placeholders::_1));
}

void CableCheckTask::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
    try
    {
        img = cv_bridge::toCvShare(msg, "bgr8")->image;
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(node_->get_logger(), "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void CableCheckTask::load_image(std::string fileName)
{
    img = cv::imread(fileName);
    return;
}

bool CableCheckTask::check_black_cable()
{
    cv::Rect rect_black(roi_black_x_edge,roi_black_y_edge,roi_black_width,roi_black_height);
    cv::Mat img_cropped = img(rect_black);

    cv::Mat img_gray;
    cv::cvtColor(img_cropped,img_gray,cv::COLOR_BGR2GRAY);
    
    cv::Mat black_edges;
    cv::Canny(img_gray,black_edges,threshold_canny_black_low,threshold_canny_black_high);

    std::vector<cv::Vec2f> lines_black;
    cv::HoughLines(black_edges,lines_black,1,CV_PI/180,threshold_hough_black);

    double rho = std::round((lines_black[0][0] + lines_black[1][0])/2.0);
    double theta = std::round((lines_black[0][1] + lines_black[1][1])/2.0);

    cv::Vec3f l_black(std::cos(theta),std::sin(theta),-rho);
    double x_s = l_black[2] / (-l_black[0]);

    if(x_s >= 40 && x_s <=65)
        return true;
    else
        return false;
}

bool CableCheckTask::check_first_blue_cable()
{
    cv::Rect rect_first_blue(roi_first_blue_x_edge,roi_first_blue_y_edge,roi_first_blue_width,roi_first_blue_height);
    cv::Mat img_cropped = img(rect_first_blue);

    cv::Mat img_gray;
    cv::cvtColor(img_cropped,img_gray,cv::COLOR_BGR2GRAY);

    cv::Mat first_blue_edges;
    cv::Canny(img_gray,first_blue_edges,threshold_canny_first_blue_low,threshold_canny_first_blue_high);

    std::vector<cv::Vec2f> lines_first_blue;
    cv::HoughLines(first_blue_edges,lines_first_blue,1,CV_PI/180,threshold_hough_first_blue);

    double rho = std::round((lines_first_blue[0][0] + lines_first_blue[1][0])/2.0);
    double theta = std::round((lines_first_blue[0][1] + lines_first_blue[1][1])/2.0);

    cv::Vec3f l_first_blue(std::cos(theta),std::sin(theta),-rho);
    double x_s = l_first_blue[2] / (-l_first_blue[0]);
    
    if(x_s >= 30 && x_s <= 40)
        return true;
    else
        return false;
}


bool CableCheckTask::check_second_blue_cable()
{
    cv::Rect rect_left(roi_second_blue_left_x_edge,roi_second_blue_left_y_edge,roi_second_blue_left_width,roi_second_blue_left_height);
    cv::Rect rect_right(roi_second_blue_right_x_edge,roi_second_blue_right_y_edge,roi_second_blue_right_width,roi_second_blue_right_height);

    cv::Mat img_cropped_left = img(rect_left);
    cv::Mat img_cropped_right = img(rect_right);

    cv::Mat img_gray_left,img_gray_right;
    cv::cvtColor(img_cropped_left,img_gray_left,cv::COLOR_BGR2GRAY);
    cv::cvtColor(img_cropped_right,img_gray_right,cv::COLOR_BGR2GRAY);

    cv::Mat edges_right,edges_left;
    cv::Canny(img_gray_left,edges_left,threshold_canny_second_blue_low,threshold_canny_second_blue_high);
    cv::Canny(img_gray_right,edges_right,threshold_canny_second_blue_low,threshold_canny_second_blue_high);
    
    cv::Scalar sum_left = cv::sum(edges_left);
    cv::Scalar sum_right = cv::sum(edges_right);

    if(sum_left[0] < 255*5 && sum_right[0] < 255*5)
        return true;
    else    return false;

    //other solution
    /*
    std::vector<cv::Vec2f>lines_left,lines_right;
    cv::HoughLines(edges_left,lines_left,1,CV_PI/180,threshold_hough_second_blue);
    cv::HoughLines(edges_right,lines_right,1,CV_PI/180,threshold_hough_second_blue);

    if(lines_left.empty() && lines_right.empty())
        return true;
    else
        return false;
    */
}

bool CableCheckTask::fallback_routine()
{
    bool ret = true;
    //grip cable and drive to fallback
    GripperTask grip_task_1 = GripperTask(_robot, 45,100,10, true);
    GripperTask grip_task_close = GripperTask(_robot, 0,100,10, true);
    GripperTask grip_task_open = GripperTask(_robot, 100,100,10, true);

    Task_ptr move_cable_1;
    Task_ptr move_cable_2;

    if(cableType == CableType::BLACK){
        move_cable_1 = Task_ptr (new MoveTask(_robot, "black_push_close", leitungssatz::MOVEL, vel, acc));
        move_cable_2 = Task_ptr (new MoveTask(_robot, "black_push_far", leitungssatz::MOVEL, vel, acc));
    } else if(cableType == CableType::LOWER_BLUE){
        move_cable_1 = Task_ptr (new MoveTask(_robot, "blue_down_push_close", leitungssatz::MOVEL, vel, acc));
        move_cable_2 = Task_ptr (new MoveTask(_robot, "blue_down_push_far", leitungssatz::MOVEL, vel, acc));
    } else if(cableType == CableType::UPPER_BLUE){
        move_cable_1 = Task_ptr (new MoveTask(_robot, "blue_up_push_close", leitungssatz::MOVEL, vel, acc));
        move_cable_2 = Task_ptr (new MoveTask(_robot, "blue_up_push_far", leitungssatz::MOVEL, vel, acc));
    }


    MoveTask fallback_pos_1 = MoveTask(_robot, "fallback1", leitungssatz::MOVEJ, vel, acc);
    MoveTask fallback_pos_2 = MoveTask(_robot, "fallback2", leitungssatz::MOVEJ, vel, acc);

    ret = ret && grip_task_1.execute();
    ret = ret && move_cable_1->execute();
    ret = ret && grip_task_close.execute();
    ret = ret && move_cable_2->execute();
    ret = ret && fallback_pos_1.execute();
    ret = ret && fallback_pos_2.execute();
    ret = ret && grip_task_open.execute();
    ret = ret && fallback_pos_1.execute();

    return ret;
}
TaskStatus CableCheckTask::execute()
{
    _task_status = TaskStatus::RUNNING;
    bool ret ; //= false;
    if (cableType == CableType::BLACK)
        ret = check_black_cable();
    else if (cableType == CableType::LOWER_BLUE)
        ret = check_first_blue_cable();
    else if (cableType == CableType::UPPER_BLUE)
        ret = check_second_blue_cable();

    RCLCPP_WARN(node_->get_logger(), "Did I insert the cable correctly? %s", ret ? "true" : "false");
    if (ret)
        _task_status = TaskStatus::FINISHED;
    else
        _task_status = TaskStatus::FAILED;

    if (_task_status == TaskStatus::FAILED)
    {
        fallback_routine();
        _task_status = TaskStatus::FAILEDSAFE;
    }
    return _task_status;
}
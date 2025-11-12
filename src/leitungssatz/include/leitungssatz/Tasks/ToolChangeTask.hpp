#pragma once
#include "leitungssatz/Task.hpp"
#include "leitungssatz/Tasks/MoveTask.hpp"
#include "leitungssatz/Tasks/GripperTask.hpp"
#include <leitungssatz_interfaces/srv/set_payload.hpp>
#include <leitungssatz_interfaces/srv/set_rel_cart_target.hpp>
#include <leitungssatz_interfaces/srv/set_clip_gun.hpp>


#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <memory>
#include <vector>
#include <string>

namespace leitungssatz {

enum class ToolType {
    NONE,
    STANDARD,
    HEAVY
};

class ToolChangeTask : public Task {
private:
    rclcpp::Client<leitungssatz_interfaces::srv::SetPayload>::SharedPtr payload_service_;
    
    
    Robot_ptr rob;
    const double GRIPPER_WAIT = 0.5;  // 500ms wait

    bool configurePayload(ToolType type);

public:
    ToolChangeTask(Robot_ptr robot);
    TaskStatus execute() override;
    bool moveY(double distance, double speed, double acceleration);
    bool clip();
};

}
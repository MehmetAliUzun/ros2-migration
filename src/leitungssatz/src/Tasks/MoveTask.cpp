#include "leitungssatz/Tasks/MoveTask.hpp"

using namespace leitungssatz;

bool MoveTask::init() {
  bool isPointPublished = _robot->isTransformExist(_parent_name, _point_name);
  if (!isPointPublished) {
    throw std::runtime_error("Point " + _parent_name + " -> " + _point_name + " doesn't exist!");
  }
  return true;//Original code doesn't return anything, but the function signature suggests it should return a boolean.
}

bool MoveTask::moveJ() {
  leitungssatz_interfaces::srv::SetCartTarget::Request::SharedPtr req = _robot->createCartTarget(
    _parent_name, _point_name, _vel, _acc, 0);
  return _robot->call_cart_target(req);
}

bool MoveTask::moveL() {
  leitungssatz_interfaces::srv::SetCartTarget::Request::SharedPtr req = _robot->createCartTarget(
    _parent_name, _point_name, _vel, _acc, 1);
  return _robot->call_cart_target(req);
}

bool MoveTask::moveP() {
  leitungssatz_interfaces::srv::SetCartTarget::Request::SharedPtr req = _robot->createCartTarget(
    _parent_name, _point_name, _vel, _acc, 2);
  return _robot->call_cart_target(req);
}
//The f.definitions above are adjusted to make compatible with robot class.
//Adjust also the moveC() and take notes about the changes
//because this changes may alter the original function signature

//The ros1 version of this function was returning a service object
//Now it is returning a service request object. Take extensive care when using

bool MoveTask::moveC() {
  leitungssatz_interfaces::srv::SetCartTarget::Request::SharedPtr req = _robot->createCartTarget(
    _parent_name, _point_name, _vel, _acc, 3);
  return _robot->call_cart_target(req);
}

TaskStatus MoveTask::execute()
{
    RCLCPP_INFO(rclcpp::get_logger("move_task"), "Executing MoveTask");
    _task_status = TaskStatus::RUNNING;
    bool ret = false;
    switch(_movetype){
        case MOVEJ:
            ret = moveJ();
            break;
        case MOVEL:
            ret = moveL();
            break;
        case MOVEP:
            ret = moveP();
            break;
        case MOVEC:
            ret = moveC();
            break;
    }
    if(ret)
        _task_status = TaskStatus::FINISHED;
    else
        _task_status = TaskStatus::FAILED;
    return _task_status;
}
#include <leitungssatz/Point.hpp>

namespace leitungssatz {

//Point::Point() : _name("Default"), _transformation(Eigen::Matrix4f::Identity()) {}
//above line is commented out to avoid confusion with the new constructor definition


Point::Point(const Eigen::Matrix4f& transformation)
    : _transformation(transformation), _name("Default") {}

} // namespace leitungssatz

// This source file and its header file are manually checked for
// consistency with the ROS1 version.
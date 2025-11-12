#pragma once

#include "Main.hpp"
#include <Eigen/Core>
#include <string>
#include <memory>

namespace leitungssatz {

class Point
{
private:
    Eigen::Matrix4f _transformation;
    std::string _name;
    // std::shared_ptr<Point> _parent; // Uncomment if you want to use parent pointers

public:
    Point();
    Point(const Eigen::Matrix4f& transformation);
    /*Original code does not pass a reference instead
    it passes the matrix itself which copies the matrix
    Change this to original usage if any errors occur*/
};

} // namespace leitungssatz
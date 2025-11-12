#pragma once

#include "Main.hpp"
#include "Point.hpp"
#include <string>
#include <vector>

namespace leitungssatz {

class PointLoader
{
private:
    std::vector<Point> _points;
    std::string _JSON_path;

public:
    bool loadPoints();
    Point findPoint(const std::string& name);
    bool addPoint(const Point& point);
    bool deletePoint(const Point& point);
};

} // namespace leitungssatz
#pragma once 

#include <ros/ros.h>
#include <node_c/obstacle_extractor/figures/point.h>
#include <node_c/obstacle_extractor/figures/segment.h>

class Circle
{
public:

    Circle(const Point &point = Point(), const double radius = 0.0);
    Circle(const Segment &segment);


    std::vector<PointSet> getPointSets() const;
    double getRadius() const;
    Point getCenter() const;
    double distanceTo(const Point &point);
    void incrementRadius(double radius);
    static Circle fitCircle(const std::list<Point> &pointSet);

    friend std::ostream &operator<<(std::ostream &out, const Circle &c)
    {
        out << "Center : " << c.center << ", Radius : " << c.radius;
        return out;
    }

private:
    Point center;
    double radius;
    std::vector<PointSet> pointSets;
};

#pragma once 

#include <ros/ros.h>
#include <node_c/obstacle_extractor/figures/pointset.h>
#include <node_c/obstacle_extractor/figures/point.h>
#include <armadillo>

class Segment
{
public:
    Segment(const Point &point1 = Point(), const Point &point2 = Point());
    void setFirstPoint(Point point);
    void setLastPoint(Point point);
    Point getFirstPoint() const;
    Point getLastPoint() const;
    std::vector<PointSet> getPointSets() const;
    double length() const;
    double lengthSquared() const;
    Point normal() const;
    Point projection(const Point &point) const;
    Point trueProjection(const Point &point) const;
    double distanceTo(const Point &point) const;
    double trueDistanceTo(const Point &point) const;
    static Segment fitSegment(const PointSet &pointSet);
    static Segment fitSegment(const std::vector<PointSet> &pointSets);
    friend std::ostream &operator<<(std::ostream &out, const Segment &segment)
    {
        out << "point 1 : " << segment.firstPoint << ", point 2 : " << segment.lastPoint;
        return out;
    }

private:
    Point firstPoint; // First point of the segment
    Point lastPoint;  // Last point of the segment
    std::vector<PointSet> pointSets;
};

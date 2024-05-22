#pragma once

#include <ros/ros.h>
#include <node_c/obstacle_extractor/figures/point.h>
#include <list>

typedef std::list<Point>::iterator PointIterator;

class PointSet
{
public:
    PointSet();
    void incrementNumberOfPoints();
    PointIterator getBegin() const;
    PointIterator getEnd() const;
    int getNumberOfPoints() const;
    bool getIsVisible() const;
    void setBegin(PointIterator iterator);
    void setEnd(PointIterator iterator);
    void setNumberOfPoints(int numberOfPoints);
    void setIsVisible(bool isVisible);
private:
    PointIterator begin; //Begin iterator of the poinset
    PointIterator end; //End iterator of the poinset
    int numberOfPoints; //Number of points of the pointset
    bool isVisible;  //Visibility of the pointset
};

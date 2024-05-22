#include <node_c/obstacle_extractor/figures/pointset.h>

PointSet::PointSet() : numberOfPoints(0), isVisible(false)
{}

PointIterator PointSet::getBegin() const
{
    return begin;
} // getBegin

PointIterator PointSet::getEnd() const
{
    return end;
} // getEnd

int PointSet::getNumberOfPoints() const
{
    return numberOfPoints;
} // getNumberOfPoints

bool PointSet::getIsVisible() const
{
    return isVisible;
} // getIsVisible

void PointSet::setBegin(PointIterator iterator)
{
    begin = iterator;
} // setBegin

void PointSet::setEnd(PointIterator iterator)
{
    end = iterator;
} // setEnd

void PointSet::setNumberOfPoints(int numberOfPoints)
{
    this->numberOfPoints = numberOfPoints;
} // setNumberOfPoints

void PointSet::setIsVisible(bool isVisible)
{
    this->isVisible = isVisible;
} // setIsVisible

void PointSet::incrementNumberOfPoints()
{
    numberOfPoints++;
} // incrementNumberOfPoints
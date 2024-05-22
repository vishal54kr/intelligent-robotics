#pragma once 

#include <ros/ros.h>
#include <node_c/obstacle_extractor/figures/circle.h>
#include <node_c/obstacle_extractor/figures/segment.h>
#include <node_c/obstacle_extractor/figures/pointset.h>
#include <node_c/obstacle_extractor/figures/point.h>
#include <list>

class ObstacleExtractor
{
public:
    ObstacleExtractor(std::list<Point> inputPoints, double angleIncrement);
    ObstacleExtractor(std::list<Point> inputPoints, double angleIncrement,
                      int minGroupPoints, double maxGroupDistance,
                      double maxSplitDistance, double maxMergeSeparation,
                      double maxMergeSpread, double radiusEnlargement, double maxCircleRadius);

    ObstacleExtractor(std::list<Point> inputPoints, double angleIncrement, double maxCircleRadius, double minCircleRadius, double radiusEnlargement);

    void processPoints(std::list<Segment> &segments, std::list<Circle> &circles);

private:

    void groupPoints();
    void detectSegments(const PointSet &pointSet);
    void mergeSegments();
    bool compareSegments(const Segment &segment1, const Segment &segment2, Segment &mergedSegment);
    bool checkSegmentsProximity(const Segment &segment1, const Segment &segment2);
    bool checkSegmentsCollinearity(const Segment &segment, const Segment &segment1, const Segment &segment2);
    void detectCircles();
    void mergeCircles();
    bool compareCircles(const Circle &circle1, const Circle &circle2, Circle &mergedCircle);

    std::list<Point> inputPoints;
    std::list<Segment> segments;
    std::list<Circle> circles;
    double distanceProportion;
    double maxCircleRadius = 0.35;
    double minCircleRadius = 0.05;
    double radiusEnlargement = 0.05;
};

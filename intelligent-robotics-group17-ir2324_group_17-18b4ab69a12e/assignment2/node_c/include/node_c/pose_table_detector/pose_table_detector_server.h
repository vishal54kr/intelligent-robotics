#pragma once

#include <ros/ros.h>
#include <node_c_msgs/PoseTableDetector.h>
#include <node_b_msgs/HeadMovement.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

class PoseTableDetectorServer
{
private:
    ros::NodeHandle node;
    ros::ServiceServer server;
    ros::ServiceClient headClient;
    static bool compareCircles(node_c_msgs::Circle c1, node_c_msgs::Circle c2);
    bool isSmallest(int x, int y, int z);
    bool isBiggest(int x, int y, int z);
    cv::Point2i getAvgPoint(const std::vector<cv::Point2i> &points);
    std::vector<cv::Point2i> getPoints(const cv::Mat &image);

public:
    PoseTableDetectorServer(ros::NodeHandle node);
    bool sendResponse(node_c_msgs::PoseTableDetector::Request &req, node_c_msgs::PoseTableDetector::Response &res);
};

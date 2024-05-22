#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <node_c_msgs/Circle.h>
#include <node_c_msgs/Circles.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/message_filter.h>
#include <node_c/obstacle_extractor/obstacle_extractor.h>

class ObstacleExtractorServer
{
public:
    ObstacleExtractorServer(ros::NodeHandle node);

private:
    std::list<Point> computePoints(sensor_msgs::LaserScan msg);
    bool sendResponse(node_c_msgs::Circles::Request &req, node_c_msgs::Circles::Response &res);

    ros::NodeHandle node;
    ros::ServiceServer server;
};

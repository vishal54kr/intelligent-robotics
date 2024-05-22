#pragma once

#include <ros/ros.h>
#include <node_c_msgs/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/message_filter.h>


class PosesTransformerServer
{
public:
    PosesTransformerServer(ros::NodeHandle node);

private:
    bool sendResponse(node_c_msgs::Transform::Request &req, node_c_msgs::Transform::Response &res);

    ros::NodeHandle node;
    ros::ServiceServer server;
};


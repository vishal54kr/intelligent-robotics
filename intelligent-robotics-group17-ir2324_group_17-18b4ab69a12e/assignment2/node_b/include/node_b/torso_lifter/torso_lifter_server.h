#pragma once

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <node_b_msgs/TorsoLifter.h>

class TorsoLifterServer
{
public:
    TorsoLifterServer(ros::NodeHandle node);

private:
    // Attributes
    ros::NodeHandle node;
    ros::ServiceServer server;
    moveit::planning_interface::MoveGroupInterface moveGroupInterfaceArm;

    bool sendResponse(node_b_msgs::TorsoLifter::Request &req, node_b_msgs::TorsoLifter::Response &res);
};

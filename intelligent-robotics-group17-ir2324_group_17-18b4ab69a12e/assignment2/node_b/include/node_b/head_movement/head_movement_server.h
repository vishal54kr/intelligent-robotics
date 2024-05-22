#pragma once

#include <ros/ros.h>
#include <ros/topic.h>
#include <actionlib/client/simple_action_client.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PointStamped.h>
#include <node_b_msgs/HeadMovement.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/PointHeadAction.h>


typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> HeadControllClient;
typedef actionlib::SimpleActionClient<control_msgs::PointHeadAction> PointHeadClient;


class HeadMovementServer
{
public:
    // constructor
    HeadMovementServer();
    bool headMovementCallback(node_b_msgs::HeadMovement::Request &req, node_b_msgs::HeadMovement::Response &res);
    void pointHead(cv::Point2d center);
    void pointHeadForNavigation();

private:
    PointHeadClient pointHeadClient;              // action client for moving the Tiago Head
    HeadControllClient headClient;                // action client for moving the Tiago Head
    control_msgs::FollowJointTrajectoryGoal goal; // goal to move the head
    cv::Mat cameraIntrinsics;                     // camera intrinsics parameters
    ros::NodeHandle node;
    ros::NodeHandle serverNode;
    ros::ServiceServer server;
};

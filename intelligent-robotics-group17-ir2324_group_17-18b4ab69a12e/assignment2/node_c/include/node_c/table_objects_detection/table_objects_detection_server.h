#pragma once

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect.hpp>
#include <cv_bridge/cv_bridge.h>
#include <node_b_msgs/HeadMovement.h>
#include <node_c_msgs/Detections.h>
#include <node_c_msgs/Detection.h>
#include <sensor_msgs/JointState.h>
#include <node_c_msgs/ObjectsDetectionAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/terminal_state.h>

class TableObjectsDetectionServer
{
public:
    TableObjectsDetectionServer();
    void startDetection(const node_c_msgs::ObjectsDetectionGoalConstPtr &goal);

private:
    int colorID; // id of the color to search (it arrives from the action request)
    ros::NodeHandle nodeHead;
    actionlib::SimpleActionServer<node_c_msgs::ObjectsDetectionAction> actionServer; // action server for the ObjectsDetection action

    ros::ServiceClient clientHead = nodeHead.serviceClient<node_b_msgs::HeadMovement>("/head_movement"); // client to the head movement manipulation service

    node_c_msgs::ObjectsDetectionFeedback feedback; // feedback to the action, see the ObjectsDetection action message
    node_c_msgs::ObjectsDetectionResult result;     // results to the action, see the ObjectsDetection action message

    double joint1, joint2;     // variables with the head joints variables
    std::set<int> observedIds; // set with the april tag ids already detected

    void lookToPoint(cv::Point2d center);
    void pointTable();
    void pointColor();
    void pointArea(cv::Rect colorizedArea);
    void moveHeadDown();
    void pointObstacleTags();
    void resetPosition();

};

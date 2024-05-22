#pragma once

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/terminal_state.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <navigation/NavigateAction.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/LaserScan.h>
#include <list>

// Action specification for move_base
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


class NavigationServer
{
private:
    ros::NodeHandle nh;                                                               // Node for managing the action server
    actionlib::SimpleActionServer<navigation::NavigateAction> server; // The action server
    MoveBaseClient client;                                                            // Client to the Navigation Stack
    navigation::NavigateFeedback feedback;                            // Feedback message
    navigation::NavigateResult result;                                // Result message
    navigation::NavigateGoal goal;                                    // Goal message

    void doNavigation(const navigation::NavigateGoalConstPtr &goal);

public:
    NavigationServer(std::string name);

    void navAndDetectCallback(const navigation::NavigateGoalConstPtr &goal);
};

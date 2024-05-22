#pragma once

#include <ros/ros.h>
#include <gazebo_ros_link_attacher/Attach.h>
#include <gazebo_ros_link_attacher/AttachRequest.h>
#include <gazebo_ros_link_attacher/AttachResponse.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/terminal_state.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <node_b_msgs/PickPlaceAction.h>

class PickPlaceServer
{
public:
    PickPlaceServer(ros::NodeHandle node);
    void callback(const node_b_msgs::PickPlaceGoalConstPtr &goal);

private:
    // Attributes
    moveit::planning_interface::PlanningSceneInterface planningSceneInterface;
    moveit::planning_interface::MoveGroupInterface moveGroupInterfaceArm;
    moveit::planning_interface::MoveGroupInterface moveGroupInterfaceGripper;
    std::map<int, geometry_msgs::Pose> detectionsPosesIds;
    std::vector<moveit_msgs::CollisionObject> collisionObjects;

    actionlib::SimpleActionServer<node_b_msgs::PickPlaceAction> server;
    node_b_msgs::PickPlaceFeedback feedback;
    node_b_msgs::PickPlaceResult result;
    node_b_msgs::PickPlaceGoal goal;

    // Private method
    bool moveToDefaultPose();
    bool moveToPosePick(geometry_msgs::Pose &pose, int id, double offset);
    bool moveToPosePlace(geometry_msgs::Pose &pose, int id, double offset);
    bool setGripper(std::string mode);
    void setupPlannigScene(int mode);
    void unSetupPlanningScene();
    void removeCollisionObject(std::string name);
    bool virtualAttachDetach(std::string modelName1, std::string linkName1, 
                             std::string modelName2, std::string linkName2, 
                             std::string mode);
    void publishPlanningScene();


};

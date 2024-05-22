#pragma once

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/message_filter.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <node_c_msgs/Transform.h>
#include <node_c_msgs/Detections.h>
#include <node_c_msgs/Detection.h>


class PosesDetectionPublisher
{
private:
    ros::NodeHandle node;
    ros::Publisher publisher;
    geometry_msgs::Pose transformPose(geometry_msgs::Pose pose);

public:
    PosesDetectionPublisher(ros::NodeHandle node);
    void publish();
};


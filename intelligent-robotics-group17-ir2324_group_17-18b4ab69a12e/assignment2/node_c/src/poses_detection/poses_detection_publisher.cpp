#include <node_c/poses_detection/poses_detection_publisher.h>

PosesDetectionPublisher::PosesDetectionPublisher(ros::NodeHandle node)
{
    this->node = node;
    this->publisher = this->node.advertise<node_c_msgs::Detections>("/poses_detection",1000);
}

geometry_msgs::Pose PosesDetectionPublisher::transformPose(geometry_msgs::Pose pose)
{
    geometry_msgs::Pose ret;

    ros::NodeHandle node;
    ros::ServiceClient client = node.serviceClient<node_c_msgs::Transform>("/poses_transformer");
    node_c_msgs::Transform msg;
    msg.request.source_frame = "xtion_rgb_optical_frame";
    msg.request.target_frame = "base_link";
    msg.request.pose = pose;

    if (client.call(msg))
        ret = msg.response.pose;
    else
        ROS_ERROR("Failed to call service");

    return ret;
}

void PosesDetectionPublisher::publish()
{
    apriltag_ros::AprilTagDetectionArray::ConstPtr detections = ros::topic::waitForMessage<apriltag_ros::AprilTagDetectionArray>("/tag_detections", node);
    node_c_msgs::Detections msg;
    for (int i = 0; i < detections->detections.size(); i++)
    {    
        apriltag_ros::AprilTagDetection detection = (detections->detections)[i];
        geometry_msgs::PoseWithCovarianceStamped poseCovarianceStamped = detection.pose;
        geometry_msgs::PoseWithCovariance poseCovariance = poseCovarianceStamped.pose;
        
        node_c_msgs::Detection detectionMessage;
        detectionMessage.pose = transformPose(poseCovariance.pose);
        detectionMessage.id = detection.id.at(0);
        msg.poses.push_back(detectionMessage);
    }

    publisher.publish(msg);
}


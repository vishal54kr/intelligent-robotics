#include <node_b/head_movement/head_movement_server.h>

HeadMovementServer::HeadMovementServer() : 
      headClient("/head_controller/follow_joint_trajectory", true)
    , pointHeadClient("/head_controller/point_head_action", true)
{
    this->node = node;
    this->server = this->node.advertiseService(
        "/head_movement", 
        &HeadMovementServer::headMovementCallback, 
        this
    );
    this->goal.trajectory.joint_names.push_back("head_1_joint");
    this->goal.trajectory.joint_names.push_back("head_2_joint");
    this->goal.trajectory.points.resize(1); 
    
    // Get the camera intrinsic parameters from the ROS topic
    ROS_INFO("HeadMovementServer: Waiting for camera intrinsics parameters");
    sensor_msgs::CameraInfoConstPtr msg = ros::topic::waitForMessage <sensor_msgs::CameraInfo>(
        "/xtion/rgb/camera_info", 
        ros::Duration(10.0)
    );
    if(msg.use_count() > 0)
    {
        cameraIntrinsics = cv::Mat::zeros(3,3,CV_64F);
        cameraIntrinsics.at<double>(0, 0) = msg->K[0]; //fx
        cameraIntrinsics.at<double>(1, 1) = msg->K[4]; //fy
        cameraIntrinsics.at<double>(0, 2) = msg->K[2]; //cx
        cameraIntrinsics.at<double>(1, 2) = msg->K[5]; //cy
        cameraIntrinsics.at<double>(2, 2) = 1;
    }

    pointHeadClient.waitForServer(ros::Duration(20.0)); 
    ROS_INFO("HeadMovementServer started");
}

void HeadMovementServer::pointHead(cv::Point2d center)
{
	ROS_INFO("Pointing head to %d, %d", center.x, center.y);

    geometry_msgs::PointStamped pointStamped;

    pointStamped.header.frame_id = "xtion_rgb_optical_frame";

    // compute normalized coordinates of the selected pixel
    double x = (center.x - cameraIntrinsics.at<double>(0, 2)) / cameraIntrinsics.at<double>(0, 0);
    double y = (center.y - cameraIntrinsics.at<double>(1, 2)) / cameraIntrinsics.at<double>(1, 1);
    double Z = 1.0; // define an arbitrary distance
    pointStamped.point.x = x * Z;
    pointStamped.point.y = y * Z;
    pointStamped.point.z = Z;

    // build the action goal
    control_msgs::PointHeadGoal goal;
    // the goal consists in making the Z axis of the cameraFrame to point towards the pointStamped
    goal.pointing_frame = "xtion_rgb_optical_frame";
    goal.pointing_axis.x = 0.0;
    goal.pointing_axis.y = 0.0;
    goal.pointing_axis.z = 1.0;
    goal.max_velocity = 0.25;
    goal.target = pointStamped;

    pointHeadClient.sendGoal(goal);
    ros::Duration(3).sleep();
}

bool HeadMovementServer::headMovementCallback(node_b_msgs::HeadMovement::Request &req, node_b_msgs::HeadMovement::Response &res)
{
    int modality = req.mode;

    // Resize to put the value of the joints
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    goal.trajectory.points.at(0).positions.resize(goal.trajectory.joint_names.size());

    if (modality == 3)
    { // move the head according to pixel passed
        pointHead(cv::Point2d(req.point.x, req.point.y));
        return true;
    }

    switch (modality)
    {
    case 0: // move the head to the initial position
        goal.trajectory.points.at(0).positions.at(0) = 0.0;
        goal.trajectory.points.at(0).positions.at(1) = 0.0;
        break;
    case 1: // move the head down
        goal.trajectory.points.at(0).positions.at(0) = 0.0;
        goal.trajectory.points.at(0).positions.at(1) = -0.4; 
        break;
    case 2: // move the head according to the joints values passed as parameters
        goal.trajectory.points.at(0).positions.at(0) = req.rotation_horizontal;
        goal.trajectory.points.at(0).positions.at(1) = req.rotation_vertical;
        break;
    }
    goal.trajectory.points.at(0).time_from_start = ros::Duration(2.0);

    headClient.sendGoal(goal);
    ros::Duration(3).sleep();
    res.state = true;

    return true;
}

#include <node_c/table_objects_detection/table_objects_detection_server.h>

TableObjectsDetectionServer::TableObjectsDetectionServer() : 
    actionServer(
        nodeHead, 
        "/table_objects_detection", 
        boost::bind(&TableObjectsDetectionServer::startDetection, this, _1),
        false
    )
{
    actionServer.start();
}

void TableObjectsDetectionServer::pointArea(cv::Rect colorizedArea)
{
    node_b_msgs::HeadMovement srv;
    srv.request.mode = 3;

    // send to the manipulation head movement node the image point where to look 
    geometry_msgs::Point center;
    center.x = colorizedArea.x + colorizedArea.width / 2;
    center.y = colorizedArea.y + colorizedArea.height / 2;
    srv.request.point = center;
    if (clientHead.call(srv))
    {
        ROS_INFO("Head moved");
    }
    else
    {
        ROS_ERROR("Failed to call service for head movement");
    }
}

void TableObjectsDetectionServer::resetPosition()
{
    node_b_msgs::HeadMovement srv;
    srv.request.mode = 2;

    // send to the manipulation head movement node the joints values of the table position
    srv.request.rotation_vertical = joint2;
    srv.request.rotation_horizontal = joint1;
    if (clientHead.call(srv))
    {
        ROS_INFO("Reset Head");
    }
    else
    {
        ROS_ERROR("Failed to call service for head movement");
    }
}

void TableObjectsDetectionServer::startDetection(const node_c_msgs::ObjectsDetectionGoalConstPtr &goal)
{
	ROS_INFO("Detecting Table");
	
    this->colorID = goal->colorID;
    this->feedback.state = 0;
    actionServer.publishFeedback(feedback);
    moveHeadDown();
    pointTable();
    pointTable();
	
	ROS_INFO("Saving joint position of the table: %d, %d", joint1, joint2);

    // save joints position of the table for later
    ros::NodeHandle node;
    sensor_msgs::JointState::ConstPtr jointStateMessagePointer = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", node);
    joint1 = jointStateMessagePointer->position[9];
    joint2 = jointStateMessagePointer->position[10];

    ROS_INFO("Searching for the colored object on the table");
    this->feedback.state = 1;
    actionServer.publishFeedback(feedback);
    pointColor();

    // reset head to table position
    resetPosition();

    ROS_INFO("Searching for the other objects in the table");
    this->feedback.state = 2;
    actionServer.publishFeedback(feedback);
    pointObstacleTags();

    ROS_INFO("Detection finished");

    //reset of the server
    observedIds.clear();
    joint1 = 0.0;
    joint2 = 0.0;

    // set the action state to succeeded
    actionServer.setSucceeded(result);

    result.detections.poses.clear();
    
}

void TableObjectsDetectionServer::moveHeadDown()
{
    // move the head down a little by using the manipulation package
    ros::ServiceClient client = nodeHead.serviceClient<node_b_msgs::HeadMovement>("/head_movement");
    node_b_msgs::HeadMovement srv;
    srv.request.mode = 1;
    if (client.call(srv))
    {
        ROS_INFO("Moving the head down");
    }
    else
    {
        ROS_ERROR("Failed to call the head movement server");
    }
}

void TableObjectsDetectionServer::pointTable()
{
    // get the TIAGo camera image
    sensor_msgs::ImageConstPtr imgMsg = ros::topic::waitForMessage<sensor_msgs::Image>("/xtion/rgb/image_raw", nodeHead);
    cv_bridge::CvImagePtr cvImgPtr = cv_bridge::toCvCopy(imgMsg, sensor_msgs::image_encodings::BGR8);

    // Convert from BGR to HSV colorspace
	cv::Mat imgHSV;
    cv::cvtColor(cvImgPtr->image, imgHSV, cv::COLOR_BGR2HSV);

    // search for the color treshold
	cv::Mat thresholdedImg;
    cv::inRange(imgHSV, cv::Scalar(20, 63, 0), cv::Scalar(23, 255, 81), thresholdedImg);
    cv::Rect boundingRect = cv::boundingRect(thresholdedImg);

    // point to the table
    pointArea(boundingRect);
}

void TableObjectsDetectionServer::pointColor()
{
    // get the TIAGo camera image
    sensor_msgs::ImageConstPtr imgMsg = ros::topic::waitForMessage<sensor_msgs::Image>("/xtion/rgb/image_raw", nodeHead);
    cv_bridge::CvImagePtr cvImgPtr = cv_bridge::toCvCopy(imgMsg, sensor_msgs::image_encodings::BGR8);

    cv::Mat imgHSV;
    // Convert from BGR to HSV colorspace
    cv::cvtColor(cvImgPtr->image, imgHSV, cv::COLOR_BGR2HSV);

    cv::Mat thresholdedImg;

    // search for the color treshold
    if (colorID == 1)
    { // BLUE
        cv::inRange(imgHSV, cv::Scalar(110, 50, 50), cv::Scalar(130, 255, 255), thresholdedImg);
    }
    else if (colorID == 2)
    { // GREEN
        cv::inRange(imgHSV, cv::Scalar(36, 25, 25), cv::Scalar(70, 255, 255), thresholdedImg);
    }
    else if (colorID == 3)
    { // RED
        cv::inRange(imgHSV, cv::Scalar(0, 50, 50), cv::Scalar(10, 255, 255), thresholdedImg);
    }

    cv::Rect boundingRect = cv::boundingRect(thresholdedImg);

    // point to the color
    pointArea(boundingRect);

    //record the ids that tiago see
    ros::NodeHandle node;
    node_c_msgs::Detections::ConstPtr detectionsMessagePointer = ros::topic::waitForMessage<node_c_msgs::Detections>("/poses_detection", node);
    for (int i = 0; i < detectionsMessagePointer->poses.size(); i++)
    {
        observedIds.insert(detectionsMessagePointer->poses.at(i).id);
        this->result.detections.poses.push_back(detectionsMessagePointer->poses.at(i));
    }
}

void TableObjectsDetectionServer::pointObstacleTags()
{
	ROS_INFO("Detecting AprilTags");
	
    // get the TIAGo camera image
    sensor_msgs::ImageConstPtr imgMsg = ros::topic::waitForMessage<sensor_msgs::Image>("/xtion/rgb/image_raw", nodeHead);
    cv_bridge::CvImagePtr cvImgPtr = cv_bridge::toCvCopy(imgMsg, sensor_msgs::image_encodings::BGR8);

    // find the black color of the AprilTags
    cv::Mat imgHSV;
    cv::cvtColor(cvImgPtr->image, imgHSV, cv::COLOR_BGR2HSV);
    cv::Mat thresholdedImgBlack;
    cv::inRange(imgHSV, cv::Scalar(0, 0, 0), cv::Scalar(50, 50, 50), thresholdedImgBlack);

    // extract the rectangles of the AprilTags by using tag black colors
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(thresholdedImgBlack, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    std::vector<std::vector<cv::Point>> contours_poly(contours.size());
    std::vector<cv::Rect> boundRect(contours.size());
    std::vector<cv::Point2f> centers(contours.size());
    std::vector<float> radius(contours.size());

    for (size_t i = 0; i < contours.size(); i++)
    {
        cv::approxPolyDP(contours[i], contours_poly[i], 3, true);
        boundRect[i] = cv::boundingRect(contours_poly[i]);
        minEnclosingCircle(contours_poly[i], centers[i], radius[i]);
    }

    cv::groupRectangles(boundRect, 1, 0.85);

    // boundRect contains all the rectangles of the april tags detected, we need to go through them
    // but first we need to remove the colored object tag because we have already seen it

    cv::Mat imgHSVObject;
    cv::cvtColor(cvImgPtr->image, imgHSVObject, cv::COLOR_BGR2HSV);

    cv::Mat thresholdedImgObject;

    if (colorID == 1)
    { // BLUE
        cv::inRange(imgHSV, cv::Scalar(110, 50, 50), cv::Scalar(130, 255, 255), thresholdedImgObject);
    }
    else if (colorID == 2)
    { // GREEN
        cv::inRange(imgHSV, cv::Scalar(36, 25, 25), cv::Scalar(70, 255, 255), thresholdedImgObject);
    }
    else if (colorID == 3)
    { // RED
        cv::inRange(imgHSV, cv::Scalar(0, 50, 50), cv::Scalar(10, 255, 255), thresholdedImgObject);
    }

    //remove the colored object tag from the list of tags that must be visited
    cv::Rect boundingRectObject = cv::boundingRect(thresholdedImgObject);
    std::vector<cv::Rect> finalRects;
    for (int i = 0; i < boundRect.size(); i++)
    {
        cv::Rect intersection = boundRect[i] & boundingRectObject;
        if (intersection.area() == 0)
            finalRects.push_back(boundRect[i]);
    }

	std::string print = "Rectangles to check: " + std::to_string(finalRects.size());
	ROS_INFO("Rectangles to check: %d", finalRects.size());
	
    ros::NodeHandle node;
 
    // starting the exploration and record the ids that tiago see
    for (int i = 0; i < finalRects.size(); i++)
    {
        pointArea(finalRects[i]);

        node_c_msgs::Detections::ConstPtr detectionsMessagePointer = ros::topic::waitForMessage<node_c_msgs::Detections>("/poses_detection", node);
        for (int i = 0; i < detectionsMessagePointer->poses.size(); i++)
        {
            auto it = observedIds.find(detectionsMessagePointer->poses.at(i).id);
            if (it == observedIds.end())
            {
                this->result.detections.poses.push_back(detectionsMessagePointer->poses.at(i));
                observedIds.insert(detectionsMessagePointer->poses.at(i).id);
            }
        }

        // reset head
        resetPosition();
    }
	
	ROS_INFO("Number of detected april tags: %d", this->result.detections.poses.size());
}



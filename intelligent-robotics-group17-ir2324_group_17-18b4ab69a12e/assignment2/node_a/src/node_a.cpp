#include "ros/ros.h"
#include "tiago_iaslab_simulation/Objs.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "move_base_msgs/MoveBaseActionResult.h"
#include "move_base_msgs/MoveBaseActionFeedback.h"
#include <sensor_msgs/LaserScan.h>
#include <actionlib/server/simple_action_server.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>
#include <cmath>
#include <iostream>
#include <string>
#include <actionlib/client/simple_action_client.h>

#include <navigation/NavigateAction.h>

// DETECTION
#include <node_c_msgs/Detection.h>
#include <node_c_msgs/Detections.h>
#include <node_c_msgs/Transform.h>
#include <node_c_msgs/Circles.h>
#include <node_c_msgs/ObjectsDetectionAction.h>
#include <node_c_msgs/PoseTableDetector.h>

// MANIPULATION
#include <node_b_msgs/TorsoLifter.h>
#include <node_b_msgs/PickPlaceAction.h>
#include <node_b_msgs/HeadMovement.h>


// POSES
// BLUE
const static double POSE_POSITION_X_ID1 = 8.17761;
const static double POSE_POSITION_Y_ID1 = -1.9486;
const static double POSE_ORIENTATION_Z_ID1 = -0.708202;
const static double POSE_ORIENTATION_W_ID1 = 0.70601;

// RED
const static double POSE_POSITION_X_ID3 = 7.44911;
const static double POSE_POSITION_Y_ID3 = -1.9486;
const static double POSE_ORIENTATION_Z_ID3 = -0.709458;
const static double POSE_ORIENTATION_W_ID3 = 0.704748;

// GREEN
const static double POSE_POSITION_X_ID2 = 7.69915;
const static double POSE_POSITION_Y_ID2 = -4.02856;
const static double POSE_ORIENTATION_Z_ID2 = 0.686435;
const static double POSE_ORIENTATION_W_ID2 = 0.727191;

// WAYPOINT1 (NEAR BROWN CIRCLE OBSTACLE)
const static double POSE_POSITION_X_WAYPOINT1 = 8.44668;
const static double POSE_POSITION_Y_WAYPOINT1 = -0.0281095;
const static double POSE_ORIENTATION_Z_WAYPOINT1 = -0.709564;
const static double POSE_ORIENTATION_W_WAYPOINT1 = 0.704641;

// WAYPOINT2 (NEAR SECOND ROOM ANDATA)
const static double POSE_POSITION_X_WAYPOINT2 = 8.83445;
const static double POSE_POSITION_Y_WAYPOINT2 = -4.12874;
const static double POSE_ORIENTATION_Z_WAYPOINT2 = 0.0;
const static double POSE_ORIENTATION_W_WAYPOINT2 = 1.0;

// WAYPOINT3 (ONLY FOR RED AND BLUE)
const static double POSE_POSITION_X_WAYPOINT3 = 8.80689464438;
const static double POSE_POSITION_Y_WAYPOINT3 = -1.74279238433;
const static double POSE_ORIENTATION_Z_WAYPOINT3 = -0.65988535475;
const static double POSE_ORIENTATION_W_WAYPOINT3 = 0.751366301205;

// WAYPOINT4 (DOCKING POSITION)
const static double POSE_POSITION_X_WAYPOINT4 = 11.463908;
const static double POSE_POSITION_Y_WAYPOINT4 = -2.100030;
const static double POSE_ORIENTATION_Z_WAYPOINT4 = 0.710590;
const static double POSE_ORIENTATION_W_WAYPOINT4 = 0.703605;

// WAYPOINT5 (NEAR SECOND ROOM RITORNO)
const static double POSE_POSITION_X_WAYPOINT5 = 8.83445;
const static double POSE_POSITION_Y_WAYPOINT5 = -4.12874;
const static double POSE_ORIENTATION_Z_WAYPOINT5 = 0.689923;
const static double POSE_ORIENTATION_W_WAYPOINT5 = 0.723883;

// POSE TABLE GREEN
const static double POSE_POSITION_X_TABLE_ID2 = 11.4580622631;
const static double POSE_POSITION_Y_TABLE_ID2 = -1.01143845239;
const static double POSE_ORIENTATION_Z_TABLE_ID2 = 0.728407926567;
const static double POSE_ORIENTATION_W_TABLE_ID2 = 0.685143702091;

// POSE TABLE RED
const static double POSE_POSITION_X_TABLE_ID3 = 10.524329443;
const static double POSE_POSITION_Y_TABLE_ID3 = -1.01143845239;
const static double POSE_ORIENTATION_Z_TABLE_ID3 = 0.709314931812;
const static double POSE_ORIENTATION_W_TABLE_ID3 = 0.704891713321;

// POSE TABLE BLUE
const static double POSE_POSITION_X_TABLE_ID1 = 12.6226028683;
const static double POSE_POSITION_Y_TABLE_ID1 = -1.01143845239;
const static double POSE_ORIENTATION_Z_TABLE_ID1 = 0.709314931812;
const static double POSE_ORIENTATION_W_TABLE_ID1 = 0.704891713321;

// TORSO LIFT JOINT INITIAL VALUE
const static double TORSO_LIFT_JOINT_INITIAL = 0.15;

// TORSO LIFT JOINT TARGET VALUE
const static double TORSO_LIFT_JOINT_TARGET = 0.340;

// HEIGHT LASER
const static double HEIGHT_LASER = 0.0945;

// BASE_LASER_LINK REFERENCE FRAME
const static std::string BASE_LASER_LINK_REFERENCE_FRAME = "base_laser_link";

// MAP REFERENCE FRAME
const static std::string MAP_REFERENCE_FRAME = "map";

// BASE_LINK REFERENCE FRAME
const static std::string BASE_LINK_REFERENCE_FRAME = "base_link";

// CIRCLES DETECTION PARAMETERS
const static double MAX_CIRCLE_RADIUS_TABLE_PICK = 0.1;
const static double MIN_CIRCLE_RADIUS_TABLE_PICK = 0.07;
const static double RADIUS_ENLARMENT_TABLE_PICK = 0.05;

const static double MAX_CIRCLE_RADIUS_DETECTION = 0.35;
const static double MIN_CIRCLE_RADIUS_DETECTION = 0.05;
const static double RADIUS_ENLARMENT_DETECTION = 0.05;

// THREADSHOLD CIRCLES FOR PLACE TABLES
const static double THRESHOLD_RADIUS_CIRCLES_PLACE_TABLE = 0.1;

// PICK PLACE MODE
const static unsigned char PICK_MODE = 0;
const static unsigned char PLACE_MODE = 1;

// ORIENTATION PLACE TABLE
const static double ORIENTATION_W_PLACE_TABLES = 0.685143702091;
const static double ORIENTATION_Z_PLACE_TABLES = 0.728407926567;

// OFFSET PLACE TABLE
const static double OFFSET_PLACE_POSITION_TABLE = -0.65;

// Needed for detection result
std::vector<node_c_msgs::Detection> objectsDetections;

//FUNCTIONS
void convertIdsToDestinationPoses(std::vector<int> &ids, std::vector<geometry_msgs::Pose> &poses);
void moveRobotTo(ros::NodeHandle nh, double positionX, double positionY, double orientationZ, double orientationW);
void liftTorso(ros::NodeHandle nh, bool mode);
geometry_msgs::Pose getPosePickTable(ros::NodeHandle nh);
geometry_msgs::Pose getPosePlaceTable(ros::NodeHandle nh, int id);
void moveHead(ros::NodeHandle nh);
geometry_msgs::Pose convertPose(ros::NodeHandle nh, geometry_msgs::Pose pose, std::string sourceFrame, std::string targetFrame);
int getIndexClosestCircle(std::vector<node_c_msgs::Circle> circles, int id);
std::vector<node_c_msgs::Circle> detectCircles(ros::NodeHandle nh);
void doneCb(const actionlib::SimpleClientGoalState &state, const node_c_msgs::ObjectsDetectionResultConstPtr &resultPtr);
void activeCb();
void feedbackCb(const node_c_msgs::ObjectsDetectionFeedbackConstPtr &feedback);
void PickPlaceDoneCb(const actionlib::SimpleClientGoalState &state, const node_b_msgs::PickPlaceResultConstPtr &resultPtr);
void PickPlaceActiveCb();
void PickPlaceFeedbackCb(const node_b_msgs::PickPlaceFeedbackConstPtr &feedbackPtr);
void doneCbnavigation(const actionlib::SimpleClientGoalState &state, const navigation::NavigateResultConstPtr &result);
void activeCbnavigation();
void feedbackCbnavigation(const navigation::NavigateFeedbackConstPtr &feedbackPtr);


int main(int argc, char **argv) {
    
    // Initialize ROS node
    ros::init(argc, argv, "node_A");
    ros::NodeHandle nh;
	ros::Duration(1.0).sleep();
	
	//check for extra point flag
	bool extraArgs = false;
    if(argc > 1){
		std::string arg = argv[1];
		
        if(arg.compare("extra")==0) {			
            extraArgs = true;
		}
    }
	
	// Declare varibale
    std::vector<int> ids;
    geometry_msgs::Pose pose;
    std::vector<geometry_msgs::Pose> poses;
    node_c_msgs::PoseTableDetector poseTableDetectorMessage;
    sensor_msgs::LaserScanConstPtr scanMessagePointer;
    node_c_msgs::Detections::ConstPtr detectionsMessagePointer;

    node_c_msgs::Detection detection;
    geometry_msgs::Pose posePickTable;
    geometry_msgs::Pose posePlaceTableGlobal;
    geometry_msgs::Pose posePlaceTable;

    // Client declarations
    ros::ServiceClient client = nh.serviceClient<tiago_iaslab_simulation::Objs>("/human_objects_srv");
	actionlib::SimpleActionClient<node_b_msgs::PickPlaceAction> pick_place_client ("/pick_place", true);
	actionlib::SimpleActionClient<node_c_msgs::ObjectsDetectionAction> table_objects_detector_client ("/table_objects_detection", true);
	ros::ServiceClient pose_table_detector_client = nh.serviceClient<node_c_msgs::PoseTableDetector>("/pose_table_detector");

    tiago_iaslab_simulation::Objs srv;
    srv.request.ready = true;
    srv.request.all_objs  = true;

    if (client.call(srv)) {
        ROS_INFO("Objs: %ld", srv.response.ids.size());
        ids = srv.response.ids;
		
		std::vector<geometry_msgs::Pose> poses;
		convertIdsToDestinationPoses(ids, poses);
		
		// Move robot to waypoint
		ROS_INFO("Moving to waypoint 1");
		moveRobotTo(nh, POSE_POSITION_X_WAYPOINT1, POSE_POSITION_Y_WAYPOINT1, POSE_ORIENTATION_Z_WAYPOINT1, POSE_ORIENTATION_W_WAYPOINT1);

		for (int j = 0; j < poses.size(); j++){
			// Get current pose
			pose = poses.at(j);

			// Move robot to the correct position near table
			ROS_INFO("Moving near table for object %d", ids.at(j));
			
			if(j==0 && ids.at(j) == 2){
				ROS_INFO("Going through waypoint 5 (only to pick green)");
				moveRobotTo(nh, POSE_POSITION_X_WAYPOINT5, POSE_POSITION_Y_WAYPOINT5, POSE_ORIENTATION_Z_WAYPOINT5, POSE_ORIENTATION_W_WAYPOINT5);
			}
			if (j > 0 && (ids.at(j) == 1 || ids.at(j) == 3)){
				ROS_INFO("Going through waypoint 3 (only to pick blue or red)");
				moveRobotTo(nh, POSE_POSITION_X_WAYPOINT3, POSE_POSITION_Y_WAYPOINT3, POSE_ORIENTATION_Z_WAYPOINT3, POSE_ORIENTATION_W_WAYPOINT3);
			}
			
			moveRobotTo(nh, pose.position.x, pose.position.y, pose.orientation.z, pose.orientation.w);

			// Lift Up the torso
			ROS_INFO("Lift up the torso");
            bool isLiftTorso = true;
			liftTorso(nh, isLiftTorso);

			// Get the poses of the tags
			ROS_INFO("Detecting table and objects");
			table_objects_detector_client.waitForServer();
            node_c_msgs::ObjectsDetectionGoal goal1;
            goal1.colorID = ids.at(j);
			
            table_objects_detector_client.sendGoal(goal1, &doneCb, &activeCb, &feedbackCb);
			table_objects_detector_client.waitForResult();

			for(int i = 0; i < objectsDetections.size(); i++)
				ROS_INFO("Detected object on table: %d", objectsDetections.at(i).id);

			// Move head back to original place
			ROS_INFO("Reset head");
			moveHead(nh);

			// Get the pose of the "pick" / "rectangular" table
			posePickTable = getPosePickTable(nh);

			// PICK PROCEDURE 
			ROS_INFO("Pick object %d", ids.at(j));
			
			// Add pick table
			detection.pose = posePickTable;
			detection.id = -1;
			objectsDetections.push_back(detection);

			// Send message to pick server
			pick_place_client.waitForServer();
            node_b_msgs::PickPlaceGoal goal2;

            // set the goal position
            goal2.id_object_pick_place = ids.at(j);
            goal2.mode = PICK_MODE;
            goal2.poses = objectsDetections;

            pick_place_client.sendGoal(goal2);

            pick_place_client.waitForResult();
			objectsDetections.clear();

			// Move torso down
			ROS_INFO("Lift down the torso");
            isLiftTorso = false;
			liftTorso(nh, isLiftTorso);
			
			ROS_INFO("Go to place point (waypoint 2, 3 and 4)");
			// Move to waypoint 3 if blue or red (1 or 3)
			if (ids.at(j) == 1 || ids.at(j) == 3){
				ROS_INFO("Moving to waypoint 3 (only to pick blue or red)");
				moveRobotTo(nh, POSE_POSITION_X_WAYPOINT3, POSE_POSITION_Y_WAYPOINT3, POSE_ORIENTATION_Z_WAYPOINT3, POSE_ORIENTATION_W_WAYPOINT3);
			}

			// Move to waypoint 2 (between rooms)
			moveRobotTo(nh, POSE_POSITION_X_WAYPOINT2, POSE_POSITION_Y_WAYPOINT2, POSE_ORIENTATION_Z_WAYPOINT2, POSE_ORIENTATION_W_WAYPOINT2);

			// Move to waypoint 4 (docking position)
			moveRobotTo(nh, POSE_POSITION_X_WAYPOINT4, POSE_POSITION_Y_WAYPOINT4, POSE_ORIENTATION_Z_WAYPOINT4, POSE_ORIENTATION_W_WAYPOINT4);

			// Get the pose of the "place" table / cylindrical table w.r.t. map reference frame
			posePlaceTableGlobal = getPosePlaceTable(nh, ids.at(j));

			if(extraArgs){ // if extra flag is true, detect cilinders with laser
                poseTableDetectorMessage.request.id = ids.at(j);
				poseTableDetectorMessage.request.circles = detectCircles(nh);
				pose_table_detector_client.call(poseTableDetectorMessage);

				// Save global position of the table
				posePlaceTableGlobal.position.x = poseTableDetectorMessage.response.circle.x;
				posePlaceTableGlobal.position.y = poseTableDetectorMessage.response.circle.y;

				// Move to correct position
				ROS_INFO("Going to table in position %lf, %lf", posePlaceTableGlobal.position.x, posePlaceTableGlobal.position.y);
				moveRobotTo(nh, posePlaceTableGlobal.position.x, posePlaceTableGlobal.position.y + OFFSET_PLACE_POSITION_TABLE, ORIENTATION_Z_PLACE_TABLES, ORIENTATION_W_PLACE_TABLES);
			}
			
			else { // if extra flag is false, use static cilinders positions
                // Select correct table and go there
				double posePositionX, posePositionY, posePositionZ, poseOrientationW;
				posePositionX = posePositionY = posePositionZ = poseOrientationW = 0;
				
                if (ids.at(j) == 1) {
                    double posePositionX = POSE_POSITION_X_TABLE_ID1;
                    double posePositionY = POSE_POSITION_Y_TABLE_ID1;
                    double posePositionZ = POSE_ORIENTATION_Z_TABLE_ID1;
                    double poseOrientationW = POSE_ORIENTATION_W_TABLE_ID1;

                } else if (ids.at(j) == 2) {
                    double posePositionX = POSE_POSITION_X_TABLE_ID2;
                    double posePositionY = POSE_POSITION_Y_TABLE_ID2;
                    double posePositionZ = POSE_ORIENTATION_Z_TABLE_ID2;
                    double poseOrientationW = POSE_ORIENTATION_W_TABLE_ID2;

                } else if (ids.at(j) == 3) {
                    double posePositionX = POSE_POSITION_X_TABLE_ID3;
                    double posePositionY = POSE_POSITION_Y_TABLE_ID3;
                    double posePositionZ = POSE_ORIENTATION_Z_TABLE_ID3;
                    double poseOrientationW = POSE_ORIENTATION_W_TABLE_ID3;
                }
                
				ROS_INFO("Going to table %d", ids.at(j));
                moveRobotTo(nh, posePositionX, posePositionY, posePositionZ, poseOrientationW);
			}

			// Lift Up the torso
			ROS_INFO("Lift up the torso");
            isLiftTorso = true;
			liftTorso(nh, isLiftTorso);

            ROS_INFO("Ready to place, sending goal");

			// Place procedure
			// Convert the globalPose of the place table to the base_link_reference frame
			posePlaceTable = convertPose(nh, posePlaceTableGlobal, MAP_REFERENCE_FRAME, BASE_LINK_REFERENCE_FRAME);
			posePlaceTable.position.z = -HEIGHT_LASER;

			// Add place table
			detection.pose = posePlaceTable;
			detection.id = -1;
			objectsDetections.push_back(detection);

			// Send message to place server
			pick_place_client.waitForServer();
            node_b_msgs::PickPlaceGoal goal3;

            // set the goal position
            goal3.id_object_pick_place = ids.at(j);
            goal3.mode = PLACE_MODE;
            goal3.poses = objectsDetections;

            pick_place_client.sendGoal(goal3, &PickPlaceDoneCb, &PickPlaceActiveCb, &PickPlaceFeedbackCb);

            pick_place_client.waitForResult();
			ROS_INFO("Object placed");
			objectsDetections.clear();

			// Move torso down
			ROS_INFO("Lift down the torso");
            isLiftTorso = false;
			liftTorso(nh, isLiftTorso);

			// Move to waypoint 5 (near rooms, rotated)
			ROS_INFO("Go to waypoint 5");
			moveRobotTo(nh, POSE_POSITION_X_WAYPOINT5, POSE_POSITION_Y_WAYPOINT5, POSE_ORIENTATION_Z_WAYPOINT5, POSE_ORIENTATION_W_WAYPOINT5);
		}

		ros::spin();
    } else {
        ROS_ERROR("Error: Can't call the service objs");
        return 1;
    }

    return 0;
}

void convertIdsToDestinationPoses(std::vector<int> &ids, std::vector<geometry_msgs::Pose> &poses)
{
    ROS_INFO("Converting ids to destination poses");
    for (int id : ids)
    {
        geometry_msgs::Pose pose;
        if (id == 1) {

            pose.position.x = POSE_POSITION_X_ID1;
            pose.position.y = POSE_POSITION_Y_ID1;
            pose.orientation.z = POSE_ORIENTATION_Z_ID1;
            pose.orientation.w = POSE_ORIENTATION_W_ID1;

        } else if (id == 2) {

            pose.position.x = POSE_POSITION_X_ID2;
            pose.position.y = POSE_POSITION_Y_ID2;
            pose.orientation.z = POSE_ORIENTATION_Z_ID2;
            pose.orientation.w = POSE_ORIENTATION_W_ID2;

        } else if (id == 3) {

            pose.position.x = POSE_POSITION_X_ID3;
            pose.position.y = POSE_POSITION_Y_ID3;
            pose.orientation.z = POSE_ORIENTATION_Z_ID3;
            pose.orientation.w = POSE_ORIENTATION_W_ID3;
        }

        poses.push_back(pose);
    }
}

void moveRobotTo(ros::NodeHandle nh, double positionX, double positionY, double orientationZ, double orientationW)
{
	actionlib::SimpleActionClient<navigation::NavigateAction> navigation_client ("/navigation_server", true);
    navigation_client.waitForServer();
    navigation::NavigateGoal goal;

    // set the goal position
    goal.x = positionX;
    goal.y = positionY;
    goal.orZ = orientationZ;
    goal.orW = orientationW;

    navigation_client.sendGoal(goal,&doneCbnavigation,&activeCbnavigation,&feedbackCbnavigation);  
	navigation_client.waitForResult();
}

void liftTorso(ros::NodeHandle nh, bool mode)
{
    ros::ServiceClient torso_lifter_client = nh.serviceClient<node_b_msgs::TorsoLifter>("/torso_lifter");
    node_b_msgs::TorsoLifter torsoLifterMessage;

    double torsoLiftJoint;

    if (mode)
        torsoLiftJoint = TORSO_LIFT_JOINT_TARGET;
    else
        torsoLiftJoint = TORSO_LIFT_JOINT_INITIAL;

    torsoLifterMessage.request.joint_value = torsoLiftJoint;
    torso_lifter_client.call(torsoLifterMessage);
}

geometry_msgs::Pose getPosePickTable(ros::NodeHandle nh)
{
    // Messages
    node_c_msgs::Circles circlesMessage;
    node_c_msgs::Circle circleMessage;
    geometry_msgs::Pose poseRaw;
    geometry_msgs::Pose poseRet;

    std::string baseLaserLinkReferenceFrame = BASE_LASER_LINK_REFERENCE_FRAME;
    std::string baseLinkReferenceFrame = BASE_LINK_REFERENCE_FRAME;

    // Clients
    ros::ServiceClient obstacle_extractor_client = nh.serviceClient<node_c_msgs::Circles>("/obstacle_extractor");

    // Read once the /scan
    sensor_msgs::LaserScanConstPtr scanMessagePointer = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan", nh);
    circlesMessage.request.scan_data = *scanMessagePointer;

    circlesMessage.request.max_circle_radius = MAX_CIRCLE_RADIUS_TABLE_PICK;
    circlesMessage.request.min_circle_radius = MIN_CIRCLE_RADIUS_TABLE_PICK;
    circlesMessage.request.radius_enlargment = RADIUS_ENLARMENT_TABLE_PICK;

    // Extract circles
    obstacle_extractor_client.call(circlesMessage);

    // Find circle corresponding to current id
    circleMessage = circlesMessage.response.cirles.at(0);

    // Convert to pose
    poseRaw.orientation.x = 0.0;
    poseRaw.orientation.y = 0.0;
    poseRaw.orientation.z = 0.0;
    poseRaw.orientation.w = 1.0;

    poseRaw.position.x = circleMessage.y;
    poseRaw.position.y = -circleMessage.x;
    poseRaw.position.z = 0.0;

    // Convert pose to correct reference frame
    poseRet = convertPose(nh, poseRaw, baseLaserLinkReferenceFrame, baseLinkReferenceFrame);

    // Set height of rectangular table
    poseRet.position.z = -HEIGHT_LASER;

    return poseRet;
}

geometry_msgs::Pose getPosePlaceTable(ros::NodeHandle nh, int id)
{
    double totalSum = 0.0;

    node_c_msgs::Circles circlesMessage;
    node_c_msgs::Circle circleMessage;
    geometry_msgs::Pose poseRaw;
    std::vector<node_c_msgs::Circle> thresholdedCircles;

    std::string baseLaserLinkReferenceFrame = BASE_LASER_LINK_REFERENCE_FRAME;
    std::string mapReferenceFrame = MAP_REFERENCE_FRAME;

    // Clients
    ros::ServiceClient obstacle_extractor_client = nh.serviceClient<node_c_msgs::Circles>("/obstacle_extractor");

    sensor_msgs::LaserScanConstPtr scanMessagePointer = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan", nh);
    circlesMessage.request.scan_data = *scanMessagePointer;

    circlesMessage.request.max_circle_radius = MAX_CIRCLE_RADIUS_DETECTION;
    circlesMessage.request.min_circle_radius = MIN_CIRCLE_RADIUS_DETECTION;
    circlesMessage.request.radius_enlargment = RADIUS_ENLARMENT_DETECTION;

    // Extract circles
    obstacle_extractor_client.call(circlesMessage);

    // Threshold small circles
    for (node_c_msgs::Circle circ : circlesMessage.response.cirles)
        if (THRESHOLD_RADIUS_CIRCLES_PLACE_TABLE < circ.radius)
            thresholdedCircles.push_back(circ);

    // Make the y of each circle equivalent to the avarege y
    for (node_c_msgs::Circle circlePoint : thresholdedCircles)
        totalSum += circlePoint.y;

    totalSum /= thresholdedCircles.size();

    for (int index = 0; index < thresholdedCircles.size(); index++)
        thresholdedCircles.at(index).y = totalSum;

    // Find circle corresponding to current id
    circleMessage = thresholdedCircles.at(getIndexClosestCircle(thresholdedCircles, id));

    // Convert to pose
    poseRaw.orientation.x = 0.0;
    poseRaw.orientation.y = 0.0;
    poseRaw.orientation.z = 0.0;
    poseRaw.orientation.w = 1.0;

    poseRaw.position.x = circleMessage.y;
    poseRaw.position.y = -circleMessage.x;
    poseRaw.position.z = 0.0;

    // Convert Pose to correct reference frame
    return convertPose(nh, poseRaw, baseLaserLinkReferenceFrame, mapReferenceFrame);
}

void moveHead(ros::NodeHandle nh)
{
    // Clients
    ros::ServiceClient head_movement_client = nh.serviceClient<node_b_msgs::HeadMovement>("/head_movement");

    // Messages
    node_b_msgs::HeadMovement headMovementMessage;

    headMovementMessage.request.mode = 0; // Std movement of the head
    head_movement_client.call(headMovementMessage);
}

geometry_msgs::Pose convertPose(ros::NodeHandle nh, geometry_msgs::Pose pose, std::string sourceFrame, std::string targetFrame)
{
    node_c_msgs::Transform transformMessage;
    // Clients
    ros::ServiceClient poses_transform_client = nh.serviceClient<node_c_msgs::Transform>("/poses_transformer");

    // Convert pose to correct reference frame
    transformMessage.request.pose = pose;
    transformMessage.request.source_frame = sourceFrame;
    transformMessage.request.target_frame = targetFrame;

    poses_transform_client.call(transformMessage);

    return transformMessage.response.pose;
}

int getIndexClosestCircle(std::vector<node_c_msgs::Circle> circles, int id)
{
    int j = 0;
    double actual = 0.0;

    if (id == 1) {

        // biggest x = blue
        actual = circles.at(0).x;
        for (int elem = j + 1; elem < circles.size(); elem++)
        {
            if (actual < circles.at(elem).x)
            {
                actual = circles.at(elem).x;
                j = elem;
            }
        }

    } else if (id == 2) {

        // central x = green
        actual = std::numeric_limits<double>::max();
        for (int elem = 0; elem < circles.size(); elem++)
        {
            double distance = std::abs(circles.at(elem).x);
            if (distance < actual)
            {
                actual = distance;
                j = elem;
            }
        }

    } else if (id == 3) {

        // smallest x = red
        actual = circles.at(0).x;
        for (int elem = j + 1; elem < circles.size(); elem++)
        {
            if (actual > circles.at(elem).x)
            {
                actual = circles.at(elem).x;
                j = elem;
            }
        }

    }
    return j;
}

std::vector<node_c_msgs::Circle> detectCircles(ros::NodeHandle nh)
{
    double totalSum = 0.0;

    node_c_msgs::Circles circlesMessage;
    geometry_msgs::Pose poseRaw;
    std::vector<node_c_msgs::Circle> thresholdedCircles;
    std::vector<node_c_msgs::Circle> returnedVec;

    std::string baseLaserLinkReferenceFrame = BASE_LASER_LINK_REFERENCE_FRAME;
    std::string mapReferenceFrame = MAP_REFERENCE_FRAME;

    // Clients
    ros::ServiceClient obstacle_extractor_client = nh.serviceClient<node_c_msgs::Circles>("/obstacle_extractor");


    sensor_msgs::LaserScanConstPtr scanMessagePointer = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan", nh);
    circlesMessage.request.scan_data = *scanMessagePointer;
    circlesMessage.request.max_circle_radius = MAX_CIRCLE_RADIUS_DETECTION;
    circlesMessage.request.min_circle_radius = MIN_CIRCLE_RADIUS_DETECTION;
    circlesMessage.request.radius_enlargment = RADIUS_ENLARMENT_DETECTION;

    // Extract circles
    obstacle_extractor_client.call(circlesMessage);

    // Threshold small circles
    for (node_c_msgs::Circle circ : circlesMessage.response.cirles)
        if (THRESHOLD_RADIUS_CIRCLES_PLACE_TABLE < circ.radius)
            thresholdedCircles.push_back(circ);

    // Make the y of each circle equivalent to the avarege y
    for (node_c_msgs::Circle circ : thresholdedCircles)
        totalSum += circ.y;

    totalSum /= thresholdedCircles.size();

    for (int index = 0; index < thresholdedCircles.size(); index++)
        thresholdedCircles.at(index).y = totalSum;

    for (node_c_msgs::Circle circ : thresholdedCircles)
    {
        // Convert to pose
        poseRaw.orientation.x = 0.0;
        poseRaw.orientation.y = 0.0;
        poseRaw.orientation.z = 0.0;
        poseRaw.orientation.w = 1.0;

        poseRaw.position.x = circ.y;
        poseRaw.position.y = -circ.x;
        poseRaw.position.z = 0.0;

        // Convert the pose to the correct reference frame
        geometry_msgs::Pose convertedPose = convertPose(nh, poseRaw, baseLaserLinkReferenceFrame, mapReferenceFrame);

        // Add circle conveted to the array to return
        node_c_msgs::Circle circleBack;
        circleBack.x = convertedPose.position.x;
        circleBack.y = convertedPose.position.y;
        circleBack.radius = circ.radius;
        returnedVec.push_back(circleBack);
    }

    return returnedVec;
}

// callback functions (most of them doesn't need to do anything)
void doneCb(const actionlib::SimpleClientGoalState &state, const node_c_msgs::ObjectsDetectionResultConstPtr &resultPtr){
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
        objectsDetections = resultPtr->detections.poses;
}

void activeCb(){}
void feedbackCb(const node_c_msgs::ObjectsDetectionFeedbackConstPtr &feedback){}
void PickPlaceDoneCb(const actionlib::SimpleClientGoalState &state, const node_b_msgs::PickPlaceResultConstPtr &resultPtr){}
void PickPlaceActiveCb(){}
void PickPlaceFeedbackCb(const node_b_msgs::PickPlaceFeedbackConstPtr &feedbackPtr){}
void doneCbnavigation(const actionlib::SimpleClientGoalState &state, const navigation::NavigateResultConstPtr &result){}
void activeCbnavigation(){}
void feedbackCbnavigation(const navigation::NavigateFeedbackConstPtr &feedbackPtr){}

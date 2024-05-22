#include <node_b/pick_place/pick_place_server.h>

static const double ARM_1_JOINT_VALUE_DEFAULT = 0.07;
static const double ARM_2_JOINT_VALUE_DEFAULT = 0.34;
static const double ARM_3_JOINT_VALUE_DEFAULT = -3.13;
static const double ARM_4_JOINT_VALUE_DEFAULT = 1.31;
static const double ARM_5_JOINT_VALUE_DEFAULT = 1.58;
static const double ARM_6_JOINT_VALUE_DEFAULT = 0.0;
static const double ARM_7_JOINT_VALUE_DEFAULT = 0.0;
static const double PICK_TABLE_LENGTH = 0.913;
static const double PICK_TABLE_WIDTH = 0.913;
static const double PICK_TABLE_HEIGHT = 0.775;
static const double HEXAGON_HEIGHT = 0.105364;
static const double HEXAGON_DIAMETER = 0.0325;
static const double TRIANGLE_LENGTH = 0.0325;
static const double TRIANGLE_WIDTH = 0.0325;
static const double TRIANGLE_HEIGHT = 0.021664;
static const double CUBE_LENGTH = 0.05;
static const double CUBE_WIDTH = 0.05;
static const double CUBE_HEIGHT = 0.05;
static const double CYLINDER_HEIGHT = 0.69;
static const double CYLINDER_RADIUS = 0.21;
static const double COLLISION_HEIGHT = 0.21073;
static const double COLLISION_DIAMETER = 0.0650;
static const double PLACE_TABLE_HEIGHT = 0.69;
static const double PLACE_TABLE_RADIUS = 0.225;
static const double GRIPPER_LENGTH = 0.23;
static const double OFFSET_APPROACH_DEPART = 0.1;
static const double OPEN_GRIPPER_MAX_SIZE = 0.045;

PickPlaceServer::PickPlaceServer(ros::NodeHandle node) : 
    server( node, 
            "/pick_place", 
            boost::bind(&PickPlaceServer::callback, 
            this, 
            _1)
    , false)
    , moveGroupInterfaceArm("arm")
    , moveGroupInterfaceGripper("gripper")
{
    server.start();
}



void PickPlaceServer::setupPlannigScene(int mode)
{
    ROS_INFO("Pick-Place setup started");

    // Varaibles
    std::map<int, geometry_msgs::Pose>::iterator it;
    geometry_msgs::Pose currentPose;
    int currentId;

    for (it = this->detectionsPosesIds.begin(); it != this->detectionsPosesIds.end(); it++)
    {
        currentId = it->first;
        currentPose = it->second;
        moveit_msgs::CollisionObject collisionObject;

        // frame_id
        collisionObject.header.frame_id = "base_link";

        // operation
        collisionObject.operation = collisionObject.ADD;

        // primitives resize
        collisionObject.primitives.resize(1);

        // primitives dimensions resize
        collisionObject.primitives.at(0).dimensions.resize(3);

        // primitive_poses
        collisionObject.primitive_poses.resize(1);
        collisionObject.primitive_poses.at(0).orientation = currentPose.orientation;

        if (mode == 0)
        {
            if (currentId < 0)
            {
                // id
                collisionObject.id = "pick_table";

                // primitives
                collisionObject.primitives.at(0).type = collisionObject.primitives.at(0).BOX;
                collisionObject.primitives.at(0).dimensions.at(0) = PICK_TABLE_LENGTH;
                collisionObject.primitives.at(0).dimensions.at(1) = PICK_TABLE_WIDTH;
                collisionObject.primitives.at(0).dimensions.at(2) = PICK_TABLE_HEIGHT;

                // primitive_poses position
                currentPose.position.z += (PICK_TABLE_HEIGHT / 2.0);
                collisionObject.primitive_poses.at(0).position = currentPose.position;
            }
            else if (currentId <= 3)
            {
                switch (currentId)
                {
                case 1:
                    // id
                    collisionObject.id = "hexagon";

                    // primitives
                    collisionObject.primitives.at(0).type = collisionObject.primitives.at(0).CYLINDER;
                    collisionObject.primitives.at(0).dimensions.at(0) = HEXAGON_HEIGHT;
                    collisionObject.primitives.at(0).dimensions.at(1) = (HEXAGON_DIAMETER / 2.0);

                    // primitive_poses position
                    currentPose.position.z -= (HEXAGON_HEIGHT / 2.0);
                    collisionObject.primitive_poses.at(0).position = currentPose.position;
                    break;
                case 2:
                    // id
                    collisionObject.id = "triangle";

                    // primitives
                    collisionObject.primitives.at(0).type = collisionObject.primitives.at(0).BOX;
                    collisionObject.primitives.at(0).dimensions.at(0) = TRIANGLE_LENGTH;
                    collisionObject.primitives.at(0).dimensions.at(1) = TRIANGLE_WIDTH;
                    collisionObject.primitives.at(0).dimensions.at(2) = TRIANGLE_HEIGHT;

                    // primitive_poses position
                    currentPose.position.z -= (TRIANGLE_HEIGHT / 2.0);
                    collisionObject.primitive_poses.at(0).position = currentPose.position;
                    break;
                case 3:
                    // id
                    collisionObject.id = "cube";

                    // primitives
                    collisionObject.primitives.at(0).type = collisionObject.primitives.at(0).BOX;
                    collisionObject.primitives.at(0).dimensions.at(0) = CUBE_LENGTH;
                    collisionObject.primitives.at(0).dimensions.at(1) = CUBE_WIDTH;
                    collisionObject.primitives.at(0).dimensions.at(2) = CUBE_HEIGHT;

                    // primitive_poses position
                    currentPose.position.z -= (CUBE_HEIGHT / 2.0);
                    collisionObject.primitive_poses.at(0).position = currentPose.position;
                    break;
                }
            }
            else
            {
                // id
                collisionObject.id = "collision_objects" + std::to_string(currentId);

                // primitives
                collisionObject.primitives.at(0).type = collisionObject.primitives.at(0).CYLINDER;
                collisionObject.primitives.at(0).dimensions.at(0) = COLLISION_HEIGHT;
                collisionObject.primitives.at(0).dimensions.at(1) = (COLLISION_DIAMETER / 2.0);

                // primitive_poses position
                currentPose.position.z -= (COLLISION_HEIGHT / 2.0);
                collisionObject.primitive_poses.at(0).position = currentPose.position;
            }
        }
        else
        {
            // id
            collisionObject.id = "place_table";

            // primitives
            collisionObject.primitives.at(0).type = collisionObject.primitives.at(0).CYLINDER;
            collisionObject.primitives.at(0).dimensions.at(0) = PLACE_TABLE_HEIGHT;
            collisionObject.primitives.at(0).dimensions.at(1) = PLACE_TABLE_RADIUS;

            // primitive_poses position
            currentPose.position.z += (PLACE_TABLE_HEIGHT / 2.0);
            collisionObject.primitive_poses.at(0).position = currentPose.position;
        }

        collisionObjects.push_back(collisionObject);
    }
    planningSceneInterface.applyCollisionObjects(collisionObjects);
	ROS_INFO("Pick-Place setup done");
} // setupPlannigScene

void PickPlaceServer::unSetupPlanningScene()
{
    // Variables
    std::vector<std::string> collisionObjectsIds;

    for (moveit_msgs::CollisionObject collisionObject : collisionObjects)
    {
        collisionObjectsIds.push_back(collisionObject.id);
    }

    // Delete planning scene
    planningSceneInterface.removeCollisionObjects(collisionObjectsIds);

    // Clear current collision objects
    collisionObjects.clear();
} // unSetupPlanningScene

bool PickPlaceServer::moveToDefaultPose()
{
    ROS_INFO("Moving to Default Pick-Place Pose");

    // Variables declaration
    std::map<std::string, double> targetPosition;
    std::vector<std::string> torsoArmJointNames;
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = false;

    // Set target pose
    targetPosition.insert({"arm_1_joint", ARM_1_JOINT_VALUE_DEFAULT});
    targetPosition.insert({"arm_2_joint", ARM_2_JOINT_VALUE_DEFAULT});
    targetPosition.insert({"arm_3_joint", ARM_3_JOINT_VALUE_DEFAULT});
    targetPosition.insert({"arm_4_joint", ARM_4_JOINT_VALUE_DEFAULT});
    targetPosition.insert({"arm_5_joint", ARM_5_JOINT_VALUE_DEFAULT});
    targetPosition.insert({"arm_6_joint", ARM_6_JOINT_VALUE_DEFAULT});
    targetPosition.insert({"arm_7_joint", ARM_7_JOINT_VALUE_DEFAULT});

    // Get tors arm joint names
    torsoArmJointNames = moveGroupInterfaceArm.getJoints();

    // Set group interface
    moveGroupInterfaceArm.setPlannerId("SBLkConfigDefault");
    moveGroupInterfaceArm.setStartStateToCurrentState();
    moveGroupInterfaceArm.setMaxVelocityScalingFactor(1.0);
    moveGroupInterfaceArm.setPlanningTime(50.);

    for (int i = 0; i < torsoArmJointNames.size(); i++)
        if (targetPosition.count(torsoArmJointNames.at(i)) > 0)
            moveGroupInterfaceArm.setJointValueTarget(
                torsoArmJointNames.at(i), 
                targetPosition.at(torsoArmJointNames.at(i))
            );

    if (moveGroupInterfaceArm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
    {
        ROS_INFO("\t Plan found!! ");
        success = true;
        moveGroupInterfaceArm.move();
    }
    return success;
} // moveToDefaultPose

bool PickPlaceServer::virtualAttachDetach(std::string modelName1, std::string linkName1, 
                                          std::string modelName2, std::string linkName2, 
                                          std::string mode)
{
    ROS_INFO("Setting virtual attach-detach");

    // Variables
    ros::NodeHandle n;
    ros::ServiceClient client;
    gazebo_ros_link_attacher::Attach service;
    bool success = false;
    std::string topic;

    // Define service message
    service.request.model_name_1 = modelName1;
    service.request.link_name_1 = linkName1;
    service.request.model_name_2 = modelName2;
    service.request.link_name_2 = linkName2;

    // Select correct topic
    if (mode.compare("attach") == 0)
        topic = "/link_attacher_node/attach";
    else
        topic = "/link_attacher_node/detach";

    // Create client
    client = n.serviceClient<gazebo_ros_link_attacher::Attach>(topic);

    // Call server
    if (client.call(service))
    {
        success = true;
        ROS_INFO("Attach success");
    }
    else
        ROS_ERROR_STREAM("Failed to call service '/link_attacher_node/attach'");

    return success;
} // virtualAttachDetach

bool PickPlaceServer::moveToPosePick(geometry_msgs::Pose &pose, int id, double offset)
{
	ROS_INFO("Moving to pose pick %d", id);

    // Variables
    geometry_msgs::PoseStamped targetPose;
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = false;
    tf2::Quaternion start;
    tf2::Quaternion rotation;
    tf2::Quaternion end;

    // Define reference frame of the pose
    targetPose.header.frame_id = "base_link";

    // Set orientation
    targetPose.pose.orientation.w = 1.0;
    targetPose.pose.orientation.z = 0.0;
    targetPose.pose.orientation.y = 0.0;
    targetPose.pose.orientation.x = 0.0;

    switch (id)
    {
    case 1: // Blue

        // Convert a PoseStamped to Quaternion
        tf2::convert(targetPose.pose.orientation, start);
        rotation.setRPY(M_PI_2, 0.0, 0.0); // Frontal pick
        end = (rotation * start);
        end = end.normalize();

        // Set orientation
        tf2::convert(end, targetPose.pose.orientation);

        // Set position
        targetPose.pose.position = pose.position;

        if (offset != 0.0)
        {
            targetPose.pose.position.x -= (GRIPPER_LENGTH + (HEXAGON_DIAMETER / 2.0) + offset);
            targetPose.pose.position.z -= (HEXAGON_HEIGHT / 3.0);
        }
        else
        {
            targetPose.pose.position.x -= (GRIPPER_LENGTH - (HEXAGON_DIAMETER / 2.0)); 
            targetPose.pose.position.z -= (HEXAGON_HEIGHT / 3.0); 
        } // if-else
        break;

    case 2: // Green

        // Convert a PoseStamped to Quaternion
        tf2::convert(targetPose.pose.orientation, start);
        rotation.setRPY(0.0, M_PI_2, -M_PI_2); // Top pick
        end = (rotation * start);
        end = end.normalize();

        // Set orientation
        tf2::convert(end, targetPose.pose.orientation);

        // Set position
        targetPose.pose.position = pose.position;

        if (offset != 0.0)
            targetPose.pose.position.z += (GRIPPER_LENGTH + (TRIANGLE_HEIGHT / 2.0) + offset);
        else
            targetPose.pose.position.z += GRIPPER_LENGTH;
        break;
    case 3: // Red

        // Convert a PoseStamped to Quaternion
        tf2::convert(targetPose.pose.orientation, start);
        rotation.setRPY(0.0, M_PI_2, -M_PI_2);
        end = (rotation * start);
        end = end.normalize();

        // Set orientation
        tf2::convert(end, targetPose.pose.orientation);

        // Set position
        targetPose.pose.position = pose.position;

        if (offset != 0.0)
            targetPose.pose.position.z += (GRIPPER_LENGTH + offset);
        else
            targetPose.pose.position.z += (GRIPPER_LENGTH - (CUBE_HEIGHT / 2.0));

        break;
    } // switch-case

    // Set Parameters interface
    moveGroupInterfaceArm.setPlannerId("SBLkConfigDefault");
    moveGroupInterfaceArm.setPoseReferenceFrame("base_link");
    moveGroupInterfaceArm.setPoseTarget(targetPose, "arm_tool_link");
    moveGroupInterfaceArm.setStartStateToCurrentState();
    moveGroupInterfaceArm.setMaxVelocityScalingFactor(1.0);
    moveGroupInterfaceArm.setPlanningTime(50);

    if (moveGroupInterfaceArm.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        success = true;
        moveGroupInterfaceArm.move();
    }

    return success;
} // moveToPosePick

bool PickPlaceServer::setGripper(std::string mode)
{
    // Declare variables
    std::map<std::string, double> targetPosition;
    std::vector<std::string> torsoGripperJointNames;
    moveit::planning_interface::MoveGroupInterface::Plan planGripper;
    bool success = false;

    // Set correct value of gripper depending on modality
    if (mode.compare("open") == 0)
    {
        ROS_INFO("Open gripper");
        targetPosition.insert({"gripper_left_finger_joint", OPEN_GRIPPER_MAX_SIZE});
        targetPosition.insert({"gripper_right_finger_joint", OPEN_GRIPPER_MAX_SIZE});
    }
    else if (mode.compare("close") == 0)
    {
        ROS_INFO("Close gripper");
        targetPosition.insert({"gripper_left_finger_joint", 0.0});
        targetPosition.insert({"gripper_right_finger_joint", 0.0});
    }
    else
    {
        ROS_ERROR("Set gripper modality properly!");
    }

    // Get joint names
    torsoGripperJointNames = moveGroupInterfaceGripper.getJoints();

    // Set interface params
    moveGroupInterfaceGripper.setPlannerId("SBLkConfigDefault");
    moveGroupInterfaceGripper.setStartStateToCurrentState();
    moveGroupInterfaceGripper.setMaxVelocityScalingFactor(1.0);
    moveGroupInterfaceGripper.setPlanningTime(50);

    for (int i = 1; i < torsoGripperJointNames.size(); i++)
        if (targetPosition.count(torsoGripperJointNames.at(i)) > 0)
            moveGroupInterfaceGripper.setJointValueTarget(
                torsoGripperJointNames.at(i), 
                targetPosition.at(torsoGripperJointNames.at(i))
            );

    if (moveGroupInterfaceGripper.plan(planGripper) == moveit::core::MoveItErrorCode::SUCCESS)
    {
		ROS_INFO("Moving gripper %d", mode);
		
        success = true;
        moveGroupInterfaceGripper.move();
    }

    return success;
} // setGripper

void PickPlaceServer::removeCollisionObject(std::string name)
{
    // Variables
    std::vector<std::string> collisionObjectsIds;

    // Remove from planning scene
    collisionObjectsIds.push_back(name);
    planningSceneInterface.removeCollisionObjects(collisionObjectsIds);

    // Remove from vector of collision objects
    int index = 0;
    for (int i = 0; i < collisionObjects.size(); i++)
    {
        if (collisionObjects.at(i).id.compare(name) == 0)
        {
            index = i;
            break;
        }
    }
    collisionObjects.erase(collisionObjects.begin() + index);

} // removeCollisionObject

void PickPlaceServer::publishPlanningScene()
{
    // Variables
    ros::NodeHandle node;
    ros::Publisher publisher;
    std::map<std::string, moveit_msgs::CollisionObject> map;
    ros::WallDuration sleep_t(0.5);
    moveit_msgs::PlanningScene planningSceneMessage;
    std::map<std::string, moveit_msgs::CollisionObject>::iterator it;

    // Get publisher
    publisher = node.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);

    // Wait that a client connects (rviz)
    while (publisher.getNumSubscribers() < 1)
        sleep_t.sleep();

    // Get the objects
    map = planningSceneInterface.getObjects();

    // Add collision objects
    for (it = map.begin(); it != map.end(); it++)
        planningSceneMessage.world.collision_objects.push_back(it->second);
    planningSceneMessage.is_diff = true;

    // Publish scene
    publisher.publish(planningSceneMessage);
} // publishPlanningScene

bool PickPlaceServer::moveToPosePlace(geometry_msgs::Pose &pose, int id, double offset)
{
	ROS_INFO("Moving to pose pick %d", id);

    // Variables
    geometry_msgs::PoseStamped targetPose;
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = false;
    tf2::Quaternion start;
    tf2::Quaternion rotation;
    tf2::Quaternion end;

    // Define reference frame of the pose
    targetPose.header.frame_id = "base_link";

    // Set orientation
    targetPose.pose.orientation.w = 1.0;
    targetPose.pose.orientation.z = 0.0;
    targetPose.pose.orientation.y = 0.0;
    targetPose.pose.orientation.x = 0.0;

    // Convert a PoseStamped to Quaternion
    tf2::convert(targetPose.pose.orientation, start);
    rotation.setRPY(0.0, M_PI_2, -M_PI_2); // Top place
    end = (rotation * start);
    end = end.normalize();
    // Set orientation
    tf2::convert(end, targetPose.pose.orientation);

    // Set position
    targetPose.pose.position = pose.position;
    switch (id)
    {
    case 1: // Blue
        targetPose.pose.position.y -= 0.05; 
        targetPose.pose.position.x += 0.1; 
        if (offset != 0.0)
            targetPose.pose.position.z += (GRIPPER_LENGTH + (HEXAGON_DIAMETER / 2.0) + offset);
        else
            targetPose.pose.position.z += (GRIPPER_LENGTH + (HEXAGON_DIAMETER / 2.0));
        break;

    case 2: // Green
        // targetPose.pose.position.x -= 0.2;
        if (offset != 0.0)
            targetPose.pose.position.z += (GRIPPER_LENGTH + (TRIANGLE_HEIGHT / 2.0) + offset);
        else
            targetPose.pose.position.z += GRIPPER_LENGTH + (TRIANGLE_HEIGHT / 2.0);
        break;
    case 3: // Red
        if (offset != 0.0)
            targetPose.pose.position.z += (GRIPPER_LENGTH + (CUBE_HEIGHT / 2.0) + offset);
        else
            targetPose.pose.position.z += (GRIPPER_LENGTH + (CUBE_HEIGHT / 2.0));
        break;
    } // switch-case
	
	ROS_INFO("Pose : %d", targetPose.pose);

    // Set Parameters interface
    moveGroupInterfaceArm.setPlannerId("SBLkConfigDefault");
    moveGroupInterfaceArm.setPoseReferenceFrame("base_link");
    moveGroupInterfaceArm.setPoseTarget(targetPose, "arm_tool_link");
    moveGroupInterfaceArm.setStartStateToCurrentState();
    moveGroupInterfaceArm.setMaxVelocityScalingFactor(1.0);
    moveGroupInterfaceArm.setPlanningTime(50.0);

    if (moveGroupInterfaceArm.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        success = true;
        moveGroupInterfaceArm.move();
    }

    return success;
}


void PickPlaceServer::callback(const node_b_msgs::PickPlaceGoalConstPtr &goal)
{
    // Variables
    bool success = true;
    geometry_msgs::Pose pickPose;
    geometry_msgs::Pose placePose;

    // Populate map of detections
    for (int i = 0; i < goal->poses.size(); i++)
        this->detectionsPosesIds.insert({goal->poses.at(i).id, goal->poses.at(i).pose});

    setupPlannigScene(goal->mode);

    if (goal->mode == 0)
    {
        // Get pickPose
        pickPose = detectionsPosesIds.at(goal->id_object_pick_place);

        // Move to default pose
        success &= moveToDefaultPose();

        if (success)
        {
            feedback.status = "Default Pose Reached";
            server.publishFeedback(feedback);
            
            // Open gripper
            success &= setGripper("open");

            if (success)
            {
                feedback.status = "Open Gripper";
                server.publishFeedback(feedback);
                
                // Approach the object
                success &= moveToPosePick(
                    pickPose, 
                    goal->id_object_pick_place, 
                    0.1
                );

                if (success)
                {
                    feedback.status = "Approach Position Reached";
                    server.publishFeedback(feedback);
                    
                    // Move to the object
                    success &= moveToPosePick(pickPose, goal->id_object_pick_place, 0.0);

                    if (success)
                    {
                        feedback.status = "Position Object Reached";
                        server.publishFeedback(feedback);
                        
                        // Attach virtually the object
                        switch (goal->id_object_pick_place)
                        {
                        case 1: // Blue hexagon
                            success &= virtualAttachDetach("Hexagon", 
                                                           "Hexagon_link",
                                                           "tiago", 
                                                           "arm_7_link",
                                                           "attach");
                            break;
                        case 2: // Green Triangle
                            success &= virtualAttachDetach("Triangle", 
                                                           "Triangle_link",
                                                           "tiago", 
                                                           "arm_7_link",
                                                           "attach");
                            break;
                        case 3: // Red Cube
                            success &= virtualAttachDetach("cube", 
                                                           "cube_link",
                                                           "tiago", 
                                                           "arm_7_link",
                                                           "attach");
                            break;
                        }

                        if (success)
                        {
                            feedback.status = "Object Attached";
                            server.publishFeedback(feedback);
                            
                            // Remove collision object to pick
                            switch (goal->id_object_pick_place)
                            {
                            case 1: // Blue hexagon
                                removeCollisionObject("hexagon");
                                break;
                            case 2: // Green Triangle
                                removeCollisionObject("triangle");
                                break;
                            case 3: // Red Cube
                                removeCollisionObject("cube");
                                break;
                            }

                            feedback.status = "Collision Object Removed";
                            server.publishFeedback(feedback);
                            
                            // Close gripper
                            success &= setGripper("close");

                            if (success)
                            {
                                feedback.status = "Close Gripper";
                                server.publishFeedback(feedback);
                                
                                // Move back to offset position
                                success &= moveToPosePick(
                                    pickPose, 
                                    goal->id_object_pick_place, 
                                    0.1
                                ); 

                                if (success)
                                {
                                    feedback.status = "Depart Position Reached";
                                    server.publishFeedback(feedback);
                                    
                                    // Move back to default Pose
                                    success &= moveToDefaultPose();
                                    if (success)
                                    {
                                        feedback.status = "Default Pose Reached";
                                        server.publishFeedback(feedback);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    else
    {
        // Get place Pose
        placePose = detectionsPosesIds.at(-1);

        // Correct the pose
        placePose.position.z += PLACE_TABLE_HEIGHT;

        // Move to approach position
        success &= moveToPosePlace(placePose, goal->id_object_pick_place,0.1);

        if (success)
        {
            feedback.status = "Approach Position Reached";
            server.publishFeedback(feedback);
            
            // Move to place position
            success &= moveToPosePlace(placePose, goal->id_object_pick_place, 0.0);
            if (success)
            {
                feedback.status = "Position Object Reached";
                server.publishFeedback(feedback);
                
                switch (goal->id_object_pick_place)
                        {
                        case 1: // Blue hexagon
                            success &= virtualAttachDetach("Hexagon", 
                                                           "Hexagon_link",
                                                           "tiago", 
                                                           "arm_7_link",
                                                           "detach");
                            break;
                        case 2: // Green Triangle
                            success &= virtualAttachDetach("Triangle", 
                                                           "Triangle_link",
                                                           "tiago", 
                                                           "arm_7_link",
                                                           "detach");
                            break;
                        case 3: // Red Cube
                            success &= virtualAttachDetach("cube", 
                                                           "cube_link",
                                                           "tiago", 
                                                           "arm_7_link",
                                                           "detach");
                            break;
                        }

                if (success)
                {
                    feedback.status = "Object Detached";
                    server.publishFeedback(feedback);
                    
                    // Open the gripper
                    success &= setGripper("open");

                    if (success)
                    {

                        feedback.status = "Open Gripper";
                        server.publishFeedback(feedback);
                        
                        // Move to depart position
                        success &= moveToPosePlace(
                            placePose, 
                            goal->id_object_pick_place, 
                            0.1
                        );

                        if (success)
                        {
                            feedback.status = "Depart Position Reached";
                            server.publishFeedback(feedback);
                            
                            // Move back to default Pose
                            success &= moveToDefaultPose();
                            if (success)
                            {
                                feedback.status = "Default Pose Reached";
                                server.publishFeedback(feedback);
                            }
                        }
                    }
                }
            }
        }
    }

    // Unsetup planning scene
    unSetupPlanningScene();

    // Remove current detection poses
    detectionsPosesIds.clear();

    // Set success
    this->result.state = success;
    if (success)
        server.setSucceeded(this->result);
    else
        server.setAborted(this->result);
}
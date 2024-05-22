#include <node_b/torso_lifter/torso_lifter_server.h>

TorsoLifterServer::TorsoLifterServer(ros::NodeHandle node) : 
    moveGroupInterfaceArm("arm_torso")
{
    this->node = node;
    this->server = this->node.advertiseService("/torso_lifter", &TorsoLifterServer::sendResponse, this);
}

bool TorsoLifterServer::sendResponse(node_b_msgs::TorsoLifter::Request &req, node_b_msgs::TorsoLifter::Response &res)
{
    moveit::planning_interface::MoveGroupInterface::Plan planTorso;
    moveGroupInterfaceArm.setPlannerId("SBLkConfigDefault");
    moveGroupInterfaceArm.setStartStateToCurrentState();
    moveGroupInterfaceArm.setMaxVelocityScalingFactor(1.0);
    std::vector<double> jointValues = moveGroupInterfaceArm.getCurrentJointValues();
    std::vector<std::string> jointNames = moveGroupInterfaceArm.getJoints();

    int index = std::distance(
        jointNames.begin(), 
        std::find(jointNames.begin(), 
            jointNames.end(), 
            "torso_lift_joint"
        )
    );

    bool success = false;
    if (index >= 0 && index < jointValues.size())
    {
        jointValues.at(index) = req.joint_value;
        moveGroupInterfaceArm.setJointValueTarget(jointValues);
        moveGroupInterfaceArm.setPlanningTime(50.0);

        if (moveGroupInterfaceArm.plan(planTorso) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            ROS_INFO("Lifting torso...");
            success = true;
            moveGroupInterfaceArm.move();
        }
    }
    res.state = success;
    return success;
}
#include <navigation/navigation_server.h>

NavigationServer::NavigationServer(std::string name) : server(nh, name, boost::bind(&NavigationServer::navAndDetectCallback, this, _1), false), client("move_base", true)
{
    server.start();
}

void NavigationServer::doNavigation(const navigation::NavigateGoalConstPtr &goal)
{
    // Wait for the action server to come up so that we can begin processing goals.
    client.waitForServer();

    // create the MoveBase message
    move_base_msgs::MoveBaseGoal goalMsg;

    // set the goal position
    goalMsg.target_pose.header.frame_id = "map";
    goalMsg.target_pose.header.stamp = ros::Time::now();

    goalMsg.target_pose.pose.position.x = goal->x;
    goalMsg.target_pose.pose.position.y = goal->y;
    goalMsg.target_pose.pose.orientation.z = goal->orZ;
    goalMsg.target_pose.pose.orientation.w = goal->orW;

    // Send the goal and wait
    actionlib::SimpleClientGoalState result = client.sendGoalAndWait(goalMsg);

    feedback.state = 1; 
    server.publishFeedback(feedback);

    // Finished
    this->result.arrived = true;
    server.setSucceeded(this->result);
}

void NavigationServer::navAndDetectCallback(const navigation::NavigateGoalConstPtr &goal)
{
    doNavigation(goal);
}

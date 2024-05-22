#include <ros/ros.h>
#include <node_b/torso_lifter/torso_lifter_server.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "torso_lifter");

    ros::NodeHandle node;
    ros::AsyncSpinner spinner(2);
    spinner.start();

    TorsoLifterServer server(node);
    ros::waitForShutdown();

    return 0;
}
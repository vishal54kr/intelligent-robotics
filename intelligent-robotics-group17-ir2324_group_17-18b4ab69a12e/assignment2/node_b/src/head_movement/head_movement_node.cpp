#include <ros/ros.h>
#include <node_b/head_movement/head_movement_server.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "head_movement");
    ros::NodeHandle node;

    HeadMovementServer server;

    ros::spin();

    return 0;
}
#include <ros/ros.h>
#include <node_c/obstacle_extractor/obstacle_extractor_server.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstacle_extractor_node");
    ros::NodeHandle node;
    ObstacleExtractorServer server(node);

    ros::spin();

    return 0;
}
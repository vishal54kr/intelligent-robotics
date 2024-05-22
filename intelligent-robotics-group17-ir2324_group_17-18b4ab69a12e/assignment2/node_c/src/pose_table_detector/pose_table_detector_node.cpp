
#include <ros/ros.h>
#include <node_c/pose_table_detector/pose_table_detector_server.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_table_detector");
    ros::NodeHandle node;
    PoseTableDetectorServer server(node);

    ros::spin();
    return 0;

} 
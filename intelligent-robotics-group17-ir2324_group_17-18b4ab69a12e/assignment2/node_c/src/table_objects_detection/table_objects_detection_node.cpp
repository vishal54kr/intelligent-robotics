#include <ros/ros.h>
#include <node_c/table_objects_detection/table_objects_detection_server.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "table_detection_node");
    ros::NodeHandle node;

    TableObjectsDetectionServer server; 

    ros::spin();
        
    return 0;
}
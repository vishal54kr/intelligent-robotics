
#include <ros/ros.h>
#include <node_c/poses_detection/poses_detection_publisher.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "poses_detection_node");

    ros::NodeHandle node;
    PosesDetectionPublisher publisher(node);

    ros::Rate loopRate(10);

    while (ros::ok())
    {
        publisher.publish();
        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}
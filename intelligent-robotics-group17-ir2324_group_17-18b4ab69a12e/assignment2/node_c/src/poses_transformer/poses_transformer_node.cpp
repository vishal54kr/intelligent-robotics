#include <ros/ros.h>
#include <node_c/poses_transformer/poses_transformer_server.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "poses_transformer_node");
    ros::NodeHandle node;
    PosesTransformerServer server(node);

    ros::spin();

    return 0;
}
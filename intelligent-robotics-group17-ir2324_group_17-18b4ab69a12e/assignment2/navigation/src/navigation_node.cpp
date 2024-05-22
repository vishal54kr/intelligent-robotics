#include <ros/ros.h>
#include <navigation/navigation_server.h>

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "navigation_server");
    NavigationServer server("/navigation_server");
    ros::spin();

    return 0;
}
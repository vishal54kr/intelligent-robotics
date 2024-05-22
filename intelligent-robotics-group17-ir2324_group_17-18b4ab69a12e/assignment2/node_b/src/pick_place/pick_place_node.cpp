#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <node_b/pick_place/pick_place_server.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pick_place");

    ros::NodeHandle node;
    ros::AsyncSpinner spinner(2);
    spinner.start();

    PickPlaceServer pickPlace(node);

    ros::waitForShutdown();
    spinner.stop();

    return 0;
} 
#include <ros/ros.h>
#include "relay_tower_lamp/relay_tower_lamp.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv,"relay_tower_lamp");
    ros::NodeHandle n;

    ROS_INFO("%s", ros::this_node::getName().c_str());

    Relay_tower_lamp relay_tower_lamp(n);
    relay_tower_lamp.runLoop();

    return 0;
}
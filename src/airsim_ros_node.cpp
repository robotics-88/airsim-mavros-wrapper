/* 
© 2022 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#include "airsim_ros_wrapper/airsim_ned_wrapper.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "airsim_node");
    ros::NodeHandle node;

    airsim_ros::AirsimNEDWrapper airsim_ros_wrapper(node);

    ros::spin();

    return 0;
}
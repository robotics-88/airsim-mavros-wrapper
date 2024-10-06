/* 
2022 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#include <rclcpp/rclcpp.hpp>
#include "airsim_mavros/airsim_mavros.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto airsim = std::make_shared < airsim_ros::AirsimMavros > (node_options);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(airsim);
    executor.spin();

    return 0;
}
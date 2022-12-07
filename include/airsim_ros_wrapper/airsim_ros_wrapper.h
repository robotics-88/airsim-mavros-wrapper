/* 
© 2022 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#ifndef AIRSIM_ROS_WRAPPER_H_
#define AIRSIM_ROS_WRAPPER_H_

#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <octomap_msgs/Octomap.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2_ros/transform_listener.h>


namespace airsim_ros {
/**
 * @class AirsimRosWrapper
 * @brief A class for processing mostly IR imagery (supplemented with depth pointclouds) to produce vegetation indices
 */
class AirsimRosWrapper {

    public:
        AirsimRosWrapper(ros::NodeHandle& node);
        ~AirsimRosWrapper();

    private:
        ros::NodeHandle private_nh_;
        ros::NodeHandle nh_;
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

};

}

#endif

/* 
© 2022 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#include "airsim_ros_wrapper/airsim_ros_wrapper.h"

namespace airsim_ros
{
AirsimRosWrapper::AirsimRosWrapper(ros::NodeHandle& node)
  : private_nh_("~")
  , nh_(node)
  , tf_listener_(tf_buffer_)
{
}

AirsimRosWrapper::~AirsimRosWrapper(){}

}
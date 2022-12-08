/* 
© 2022 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#ifndef AIRSIM_ROS_WRAPPER_H_
#define AIRSIM_ROS_WRAPPER_H_

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2_ros/transform_listener.h>

#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "common/AirSimSettings.hpp"
#include "rpc/rpc_error.h"
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

#include <airsim_ros_wrapper/math_common.h>

namespace airsim_ros {
/**
 * @class AirsimRosWrapper
 * @brief A class wrapper for AirSim with a MultiRotor and MAVROS control
 */
class AirsimRosWrapper {
    using AirSimSettings = msr::airlib::AirSimSettings;
    using VehicleSetting = msr::airlib::AirSimSettings::VehicleSetting;
    using CameraSetting = msr::airlib::AirSimSettings::CameraSetting;
    using CaptureSetting = msr::airlib::AirSimSettings::CaptureSetting;
    using ImageRequest = msr::airlib::ImageCaptureBase::ImageRequest;
    using ImageResponse = msr::airlib::ImageCaptureBase::ImageResponse;
    using ImageType = msr::airlib::ImageCaptureBase::ImageType;

    public:
        AirsimRosWrapper(ros::NodeHandle& node);
        ~AirsimRosWrapper();

        void initializeAirsim();
        void initializeRos();

        // ROS timer callbacks
        void imgResponseTimerCallback(const ros::TimerEvent& event); // update images from airsim_client_ every nth sec
        void droneStateTimerCallback(const ros::TimerEvent& event); // update drone state from airsim_client_ every nth sec
        void lidarTimerCallback(const ros::TimerEvent& event);

        ros::Time updateState();

    private:
        // utility struct for a SINGLE robot
        class VehicleROS
        {
        public:
            virtual ~VehicleROS() {}
            std::string vehicle_name;

            /// All things ROS
            ros::Publisher odom_local_pub;
            ros::Publisher global_gps_pub;
            // ros::Publisher env_pub;
            // airsim_ros_pkgs::Environment env_msg;
            // std::vector<SensorPublisher> sensor_pubs;
            // handle lidar seperately for max performance as data is collected on its own thread/callback
            // std::vector<SensorPublisher> lidar_pubs;

            nav_msgs::Odometry curr_odom;
            sensor_msgs::NavSatFix gps_sensor_msg;

            std::vector<geometry_msgs::TransformStamped> static_tf_msg_vec;

            ros::Time stamp;

            std::string odom_frame_id;
            /// Status
            // bool is_armed_;
            // std::string mode_;
        };

        class MultiRotorROS : public VehicleROS
        {
        public:
            /// State
            msr::airlib::MultirotorState curr_drone_state;
            // bool in_air_; // todo change to "status" and keep track of this

            ros::Subscriber vel_cmd_body_frame_sub;
            ros::Subscriber vel_cmd_world_frame_sub;

            ros::ServiceServer takeoff_srvr;
            ros::ServiceServer land_srvr;

            // bool has_vel_cmd;
            // VelCmd vel_cmd;

            /// Status
            // bool in_air_; // todo change to "status" and keep track of this
        };

        ros::NodeHandle private_nh_;
        ros::NodeHandle nh_;
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;
        std::string host_ip_;

        std::unique_ptr<msr::airlib::MultirotorRpcLibClient> airsim_client_ = nullptr;
        // separate busy connections to airsim, update in their own thread
        msr::airlib::MultirotorRpcLibClient airsim_client_images_;
        msr::airlib::MultirotorRpcLibClient airsim_client_lidar_;

        std::unordered_map<std::string, std::unique_ptr<VehicleROS>> vehicle_name_ptr_map_;
        static const std::unordered_map<int, std::string> image_type_int_to_string_map_;

        /// ROS Timers.
        ros::Timer airsim_img_response_timer_;
        ros::Timer airsim_control_update_timer_;
        ros::Timer airsim_lidar_update_timer_;

        typedef std::pair<std::vector<ImageRequest>, std::string> airsim_img_request_vehicle_name_pair;
        std::vector<airsim_img_request_vehicle_name_pair> airsim_img_request_vehicle_name_pair_vec_;
        std::vector<image_transport::Publisher> image_pub_vec_;
        std::vector<ros::Publisher> cam_info_pub_vec_;
        std::vector<sensor_msgs::CameraInfo> camera_info_msg_vec_;

        void createRosPubsFromSettingsJson();
        void setNansToZerosInPose(VehicleSetting& vehicle_setting) const;
        void setNansToZerosInPose(const VehicleSetting& vehicle_setting, CameraSetting& camera_setting) const;

        ros::Time airsimTimestampToRos(const msr::airlib::TTimePoint& stamp) const;
        ros::Time chronoTimestampToRos(const std::chrono::system_clock::time_point& stamp) const;
        sensor_msgs::NavSatFix getGpsSensorMsgFromAirsimGeopoint(const msr::airlib::GeoPoint& geo_point) const;
        sensor_msgs::CameraInfo generateCamInfo(const std::string& camera_name,
                                                const CameraSetting& camera_setting,
                                                const CaptureSetting& capture_setting) const;

};

}

#endif

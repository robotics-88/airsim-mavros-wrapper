/* 
2023 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#ifndef AIRSIM_NED_WRAPPER_H_
#define AIRSIM_NED_WRAPPER_H_

#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF //todo what does this do?
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
    STRICT_MODE_ON

#include "common/AirSimSettings.hpp"
#include "common/common_utils/FileSystem.hpp"
#include "sensors/lidar/LidarSimpleParams.hpp"
#include "sensors/imu/ImuBase.hpp"
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "vehicles/car/api/CarRpcLibClient.hpp"

#include <chrono>
#include <iostream>
#include <math.h>
#include <unordered_map>
#include <memory>

#include "ros/ros.h"

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/opencv.hpp>
#include <ros/callback_queue.h>
#include <ros/console.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Range.h>
#include <rosgraph_msgs/Clock.h>
#include <std_srvs/Empty.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>

#include <airsim_mavros/math_common.h>


namespace airsim_ros {
/**
 * @class AirsimMavros
 * @brief A class wrapper for AirSim with a MultiRotor and MAVROS control
 */

class AirsimMavros
{
    using AirSimSettings = msr::airlib::AirSimSettings;
    using SensorBase = msr::airlib::SensorBase;
    using CameraSetting = msr::airlib::AirSimSettings::CameraSetting;
    using CaptureSetting = msr::airlib::AirSimSettings::CaptureSetting;
    using LidarSetting = msr::airlib::AirSimSettings::LidarSetting;
    using VehicleSetting = msr::airlib::AirSimSettings::VehicleSetting;
    using ImageRequest = msr::airlib::ImageCaptureBase::ImageRequest;
    using ImageResponse = msr::airlib::ImageCaptureBase::ImageResponse;
    using ImageType = msr::airlib::ImageCaptureBase::ImageType;

public:
    enum class AIRSIM_MODE : unsigned
    {
        DRONE,
        CAR
    };

    AirsimMavros(const ros::NodeHandle& nh);
    ~AirsimMavros(){};

    void initialize_airsim();
    void initialize_ros();

    // std::vector<ros::CallbackQueue> callback_queues_;
    ros::AsyncSpinner img_async_spinner_;
    ros::AsyncSpinner lidar_async_spinner_;
    bool is_used_lidar_timer_cb_queue_;
    bool is_used_img_timer_cb_queue_;

private:
    struct SensorPublisher
    {
        SensorBase::SensorType sensor_type;
        std::string sensor_name;
        ros::Publisher publisher;
    };

    // utility struct for a SINGLE robot
    class VehicleROS
    {
    public:
        virtual ~VehicleROS() {}
        std::string vehicle_name;

        /// All things ROS
        ros::Publisher odom_local_pub;
        ros::Publisher global_gps_pub;
        ros::Publisher env_pub;
        ros::Publisher attitude_pub;
        std::vector<SensorPublisher> sensor_pubs;
        // handle lidar separately for max performance as data is collected on its own thread/callback
        std::vector<SensorPublisher> lidar_pubs;

        nav_msgs::Odometry curr_odom;
        geometry_msgs::QuaternionStamped curr_attitude;
        sensor_msgs::NavSatFix gps_sensor_msg;

        std::vector<geometry_msgs::TransformStamped> static_tf_msg_vec;

        ros::Time stamp;

        std::string odom_frame_id;
    };

    class CarROS : public VehicleROS
    {
    public:
        msr::airlib::CarApiBase::CarState curr_car_state;

        ros::Subscriber car_cmd_sub;
        ros::Publisher car_state_pub;

        bool has_car_cmd;
        msr::airlib::CarApiBase::CarControls car_cmd;
    };

    class MultiRotorROS : public VehicleROS
    {
    public:
        /// State
        msr::airlib::MultirotorState curr_drone_state;

        ros::Subscriber vel_cmd_body_frame_sub;
        ros::Subscriber vel_cmd_world_frame_sub;

        ros::ServiceServer takeoff_srvr;
        ros::ServiceServer land_srvr;
    };

    bool parseAirsimSettings();
    std::string getSimMode();

    /// ROS timer callbacks
    void img_response_timer_cb(const ros::TimerEvent& event); // update images from airsim_client_ every nth sec
    void drone_imu_timer_cb(const ros::TimerEvent& event);
    void drone_state_timer_cb(const ros::TimerEvent& event); // update drone state from airsim_client_ every nth sec
    void lidar_timer_cb(const ros::TimerEvent& event);

    // state, returns the simulation timestamp best guess based on drone state timestamp, airsim needs to return timestap for environment
    void update_and_publish_static_transforms(VehicleROS* vehicle_ros);
    void publish_vehicle_state();

    /// ROS tf broadcasters
    void publish_odom_tf(const nav_msgs::Odometry& odom_ned_msg);

    /// camera helper methods
    sensor_msgs::CameraInfo generate_cam_info(const std::string& camera_name, const CameraSetting& camera_setting, const CaptureSetting& capture_setting) const;
    cv::Mat manual_decode_depth(const ImageResponse& img_response) const;

    sensor_msgs::ImagePtr get_img_msg_from_response(const ImageResponse& img_response, const ros::Time curr_ros_time, const std::string frame_id);
    sensor_msgs::ImagePtr get_depth_img_msg_from_response(const ImageResponse& img_response, const ros::Time curr_ros_time, const std::string frame_id);

    void process_and_publish_img_response(const std::vector<ImageResponse>& img_response_vec, const int img_response_idx, const std::string& vehicle_name);

    // methods which parse setting json ang generate ros pubsubsrv
    void create_ros_pubs_from_settings_json();
    void append_static_camera_tf(VehicleROS* vehicle_ros, const std::string& camera_name, const CameraSetting& camera_setting);
    void append_static_lidar_tf(VehicleROS* vehicle_ros, const std::string& lidar_name, const msr::airlib::LidarSimpleParams& lidar_setting);
    void set_nans_to_zeros_in_pose(VehicleSetting& vehicle_setting) const;
    void set_nans_to_zeros_in_pose(const VehicleSetting& vehicle_setting, CameraSetting& camera_setting) const;

    /// utils. todo parse into an Airlib<->ROS conversion class
    nav_msgs::Odometry get_odom_msg_from_multirotor_state(const msr::airlib::MultirotorState& drone_state);
    geometry_msgs::QuaternionStamped get_attitude_from_airsim_state(const msr::airlib::MultirotorState& drone_state);
    sensor_msgs::NavSatFix get_gps_sensor_msg_from_airsim_geo_point(const msr::airlib::GeoPoint& geo_point) const;
    sensor_msgs::PointCloud2 get_lidar_msg_from_airsim(const msr::airlib::LidarData& lidar_data) const;
    sensor_msgs::MagneticField get_mag_msg_from_airsim(const msr::airlib::MagnetometerBase::Output& mag_data);
    sensor_msgs::Imu get_imu_msg_from_airsim(const msr::airlib::ImuBase::Output& imu_data);

    // simulation time utility
    ros::Time airsim_timestamp_to_ros(const msr::airlib::TTimePoint& stamp) const;
    ros::Time chrono_timestamp_to_ros(const std::chrono::system_clock::time_point& stamp) const;
    ros::Time make_ts(uint64_t unreal_ts);
    
    ros::Time first_imu_ros_ts;
    int64_t first_imu_unreal_ts = -1;

    // Utility methods to convert airsim_client_
    msr::airlib::MultirotorRpcLibClient* get_multirotor_client();
    msr::airlib::CarRpcLibClient* get_car_client();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    std::string host_ip_;

    std::string settings_text_;

    AIRSIM_MODE airsim_mode_ = AIRSIM_MODE::DRONE;

    std::unordered_map<std::string, std::unique_ptr<VehicleROS>> vehicle_name_ptr_map_;
    std::unique_ptr<VehicleROS> vehicle_ros_;
    static const std::unordered_map<int, std::string> image_type_int_to_string_map_;
    std::map<std::string, std::string> vehicle_imu_map_;
    std::map<std::string, std::string> vehicle_lidar_map_;

    bool is_vulkan_; // rosparam obtained from launch file. If vulkan is being used, we BGR encoding instead of RGB

    std::unique_ptr<msr::airlib::RpcLibClientBase> airsim_client_ = nullptr;
    // seperate busy connections to airsim, update in their own thread
    msr::airlib::RpcLibClientBase airsim_client_images_;
    msr::airlib::RpcLibClientBase airsim_client_lidar_;

    // todo not sure if async spinners shuold be inside this class, or should be instantiated in airsim_node.cpp, and cb queues should be public
    // todo for multiple drones with multiple sensors, this won't scale. make it a part of VehicleROS?
    ros::CallbackQueue img_timer_cb_queue_;
    ros::CallbackQueue lidar_timer_cb_queue_;

    std::recursive_mutex drone_control_mutex_;

    // gimbal control
    bool has_gimbal_cmd_;

    /// ROS tf
    std::string odom_frame_id_;
    std::string world_frame_id_;
    std::string map_frame_id_;
    std::string vehicle_frame_id_;
    const std::string AIRSIM_FRAME_ID = "world_ned";
    const std::string AIRSIM_ODOM_FRAME_ID = "odom_local_ned";
    const std::string ENU_ODOM_FRAME_ID = "odom_local_enu";
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    tf2_ros::StaticTransformBroadcaster static_tf_pub_;

    bool isENU_ = false;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    /// ROS params
    bool enable_cameras_;

    /// ROS Timers.
    ros::Timer airsim_img_response_timer_;
    ros::Timer airsim_control_update_timer_;
    ros::Timer airsim_imu_update_timer_;
    ros::Timer airsim_lidar_update_timer_;
    ros::ServiceClient mavros_client_;

    typedef std::pair<std::vector<ImageRequest>, std::string> airsim_img_request_vehicle_name_pair;
    std::vector<airsim_img_request_vehicle_name_pair> airsim_img_request_vehicle_name_pair_vec_;
    std::vector<ImageRequest> airsim_img_request_vec_;
    std::vector<image_transport::Publisher> image_pub_vec_;
    std::vector<ros::Publisher> cam_info_pub_vec_;
    std::vector<sensor_msgs::CameraInfo> camera_info_msg_vec_;
    std::vector<ros::Publisher> lidar_pub_vec_;
    std::vector<ros::Publisher> imu_pub_vec_;

    /// ROS other publishers
    ros::Publisher clock_pub_;
    rosgraph_msgs::Clock ros_clock_;
    bool publish_clock_ = false;

};

}

#endif

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

#include "rclcpp/rclcpp.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <image_transport/image_transport.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/stream_rate.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <std_srvs/srv/empty.hpp>
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

template <typename T>
struct SensorPublisher
{
    msr::airlib::SensorBase::SensorType sensor_type;
    std::string sensor_name;
    typename rclcpp::Publisher<T>::SharedPtr publisher;
};

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

    AirsimMavros(const std::shared_ptr<rclcpp::Node> nh, const std::shared_ptr<rclcpp::Node> nh_img, const std::shared_ptr<rclcpp::Node> nh_lidar, const std::string& host_ip);
    ~AirsimMavros(){};

    void initialize_airsim();
    void initialize_ros();

    // std::vector<ros::CallbackQueue> callback_queues_;
    // ros::AsyncSpinner img_async_spinner_;
    // ros::AsyncSpinner lidar_async_spinner_;
    bool is_used_lidar_timer_cb_queue_;
    bool is_used_img_timer_cb_queue_;

private:

    // utility struct for a SINGLE robot
    class VehicleROS
    {
    public:
        virtual ~VehicleROS() {}
        std::string vehicle_name;

        /// All things ROS
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_local_pub;
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr global_gps_pub;

        std::vector<SensorPublisher<sensor_msgs::msg::Imu>> imu_pubs;
        std::vector<SensorPublisher<sensor_msgs::msg::NavSatFix>> gps_pubs;
        std::vector<SensorPublisher<sensor_msgs::msg::MagneticField>> magnetometer_pubs;
        std::vector<SensorPublisher<sensor_msgs::msg::Range>> distance_pubs;
        std::vector<SensorPublisher<sensor_msgs::msg::PointCloud2>> lidar_pubs;

        // handle lidar seperately for max performance as data is collected on its own thread/callback

        nav_msgs::msg::Odometry curr_odom;
        geometry_msgs::msg::QuaternionStamped curr_attitude;
        sensor_msgs::msg::NavSatFix gps_sensor_msg;

        std::vector<geometry_msgs::msg::TransformStamped> static_tf_msg_vec;

        rclcpp::Time stamp;

        std::string odom_frame_id;
    };

    class CarROS : public VehicleROS
    {
    public:
        msr::airlib::CarApiBase::CarState curr_car_state_;

        // bool has_car_cmd_;
        // msr::airlib::CarApiBase::CarControls car_cmd_;
    };

    class MultiRotorROS : public VehicleROS
    {
    public:
        /// State
        msr::airlib::MultirotorState curr_drone_state;

        // bool has_vel_cmd_;
        // VelCmd vel_cmd_;
    };

    bool parseAirsimSettings();
    std::string getSimMode();

    /// ROS timer callbacks
    void img_response_timer_cb(); // update images from airsim_client_ every nth sec
    void drone_imu_timer_cb();
    void drone_state_timer_cb(); // update drone state from airsim_client_ every nth sec
    void lidar_timer_cb();

    // state, returns the simulation timestamp best guess based on drone state timestamp, airsim needs to return timestap for environment
    void update_and_publish_static_transforms(VehicleROS* vehicle_ros);
    void publish_vehicle_state();

    /// ROS tf broadcasters
    void publish_odom_tf(const nav_msgs::msg::Odometry& odom_ned_msg);

    /// camera helper methods
    sensor_msgs::msg::CameraInfo generate_cam_info(const std::string& camera_name, const CameraSetting& camera_setting, const CaptureSetting& capture_setting) const;
    cv::Mat manual_decode_depth(const ImageResponse& img_response) const;

    std::shared_ptr<sensor_msgs::msg::Image> get_img_msg_from_response(const ImageResponse& img_response, const rclcpp::Time curr_ros_time, const std::string frame_id);
    std::shared_ptr<sensor_msgs::msg::Image> get_depth_img_msg_from_response(const ImageResponse& img_response, const rclcpp::Time curr_ros_time, const std::string frame_id);

    void process_and_publish_img_response(const std::vector<ImageResponse>& img_response_vec, const int img_response_idx, const std::string& vehicle_name);

    // methods which parse setting json ang generate ros pubsubsrv
    void create_ros_pubs_from_settings_json();
    void append_static_camera_tf(VehicleROS* vehicle_ros, const std::string& camera_name, const CameraSetting& camera_setting);
    void append_static_lidar_tf(VehicleROS* vehicle_ros, const std::string& lidar_name, const msr::airlib::LidarSimpleParams& lidar_setting);
    void set_nans_to_zeros_in_pose(VehicleSetting& vehicle_setting) const;
    void set_nans_to_zeros_in_pose(const VehicleSetting& vehicle_setting, CameraSetting& camera_setting) const;

    /// utils. todo parse into an Airlib<->ROS conversion class
    nav_msgs::msg::Odometry get_odom_msg_from_multirotor_state(const msr::airlib::MultirotorState& drone_state);
    geometry_msgs::msg::QuaternionStamped get_attitude_from_airsim_state(const msr::airlib::MultirotorState& drone_state);
    sensor_msgs::msg::NavSatFix get_gps_sensor_msg_from_airsim_geo_point(const msr::airlib::GeoPoint& geo_point) const;
    sensor_msgs::msg::PointCloud2 get_lidar_msg_from_airsim(const msr::airlib::LidarData& lidar_data) const;
    sensor_msgs::msg::MagneticField get_mag_msg_from_airsim(const msr::airlib::MagnetometerBase::Output& mag_data);
    sensor_msgs::msg::Imu get_imu_msg_from_airsim(const msr::airlib::ImuBase::Output& imu_data);

    // simulation time utility
    // rclcpp::Time airsim_timestamp_to_ros(const msr::airlib::TTimePoint& stamp) const;
    // rclcpp::Time chrono_timestamp_to_ros(const std::chrono::system_clock::time_point& stamp) const;
    // rclcpp::Time make_ts(uint64_t unreal_ts);
    
    rclcpp::Time first_imu_ros_ts;
    int64_t first_imu_unreal_ts = -1;

    // Utility methods to convert airsim_client_
    msr::airlib::MultirotorRpcLibClient* get_multirotor_client();
    msr::airlib::CarRpcLibClient* get_car_client();

    template <typename T>
    const SensorPublisher<T> create_sensor_publisher(const std::string& sensor_type_name, const std::string& sensor_name,
                                                     SensorBase::SensorType sensor_type, const std::string& topic_name, int QoS);

private:
    std::shared_ptr<rclcpp::Node> nh_;
    std::shared_ptr<rclcpp::Node> nh_img_;
    std::shared_ptr<rclcpp::Node> nh_lidar_;

    std::string host_ip_;

    std::string settings_text_;

    AIRSIM_MODE airsim_mode_ = AIRSIM_MODE::DRONE;

    std::unordered_map<std::string, std::unique_ptr<VehicleROS>> vehicle_name_ptr_map_;
    std::unique_ptr<MultiRotorROS> vehicle_ros_;
    static const std::unordered_map<int, std::string> image_type_int_to_string_map_;
    // std::map<std::string, std::string> vehicle_imu_map_;
    // std::map<std::string, std::string> vehicle_lidar_map_;

    bool is_vulkan_ = false; // rosparam obtained from launch file. If vulkan is being used, we BGR encoding instead of RGB

    std::unique_ptr<msr::airlib::RpcLibClientBase> airsim_client_;
    // seperate busy connections to airsim, update in their own thread
    msr::airlib::RpcLibClientBase airsim_client_images_;
    msr::airlib::RpcLibClientBase airsim_client_lidar_;

    // todo not sure if async spinners shuold be inside this class, or should be instantiated in airsim_node.cpp, and cb queues should be public
    // todo for multiple drones with multiple sensors, this won't scale. make it a part of VehicleROS?
    // ros::CallbackQueue img_timer_cb_queue_;
    // ros::CallbackQueue lidar_timer_cb_queue_;

    std::mutex drone_control_mutex_;

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
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_pub_;

    bool isENU_ = false;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    /// ROS params
    bool enable_cameras_;

    /// ROS Timers.
    rclcpp::TimerBase::SharedPtr airsim_img_response_timer_;
    rclcpp::TimerBase::SharedPtr airsim_control_update_timer_;
    rclcpp::TimerBase::SharedPtr airsim_lidar_update_timer_;
    rclcpp::TimerBase::SharedPtr airsim_imu_update_timer_;
    rclcpp::Client<mavros_msgs::srv::StreamRate>::SharedPtr mavros_client_;

    /// Callback groups
    std::vector<rclcpp::CallbackGroup::SharedPtr> airsim_img_callback_groups_;
    rclcpp::CallbackGroup::SharedPtr airsim_control_callback_group_;
    std::vector<rclcpp::CallbackGroup::SharedPtr> airsim_lidar_callback_groups_;

    typedef std::pair<std::vector<ImageRequest>, std::string> airsim_img_request_vehicle_name_pair;
    std::vector<airsim_img_request_vehicle_name_pair> airsim_img_request_vehicle_name_pair_vec_;
    std::vector<image_transport::Publisher> image_pub_vec_;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> cam_info_pub_vec_;
    std::vector<sensor_msgs::msg::CameraInfo> camera_info_msg_vec_;

    // typedef std::pair<std::vector<ImageRequest>, std::string> airsim_img_request_vehicle_name_pair;
    // std::vector<airsim_img_request_vehicle_name_pair> airsim_img_request_vehicle_name_pair_vec_;
    std::vector<ImageRequest> airsim_img_request_vec_;
    // std::vector<image_transport::Publisher> image_pub_vec_;
    // std::vector<ros::Publisher> cam_info_pub_vec_;
    // std::vector<sensor_msgs::CameraInfo> camera_info_msg_vec_;
    // std::vector<ros::Publisher> lidar_pub_vec_;
    // std::vector<ros::Publisher> imu_pub_vec_;

    /// ROS other publishers
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
    rosgraph_msgs::msg::Clock ros_clock_;
    bool publish_clock_ = false;

};

}

#endif

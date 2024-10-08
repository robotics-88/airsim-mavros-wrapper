/* 
2023 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#include <airsim_mavros/airsim_mavros.h>
#include <common/AirSimSettings.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace airsim_ros
{

const std::unordered_map<int, std::string> AirsimMavros::image_type_int_to_string_map_ = {
    { 0, "Scene" },
    { 1, "DepthPlanar" },
    { 2, "DepthPerspective" },
    { 3, "DepthVis" },
    { 4, "DisparityNormalized" },
    { 5, "Segmentation" },
    { 6, "SurfaceNormals" },
    { 7, "Infrared" }
};

AirsimMavros::AirsimMavros(const rclcpp::NodeOptions &options)
    : Node("airsim_node", options)
    , is_used_lidar_timer_cb_queue_(false)
    , is_used_img_timer_cb_queue_(false)
    , nh_img_(this->create_sub_node("img"))
    , nh_lidar_(this->create_sub_node("nh_lidar"))
    , image_transport_(rclcpp::Node::make_shared("image_transport_node"))
    , host_ip_("127.0.0.1")
    , airsim_client_(nullptr)
    , airsim_client_images_(host_ip_)
    , airsim_client_lidar_(host_ip_)
    , has_gimbal_cmd_(false)
    , odom_frame_id_("odom")
    , world_frame_id_("world")
    , map_frame_id_("map")
    , vehicle_frame_id_("base_link_frd")
    , enable_cameras_(false)
{
    ros_clock_.clock = rclcpp::Time(0);

    if (AirSimSettings::singleton().simmode_name != AirSimSettings::kSimModeTypeCar) {
        airsim_mode_ = AIRSIM_MODE::DRONE;
    }
    else {
        // TODO remove car mode option
        airsim_mode_ = AIRSIM_MODE::CAR;
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    static_tf_pub_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    parseAirsimSettings();
    initialize_ros();

    RCLCPP_INFO(this->get_logger(), "AirsimMavros Initialized!\n");
}

bool AirsimMavros::parseAirsimSettings() {
    msr::airlib::RpcLibClientBase airsim_client(host_ip_);
    airsim_client.confirmConnection();

    settings_text_ = airsim_client.getSettingsString();

    bool text = !settings_text_.empty();

    if (text) {
        AirSimSettings::initializeSettings(settings_text_);

        AirSimSettings::singleton().load(std::bind(&AirsimMavros::getSimMode, this));
        RCLCPP_INFO(this->get_logger(), "SimMode: %s", AirSimSettings::singleton().simmode_name.c_str());

        return true;
    }
    return false;
}

std::string AirsimMavros::getSimMode() {
    const auto& settings_json = msr::airlib::Settings::loadJSonString(settings_text_);
    return settings_json.getString("SimMode", "");
}

void AirsimMavros::initialize_airsim()
{
    // todo do not reset if already in air?
    try {
        airsim_client_ = std::unique_ptr<msr::airlib::RpcLibClientBase>(new msr::airlib::MultirotorRpcLibClient(host_ip_));
        airsim_client_->confirmConnection();
        airsim_client_images_.confirmConnection();
        airsim_client_lidar_.confirmConnection();
    }
    catch (rpc::rpc_error& e) {
        std::string msg = e.get_error().as<std::string>();
        RCLCPP_ERROR(this->get_logger(), "Exception raised by the API, something went wrong. %s", msg.c_str());
    }
}

void AirsimMavros::initialize_ros()
{
    // ros params
    double update_airsim_control_every_n_sec;
    double imu_n_sec = .01;
    this->get_parameter("is_vulkan", is_vulkan_);
    this->get_parameter("update_airsim_control_every_n_sec", update_airsim_control_every_n_sec);
    this->get_parameter("publish_clock", publish_clock_);
    this->get_parameter_or("vehicle_frame_id", vehicle_frame_id_, vehicle_frame_id_);
    this->get_parameter_or("map_frame_id", map_frame_id_, map_frame_id_);
    this->get_parameter_or("world_frame_id", world_frame_id_, world_frame_id_);
    this->get_parameter_or("odom_frame_id", odom_frame_id_, odom_frame_id_);
    this->get_parameter("update_imu_n_sec", imu_n_sec);
    this->get_parameter("enable_cameras", enable_cameras_);

    this->declare_parameter("vehicle_name", rclcpp::ParameterValue(""));

    mavros_client_ = this->create_client<mavros_msgs::srv::StreamRate>("/mavros/set_stream_rate");

    create_ros_pubs_from_settings_json();
    airsim_control_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    airsim_control_update_timer_ = this->create_wall_timer(std::chrono::duration<double>(update_airsim_control_every_n_sec), std::bind(&AirsimMavros::drone_state_timer_cb, this), airsim_control_callback_group_);
    airsim_imu_update_timer_ = this->create_wall_timer(std::chrono::duration<double>(imu_n_sec), std::bind(&AirsimMavros::drone_imu_timer_cb, this), airsim_control_callback_group_);
}

void AirsimMavros::create_ros_pubs_from_settings_json()
{
    airsim_img_request_vehicle_name_pair_vec_.clear();
    image_pub_vec_.clear();
    cam_info_pub_vec_.clear();
    camera_info_msg_vec_.clear();
    vehicle_name_ptr_map_.clear();
    size_t lidar_cnt = 0;

    // iterate over std::map<std::string, std::unique_ptr<VehicleSetting>> vehicles;
    for (const auto& curr_vehicle_elem : AirSimSettings::singleton().vehicles) {
        auto& vehicle_setting = curr_vehicle_elem.second;
        auto curr_vehicle_name = curr_vehicle_elem.first;

        this->set_parameter(rclcpp::Parameter("vehicle_name", curr_vehicle_name));

        set_nans_to_zeros_in_pose(*vehicle_setting);

        vehicle_ros_ = std::unique_ptr<MultiRotorROS>(new MultiRotorROS());

        vehicle_ros_->odom_frame_id = curr_vehicle_name + "/" + odom_frame_id_;
        vehicle_ros_->vehicle_name = curr_vehicle_name;

        vehicle_ros_->odom_local_pub = this->create_publisher<nav_msgs::msg::Odometry>(curr_vehicle_name + "/" + odom_frame_id_, 10);

        vehicle_ros_->global_gps_pub = this->create_publisher<sensor_msgs::msg::NavSatFix>(curr_vehicle_name + "/global_gps", 10);

        // iterate over camera map std::map<std::string, CameraSetting> .cameras;
        for (auto& curr_camera_elem : vehicle_setting->cameras) {
            auto& camera_setting = curr_camera_elem.second;
            auto& curr_camera_name = curr_camera_elem.first;

            set_nans_to_zeros_in_pose(*vehicle_setting, camera_setting);
            append_static_camera_tf(vehicle_ros_.get(), curr_camera_name, camera_setting);

            // iterate over capture_setting std::map<int, CaptureSetting> capture_settings
            for (const auto& curr_capture_elem : camera_setting.capture_settings) {
                const auto& capture_setting = curr_capture_elem.second;

                if (!(std::isnan(capture_setting.fov_degrees))) {
                    const ImageType curr_image_type = msr::airlib::Utils::toEnum<ImageType>(capture_setting.image_type);
                    // if scene / segmentation / surface normals / infrared, get uncompressed image with pixels_as_floats = false
                    if (curr_image_type == ImageType::Scene ||
                        curr_image_type == ImageType::Segmentation ||
                        curr_image_type == ImageType::SurfaceNormals ||
                        curr_image_type == ImageType::Infrared) {
                        airsim_img_request_vec_.push_back(ImageRequest(curr_camera_name, curr_image_type, false, false));
                    }
                    // if {DepthPlanar, DepthPerspective,DepthVis, DisparityNormalized}, get float image
                    else {
                        airsim_img_request_vec_.push_back(ImageRequest(curr_camera_name, curr_image_type, true));
                    }

                    const std::string cam_image_topic = curr_vehicle_name + "/" + curr_camera_name + "/" +
                                                        image_type_int_to_string_map_.at(capture_setting.image_type);

                    image_pub_vec_.push_back(image_transport_.advertise(cam_image_topic, 1));
                    cam_info_pub_vec_.push_back(this->create_publisher<sensor_msgs::msg::CameraInfo>(cam_image_topic + "/camera_info", 10));
                    camera_info_msg_vec_.push_back(generate_cam_info(curr_camera_name, camera_setting, capture_setting));
                }
            }
            // push back pair (vector of image captures, current vehicle name)
            airsim_img_request_vehicle_name_pair_vec_.push_back(std::make_pair(airsim_img_request_vec_, curr_vehicle_name));
        }

        // iterate over sensors
        for (auto& sensor_publisher : vehicle_setting->sensors) {
            auto& sensor_name = sensor_publisher.first;
            auto& sensor_setting = sensor_publisher.second;
            if (sensor_setting->enabled) {
                switch (sensor_setting->sensor_type) {
                    // TODO decide whether to keep the cases not currently used
                    case SensorBase::SensorType::Imu: {
                        SensorPublisher<sensor_msgs::msg::Imu> sensor_publisher =
                            create_sensor_publisher<sensor_msgs::msg::Imu>("Imu", sensor_setting->sensor_name, sensor_setting->sensor_type, curr_vehicle_name + "/imu/" + sensor_name, 10);
                        vehicle_ros_->imu_pubs.emplace_back(sensor_publisher);
                        break;
                    }
                    case SensorBase::SensorType::Gps: {
                        SensorPublisher<sensor_msgs::msg::NavSatFix> sensor_publisher =
                            create_sensor_publisher<sensor_msgs::msg::NavSatFix>("Gps", sensor_setting->sensor_name, sensor_setting->sensor_type, curr_vehicle_name + "/gps/" + sensor_name, 10);
                        vehicle_ros_->gps_pubs.emplace_back(sensor_publisher);
                        break;
                    }
                    case SensorBase::SensorType::Magnetometer: {
                        SensorPublisher<sensor_msgs::msg::MagneticField> sensor_publisher =
                            create_sensor_publisher<sensor_msgs::msg::MagneticField>("Magnetometer", sensor_setting->sensor_name, sensor_setting->sensor_type, curr_vehicle_name + "/magnetometer/" + sensor_name, 10);
                        vehicle_ros_->magnetometer_pubs.emplace_back(sensor_publisher);
                        break;
                    }
                    case SensorBase::SensorType::Distance: {
                        SensorPublisher<sensor_msgs::msg::Range> sensor_publisher =
                            create_sensor_publisher<sensor_msgs::msg::Range>("Distance", sensor_setting->sensor_name, sensor_setting->sensor_type, curr_vehicle_name + "/distance/" + sensor_name, 10);
                        vehicle_ros_->distance_pubs.emplace_back(sensor_publisher);
                        break;
                    }
                    case SensorBase::SensorType::Lidar: {
                        auto lidar_setting = *static_cast<LidarSetting*>(sensor_setting.get());
                        msr::airlib::LidarSimpleParams params;
                        params.initializeFromSettings(lidar_setting);
                        append_static_lidar_tf(vehicle_ros_.get(), sensor_name, params);

                        SensorPublisher<sensor_msgs::msg::PointCloud2> sensor_publisher =
                            create_sensor_publisher<sensor_msgs::msg::PointCloud2>("Lidar", sensor_setting->sensor_name, sensor_setting->sensor_type, curr_vehicle_name + "/lidar/" + sensor_name, 10);
                        vehicle_ros_->lidar_pubs.emplace_back(sensor_publisher);
                        lidar_cnt += 1;
                        break;
                    }
                    default: {
                        // TODO handle this?
                    }
                }
            }
        }
        
        vehicle_name_ptr_map_.emplace(curr_vehicle_name, std::move(vehicle_ros_)); // allows fast lookup in command callbacks in case of a lot of drones
    }

    if (publish_clock_) {
        clock_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("~/clock", 1);
    }

    // if >0 cameras, add one more thread for img_request_timer_cb
    if (!airsim_img_request_vehicle_name_pair_vec_.empty() && enable_cameras_) {
        double update_airsim_img_response_every_n_sec;
        this->get_parameter("update_airsim_img_response_every_n_sec", update_airsim_img_response_every_n_sec);
        auto cb = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        airsim_img_callback_groups_.push_back(cb);
        airsim_img_response_timer_ = nh_img_->create_wall_timer(std::chrono::duration<double>(update_airsim_img_response_every_n_sec), std::bind(&AirsimMavros::img_response_timer_cb, this), cb);
        is_used_img_timer_cb_queue_ = true;
    }

    // lidars update on their own callback/thread at a given rate
    if (lidar_cnt > 0) {
        double update_lidar_every_n_sec;
        this->get_parameter("update_lidar_every_n_sec", update_lidar_every_n_sec);
        auto cb = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        airsim_lidar_callback_groups_.push_back(cb);
        airsim_lidar_update_timer_ = nh_lidar_->create_wall_timer(std::chrono::duration<double>(update_lidar_every_n_sec), std::bind(&AirsimMavros::lidar_timer_cb, this), cb);
        is_used_lidar_timer_cb_queue_ = true;
    }

    initialize_airsim();
}

// QoS - The depth of the publisher message queue.
// more details here - https://docs.ros.org/en/foxy/Concepts/About-Quality-of-Service-Settings.html
template <typename T>
const SensorPublisher<T> AirsimMavros::create_sensor_publisher(const std::string& sensor_type_name, const std::string& sensor_name,
                                                                   SensorBase::SensorType sensor_type, const std::string& topic_name, int QoS)
{
    RCLCPP_INFO_STREAM(this->get_logger(), sensor_type_name);
    SensorPublisher<T> sensor_publisher;
    sensor_publisher.sensor_name = sensor_name;
    sensor_publisher.sensor_type = sensor_type;
    sensor_publisher.publisher = this->create_publisher<T>("~/" + topic_name, QoS);
    return sensor_publisher;
}

nav_msgs::msg::Odometry AirsimMavros::get_odom_msg_from_multirotor_state(const msr::airlib::MultirotorState& drone_state)
{

    // FLU 
    nav_msgs::msg::Odometry odom_flu_msg;
    odom_flu_msg.pose.pose.position.x = drone_state.getPosition().x();
    odom_flu_msg.pose.pose.position.y = -drone_state.getPosition().y();
    odom_flu_msg.pose.pose.position.z = -drone_state.getPosition().z();

    tf2::Quaternion quat(drone_state.getOrientation().w(), drone_state.getOrientation().x(), drone_state.getOrientation().y(), drone_state.getOrientation().z());
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    pitch = pitch + M_PI;
    yaw = yaw + M_PI;
    quat.setRPY(roll, pitch, yaw);
    quat.normalize();

    odom_flu_msg.pose.pose.orientation.x = quat.x();
    odom_flu_msg.pose.pose.orientation.y = quat.y();
    odom_flu_msg.pose.pose.orientation.z = quat.z();
    odom_flu_msg.pose.pose.orientation.w = quat.w();

    odom_flu_msg.twist.twist.linear.x = drone_state.kinematics_estimated.twist.linear.x();
    odom_flu_msg.twist.twist.linear.y = -drone_state.kinematics_estimated.twist.linear.y();
    odom_flu_msg.twist.twist.linear.z = -drone_state.kinematics_estimated.twist.linear.z();
    odom_flu_msg.twist.twist.angular.x = drone_state.kinematics_estimated.twist.angular.x();
    odom_flu_msg.twist.twist.angular.y = -drone_state.kinematics_estimated.twist.angular.y();
    odom_flu_msg.twist.twist.angular.z = -drone_state.kinematics_estimated.twist.angular.z();
    return odom_flu_msg;
}

geometry_msgs::msg::QuaternionStamped AirsimMavros::get_attitude_from_airsim_state(const msr::airlib::MultirotorState& drone_state)
{
    geometry_msgs::msg::QuaternionStamped attitude;
    attitude.quaternion.x = drone_state.getOrientation().x();
    attitude.quaternion.y = drone_state.getOrientation().y();
    attitude.quaternion.z = drone_state.getOrientation().z();
    attitude.quaternion.w = drone_state.getOrientation().w();

    return attitude;
}

// https://docs.ros.org/jade/api/sensor_msgs/html/point__cloud__conversion_8h_source.html#l00066
// look at UnrealLidarSensor.cpp UnrealLidarSensor::getPointCloud() for math
// read this carefully https://docs.ros.org/kinetic/api/sensor_msgs/html/msg/PointCloud2.html
sensor_msgs::msg::PointCloud2 AirsimMavros::get_lidar_msg_from_airsim(const msr::airlib::LidarData& lidar_data) const
{
    sensor_msgs::msg::PointCloud2 lidar_msg;

    if (lidar_data.point_cloud.size() > 3)
    {
        lidar_msg.height = 1;
        lidar_msg.width = lidar_data.point_cloud.size() / 3;

        lidar_msg.fields.resize(4);
        lidar_msg.fields[0].name = "x"; 
        lidar_msg.fields[1].name = "y"; 
        lidar_msg.fields[2].name = "z"; 
        lidar_msg.fields[3].name = "intensity";
        int offset = 0;

        for (size_t d = 0; d < lidar_msg.fields.size(); ++d, offset += 4)
        {
            lidar_msg.fields[d].offset = offset;
            lidar_msg.fields[d].datatype = sensor_msgs::msg::PointField::FLOAT32;
            lidar_msg.fields[d].count  = 1;
        }

        lidar_msg.is_bigendian = false;
        lidar_msg.point_step = offset; // 4 * num fields
        lidar_msg.row_step = lidar_msg.point_step * lidar_msg.width;
        lidar_msg.is_dense = true; // todo
        std::vector<float> data_std_pre = lidar_data.point_cloud;
        std::vector<float> data_std;// = lidar_data.point_cloud;
        for (int ii = 0; ii < data_std_pre.size(); ii++) {
            data_std.push_back(data_std_pre.at(ii));
            if (ii % 3 == 2) {
                float intensity = 0.0; // TODO Could put useful intensity at some point, but for now just a placeholder
                data_std.push_back(intensity);
            }
        }

        const unsigned char* bytes = reinterpret_cast<const unsigned char*>(&data_std[0]);
        std::vector<unsigned char> lidar_msg_data(bytes, bytes + sizeof(float) * data_std.size());
        lidar_msg.data = std::move(lidar_msg_data);
    }
    else
    {
        // msg = []
    }
    return lidar_msg;
}

sensor_msgs::msg::NavSatFix AirsimMavros::get_gps_sensor_msg_from_airsim_geo_point(const msr::airlib::GeoPoint& geo_point) const
{
    sensor_msgs::msg::NavSatFix gps_msg;
    gps_msg.latitude = geo_point.latitude;
    gps_msg.longitude = geo_point.longitude;
    gps_msg.altitude = geo_point.altitude;
    return gps_msg;
}

msr::airlib::MultirotorRpcLibClient* AirsimMavros::get_multirotor_client()
{
    return static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get());
}

sensor_msgs::msg::Imu AirsimMavros::get_imu_msg_from_airsim(const msr::airlib::ImuBase::Output& imu_data)
{
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.frame_id = vehicle_frame_id_;// todo multiple drones
    imu_msg.orientation.x = imu_data.orientation.x();
    imu_msg.orientation.y = imu_data.orientation.y();
    imu_msg.orientation.z = imu_data.orientation.z();
    imu_msg.orientation.w = imu_data.orientation.w();

    // todo radians per second
    imu_msg.angular_velocity.x = (imu_data.angular_velocity.x());
    imu_msg.angular_velocity.y = -(imu_data.angular_velocity.y());
    imu_msg.angular_velocity.z = -(imu_data.angular_velocity.z());

    // meters/s2^m 
    imu_msg.linear_acceleration.x = imu_data.linear_acceleration.x();
    imu_msg.linear_acceleration.y = -imu_data.linear_acceleration.y();
    imu_msg.linear_acceleration.z = -imu_data.linear_acceleration.z();
    return imu_msg;
}

void AirsimMavros::drone_imu_timer_cb()
{
    // TODO: leave as is for now, IMU timer cb does nothing. Waiting to see what IMU requirements based on a better SLAM algorithm. Use /mavros/imu/data instead for IMU. TBD if remove this method and timer.

    // Uncomment below if require the airsim_ros_node Imu topic, but experience suggests it is very inaccurate and MAVROS IMU should be preferred

    // try
    // {
    //     if (imu_pub_vec_.size() > 0)
    //     {
    //         int ctr = 0;
    //         for (const auto& vehicle_imu_pair: vehicle_imu_map_)
    //         {
    //             std::unique_lock<std::mutex> lck(drone_control_mutex_);
    //             rclcpp::Time curr_ros_time = this->now();
    //             auto imu_data = get_multirotor_client()->getImuData(vehicle_imu_pair.second, vehicle_imu_pair.first);
    //             lck.unlock();
    //             sensor_msgs::msg::Imu imu_msg = get_imu_msg_from_airsim(imu_data);
    //             imu_msg.header.stamp = curr_ros_time;
    //             // imu_msg.header.frame_id = vehicle_imu_pair.first;
    //             imu_pub_vec_[ctr].publish(imu_msg);
    //             ctr++;
    //         } 
    //     }
    // }

    // catch (rpc::rpc_error& e)
    // {
    //     std::cout << "error" << std::endl;
    //     std::string msg = e.get_error().as<std::string>();
    //     std::cout << "Exception raised by the API:" << std::endl << msg << std::endl;
    // }
}


void AirsimMavros::drone_state_timer_cb()
{
    try
    {
        for (auto& vehicle_name_ptr_pair : vehicle_name_ptr_map_) {
            rclcpp::Time vehicle_time;
            // get drone state from airsim
            auto& vehicle_ros = vehicle_name_ptr_pair.second;
            auto multirotor_ros = static_cast<MultiRotorROS*>(vehicle_ros.get());
            // get drone state from airsim
            rclcpp::Time curr_ros_time = this->now();
            auto rpc = static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get());
            multirotor_ros->curr_drone_state = rpc->getMultirotorState(multirotor_ros->vehicle_name);

            // convert airsim drone state to ROS msgs
            multirotor_ros->curr_odom = get_odom_msg_from_multirotor_state(multirotor_ros->curr_drone_state);
            multirotor_ros->curr_odom.child_frame_id = multirotor_ros->odom_frame_id;
            multirotor_ros->curr_odom.header.stamp = curr_ros_time;

            multirotor_ros->curr_attitude = get_attitude_from_airsim_state(multirotor_ros->curr_drone_state);
            multirotor_ros->curr_attitude.header.stamp = curr_ros_time;

            multirotor_ros->gps_sensor_msg = get_gps_sensor_msg_from_airsim_geo_point(multirotor_ros->curr_drone_state.gps_location);
            multirotor_ros->gps_sensor_msg.header.stamp = curr_ros_time;

            // publish to ROS!  
            multirotor_ros->odom_local_pub->publish(multirotor_ros->curr_odom);
            publish_odom_tf(multirotor_ros->curr_odom);
            multirotor_ros->global_gps_pub->publish(multirotor_ros->gps_sensor_msg);

            update_and_publish_static_transforms(multirotor_ros);

            has_gimbal_cmd_ = false;
        }
    }

    catch (rpc::rpc_error& e)
    {
        RCLCPP_WARN(this->get_logger(), "error");
        std::string msg = e.get_error().as<std::string>();
        RCLCPP_WARN(this->get_logger(), "Exception raised by the API: %s", msg.c_str());
    }
}

void AirsimMavros::update_and_publish_static_transforms(VehicleROS* vehicle_ros)
{
    // TODO updating static makes no sense other than to check for newly added static tf, which would also be very uncommon midrun. make part of instantiation but not regular loop.
    if (vehicle_ros && !vehicle_ros->static_tf_msg_vec.empty()) {
        for (auto& static_tf_msg : vehicle_ros->static_tf_msg_vec) {
            static_tf_pub_->sendTransform(static_tf_msg);
        }
    }
}

void AirsimMavros::publish_odom_tf(const nav_msgs::msg::Odometry& odom_ned_msg)
{
    geometry_msgs::msg::TransformStamped odom_tf;
    odom_tf.header.frame_id = odom_frame_id_;
    odom_tf.header.stamp = odom_ned_msg.header.stamp;// this->now();
    odom_tf.child_frame_id = vehicle_frame_id_ + "/" + odom_frame_id_; 
    odom_tf.transform.translation.x = odom_ned_msg.pose.pose.position.x;
    odom_tf.transform.translation.y = odom_ned_msg.pose.pose.position.y;
    odom_tf.transform.translation.z = odom_ned_msg.pose.pose.position.z;
    odom_tf.transform.rotation.x = odom_ned_msg.pose.pose.orientation.x;
    odom_tf.transform.rotation.y = odom_ned_msg.pose.pose.orientation.y;
    odom_tf.transform.rotation.z = odom_ned_msg.pose.pose.orientation.z;
    odom_tf.transform.rotation.w = odom_ned_msg.pose.pose.orientation.w;
    tf_broadcaster_->sendTransform(odom_tf);
}

// airsim uses nans for zeros in settings.json. we set them to zeros here for handling tfs in ROS
void AirsimMavros::set_nans_to_zeros_in_pose(VehicleSetting& vehicle_setting) const
{
    if (std::isnan(vehicle_setting.position.x()))
        vehicle_setting.position.x() = 0.0;

    if (std::isnan(vehicle_setting.position.y()))
        vehicle_setting.position.y() = 0.0;

    if (std::isnan(vehicle_setting.position.z()))
        vehicle_setting.position.z() = 0.0;

    if (std::isnan(vehicle_setting.rotation.yaw))
        vehicle_setting.rotation.yaw = 0.0;

    if (std::isnan(vehicle_setting.rotation.pitch))
        vehicle_setting.rotation.pitch = 0.0;

    if (std::isnan(vehicle_setting.rotation.roll))
        vehicle_setting.rotation.roll = 0.0;
}

// if any nan's in camera pose, set them to match vehicle pose (which has already converted any potential nans to zeros)
void AirsimMavros::set_nans_to_zeros_in_pose(const VehicleSetting& vehicle_setting, CameraSetting& camera_setting) const
{
    if (std::isnan(camera_setting.position.x()))
        camera_setting.position.x() = vehicle_setting.position.x();

    if (std::isnan(camera_setting.position.y()))
        camera_setting.position.y() = vehicle_setting.position.y();

    if (std::isnan(camera_setting.position.z()))
        camera_setting.position.z() = vehicle_setting.position.z();

    if (std::isnan(camera_setting.rotation.yaw))
        camera_setting.rotation.yaw = vehicle_setting.rotation.yaw;

    if (std::isnan(camera_setting.rotation.pitch))
        camera_setting.rotation.pitch = vehicle_setting.rotation.pitch;

    if (std::isnan(camera_setting.rotation.roll))
        camera_setting.rotation.roll = vehicle_setting.rotation.roll;
}

void AirsimMavros::append_static_lidar_tf(VehicleROS* vehicle_ros, const std::string& lidar_name, const msr::airlib::LidarSimpleParams& lidar_setting)
{
    geometry_msgs::msg::TransformStamped lidar_tf_msg;
    lidar_tf_msg.header.frame_id = vehicle_frame_id_;
    lidar_tf_msg.child_frame_id = vehicle_frame_id_ + "/" + lidar_name;
    lidar_tf_msg.transform.translation.x = lidar_setting.relative_pose.position.x();
    lidar_tf_msg.transform.translation.y = - lidar_setting.relative_pose.position.y();
    lidar_tf_msg.transform.translation.z = - lidar_setting.relative_pose.position.z();

    // Add PI to pitch, yaw, because AirSim settings process it as attached to a NED frame but should be FLU
    tf2::Quaternion quat(lidar_setting.relative_pose.orientation.x(), lidar_setting.relative_pose.orientation.y(), lidar_setting.relative_pose.orientation.z(), lidar_setting.relative_pose.orientation.w());
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    pitch = pitch + M_PI;
    yaw = yaw + M_PI;
    quat.setRPY(roll, pitch, yaw);
    quat.normalize();

    lidar_tf_msg.transform.rotation.x = quat.getX();
    lidar_tf_msg.transform.rotation.y = quat.getY();
    lidar_tf_msg.transform.rotation.z = quat.getZ();
    lidar_tf_msg.transform.rotation.w = quat.getW();

    if (isENU_) {
        std::swap(lidar_tf_msg.transform.translation.x, lidar_tf_msg.transform.translation.y);
        std::swap(lidar_tf_msg.transform.rotation.x, lidar_tf_msg.transform.rotation.y);
        lidar_tf_msg.transform.translation.z = -lidar_tf_msg.transform.translation.z;
        lidar_tf_msg.transform.rotation.z = -lidar_tf_msg.transform.rotation.z;
    }

    vehicle_ros->static_tf_msg_vec.emplace_back(lidar_tf_msg);
}

void AirsimMavros::append_static_camera_tf(VehicleROS* vehicle_ros, const std::string& camera_name, const CameraSetting& camera_setting)
{
    geometry_msgs::msg::TransformStamped static_cam_tf_body_msg;
    static_cam_tf_body_msg.header.frame_id = vehicle_frame_id_;
    static_cam_tf_body_msg.child_frame_id = camera_name + "_body";
    static_cam_tf_body_msg.transform.translation.x = camera_setting.position.x();
    static_cam_tf_body_msg.transform.translation.y = camera_setting.position.y();
    static_cam_tf_body_msg.transform.translation.z = camera_setting.position.z();
    tf2::Quaternion quat;
    quat.setRPY(camera_setting.rotation.roll, camera_setting.rotation.pitch, camera_setting.rotation.yaw);
    static_cam_tf_body_msg.transform.rotation.x = quat.x();
    static_cam_tf_body_msg.transform.rotation.y = quat.y();
    static_cam_tf_body_msg.transform.rotation.z = quat.z();
    static_cam_tf_body_msg.transform.rotation.w = quat.w();

    if (isENU_) {
        std::swap(static_cam_tf_body_msg.transform.translation.x, static_cam_tf_body_msg.transform.translation.y);
        std::swap(static_cam_tf_body_msg.transform.rotation.x, static_cam_tf_body_msg.transform.rotation.y);
        static_cam_tf_body_msg.transform.translation.z = -static_cam_tf_body_msg.transform.translation.z;
        static_cam_tf_body_msg.transform.rotation.z = -static_cam_tf_body_msg.transform.rotation.z;
    }

    geometry_msgs::msg::TransformStamped static_cam_tf_optical_msg;
    static_cam_tf_optical_msg.header.frame_id = camera_name + "_body";
    static_cam_tf_optical_msg.child_frame_id = camera_name + "_optical";
    static_cam_tf_optical_msg.transform.translation.x = 0;
    static_cam_tf_optical_msg.transform.translation.y = 0;
    static_cam_tf_optical_msg.transform.translation.z = 0;
    tf2::Quaternion quat2;
    quat2.setRPY(-M_PI_2, 0, -M_PI_2);
    static_cam_tf_optical_msg.transform.rotation.x = quat2.x();
    static_cam_tf_optical_msg.transform.rotation.y = quat2.y();
    static_cam_tf_optical_msg.transform.rotation.z = quat2.z();
    static_cam_tf_optical_msg.transform.rotation.w = quat2.w();

    vehicle_ros->static_tf_msg_vec.emplace_back(static_cam_tf_body_msg);
    vehicle_ros->static_tf_msg_vec.emplace_back(static_cam_tf_optical_msg);
}

void AirsimMavros::img_response_timer_cb()
{
    try {
        int image_response_idx = 0;
        const std::vector<ImageResponse>& img_response = airsim_client_images_.simGetImages(airsim_img_request_vec_);
        process_and_publish_img_response(img_response, image_response_idx, vehicle_ros_->vehicle_name);
    }

    catch (rpc::rpc_error& e) {
        std::string msg = e.get_error().as<std::string>();
        RCLCPP_WARN(this->get_logger(), "Exception raised by the API, didn't get image response.", msg.c_str());
    }
}

void AirsimMavros::lidar_timer_cb()
{
    try {
        for (auto& vehicle_name_ptr_pair : vehicle_name_ptr_map_) {
            if (!vehicle_name_ptr_pair.second->lidar_pubs.empty()) {
                for (auto& lidar_publisher : vehicle_name_ptr_pair.second->lidar_pubs) {
                    rclcpp::Time curr_ros_time = this->now();
                    auto lidar_data = airsim_client_lidar_.getLidarData(lidar_publisher.sensor_name, vehicle_name_ptr_pair.first);
                    sensor_msgs::msg::PointCloud2 lidar_msg = get_lidar_msg_from_airsim(lidar_data);
                    lidar_msg.header.frame_id = vehicle_frame_id_ + "/" + lidar_publisher.sensor_name;
                    lidar_msg.header.stamp = curr_ros_time;
                    lidar_publisher.publisher->publish(lidar_msg);
                }
            }
        }
    }

    catch (rpc::rpc_error& e)
    {
        std::string msg = e.get_error().as<std::string>();
        RCLCPP_WARN(this->get_logger(), "Exception raised by the API, didn't get image response.", msg.c_str());
    }
}

cv::Mat AirsimMavros::manual_decode_depth(const ImageResponse& img_response) const
{
    cv::Mat mat(img_response.height, img_response.width, CV_32FC1, cv::Scalar(0));
    int img_width = img_response.width;

    for (int row = 0; row < img_response.height; row++)
        for (int col = 0; col < img_width; col++)
            mat.at<float>(row, col) = img_response.image_data_float[row * img_width + col];
    return mat;
}

std::shared_ptr<sensor_msgs::msg::Image> AirsimMavros::get_img_msg_from_response(const ImageResponse& img_response,
                                                                  const rclcpp::Time curr_ros_time,
                                                                  const std::string frame_id)
{
    std::shared_ptr<sensor_msgs::msg::Image> img_msg_ptr = std::make_shared<sensor_msgs::msg::Image>();
    img_msg_ptr->data = img_response.image_data_uint8;
    img_msg_ptr->step = img_response.width * 3; // todo un-hardcode. image_width*num_bytes? or img_msg_ptr->step = img_response.image_data_uint8.size() / img_response.height;
    img_msg_ptr->header.stamp = rclcpp::Time(img_response.time_stamp);
    img_msg_ptr->header.frame_id = frame_id;
    img_msg_ptr->height = img_response.height;
    img_msg_ptr->width = img_response.width;
    img_msg_ptr->encoding = "bgr8";
    if (is_vulkan_)
        img_msg_ptr->encoding = "rgb8";
    img_msg_ptr->is_bigendian = 0;
    return img_msg_ptr;
}

std::shared_ptr<sensor_msgs::msg::Image> AirsimMavros::get_depth_img_msg_from_response(const ImageResponse& img_response,
                                                                        const rclcpp::Time curr_ros_time,
                                                                        const std::string frame_id)
{
    // todo using img_response.image_data_float direclty as done get_img_msg_from_response() throws an error,
    // hence the dependency on opencv and cv_bridge. however, this is an extremely fast op, so no big deal.
    auto depth_img_msg = std::make_shared<sensor_msgs::msg::Image>();
    depth_img_msg->width = img_response.width;
    depth_img_msg->height = img_response.height;
    depth_img_msg->data.resize(img_response.image_data_float.size() * sizeof(float));
    memcpy(depth_img_msg->data.data(), img_response.image_data_float.data(), depth_img_msg->data.size());
    depth_img_msg->encoding = "32FC1";
    depth_img_msg->step = depth_img_msg->data.size() / img_response.height;
    depth_img_msg->is_bigendian = 0;
    depth_img_msg->header.stamp = rclcpp::Time(img_response.time_stamp);
    depth_img_msg->header.frame_id = frame_id;
    return depth_img_msg;
}

// todo have a special stereo pair mode and get projection matrix by calculating offset wrt drone body frame?
sensor_msgs::msg::CameraInfo AirsimMavros::generate_cam_info(const std::string& camera_name,
                                                            const CameraSetting& camera_setting,
                                                            const CaptureSetting& capture_setting) const
{
    sensor_msgs::msg::CameraInfo cam_info_msg;
    cam_info_msg.header.frame_id = camera_name + "_optical";
    cam_info_msg.height = capture_setting.height;
    cam_info_msg.width = capture_setting.width;
    float f_x = (capture_setting.width / 2.0) / tan(math_common::deg2rad(capture_setting.fov_degrees / 2.0));
    // todo focal length in Y direction should be same as X it seems. this can change in future a scene capture component which exactly correponds to a cine camera
    // float f_y = (capture_setting.height / 2.0) / tan(math_common::deg2rad(fov_degrees / 2.0));
    cam_info_msg.k = { f_x, 0.0, capture_setting.width / 2.0, 0.0, f_x, capture_setting.height / 2.0, 0.0, 0.0, 1.0 };
    cam_info_msg.p = { f_x, 0.0, capture_setting.width / 2.0, 0.0, 0.0, f_x, capture_setting.height / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0 };
    return cam_info_msg;
}

void AirsimMavros::process_and_publish_img_response(const std::vector<ImageResponse>& img_response_vec, const int img_response_idx, const std::string& vehicle_name)
{
    unused(vehicle_name);
    int img_response_idx_internal = img_response_idx;
    rclcpp::Time curr_ros_time = this->now();

    for (const auto& curr_img_response : img_response_vec) {
        // update timestamp of saved cam info msgs

        camera_info_msg_vec_[img_response_idx_internal].header.stamp = curr_ros_time;
        cam_info_pub_vec_[img_response_idx_internal]->publish(camera_info_msg_vec_[img_response_idx_internal]);

        // DepthPlanar / DepthPerspective / DepthVis / DisparityNormalized
        if (curr_img_response.pixels_as_float) {
            image_pub_vec_[img_response_idx_internal].publish(get_depth_img_msg_from_response(curr_img_response,
                                                                                              curr_ros_time,
                                                                                              curr_img_response.camera_name + "_optical"));
        }
        // Scene / Segmentation / SurfaceNormals / Infrared
        else {
            image_pub_vec_[img_response_idx_internal].publish(get_img_msg_from_response(curr_img_response,
                                                                                        curr_ros_time,
                                                                                        curr_img_response.camera_name + "_optical"));
        }
        img_response_idx_internal++;
    }
}

}
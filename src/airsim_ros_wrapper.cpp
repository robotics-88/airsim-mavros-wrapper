/* 
© 2022 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#include "airsim_ros_wrapper/airsim_ros_wrapper.h"

#include <math.h>

namespace airsim_ros
{
    const std::unordered_map<int, std::string> AirsimRosWrapper::image_type_int_to_string_map_ = {
        { 0, "Scene" },
        { 1, "DepthPlanar" },
        { 2, "DepthPerspective" },
        { 3, "DepthVis" },
        { 4, "DisparityNormalized" },
        { 5, "Segmentation" },
        { 6, "SurfaceNormals" },
        { 7, "Infrared" }
    };

AirsimRosWrapper::AirsimRosWrapper(ros::NodeHandle& node)
  : is_used_lidar_timer_cb_queue_(false)
  , is_used_img_timer_cb_queue_(false)
  , private_nh_("~")
  , nh_(node)
  , tf_listener_(tf_buffer_)
  , host_ip_("127.0.0.1")
  , odom_frame_id_("odom")
  , world_frame_id_("world")
  , map_frame_id_("map")
  , vehicle_frame_id_("base_link")
  , airsim_client_images_(host_ip_)
  , airsim_client_lidar_(host_ip_)
{
    ros_clock_.clock.fromSec(0);

    parseAirsimSettings();
    initializeRos();

    std::cout << "AirsimROSWrapper Initialized!\n";
}

AirsimRosWrapper::~AirsimRosWrapper(){}

bool AirsimRosWrapper::parseAirsimSettings() {
    msr::airlib::RpcLibClientBase airsim_client(host_ip_);
    airsim_client.confirmConnection();

    settings_text_ = airsim_client.getSettingsString();

    bool text = !settings_text_.empty();

    if (text) {
        AirSimSettings::initializeSettings(settings_text_);

        AirSimSettings::singleton().load(std::bind(&AirsimRosWrapper::getSimMode, this));
        std::cout << "SimMode: " << AirSimSettings::singleton().simmode_name << std::endl;

        return true;
    }
    return false;
}

std::string AirsimRosWrapper::getSimMode() {
    const auto& settings_json = msr::airlib::Settings::loadJSonString(settings_text_);
    return settings_json.getString("SimMode", "");
}

void AirsimRosWrapper::initializeAirsim()
{
    // todo do not reset if already in air?
    try {

        airsim_client_ = std::unique_ptr<msr::airlib::MultirotorRpcLibClient>(new msr::airlib::MultirotorRpcLibClient(host_ip_));
        airsim_client_->confirmConnection();
        airsim_client_images_.confirmConnection();
        airsim_client_lidar_.confirmConnection();

        for (const auto& vehicle_name_ptr_pair : vehicle_name_ptr_map_) {
            airsim_client_->enableApiControl(true, vehicle_name_ptr_pair.first); // todo expose as rosservice?
            airsim_client_->armDisarm(true, vehicle_name_ptr_pair.first); // todo exposes as rosservice?
        }

        // origin_geo_point_ = airsim_client_->getHomeGeoPoint("");
        // // todo there's only one global origin geopoint for environment. but airsim API accept a parameter vehicle_name? inside carsimpawnapi.cpp, there's a geopoint being assigned in the constructor. by?
        // origin_geo_point_msg_ = get_gps_msg_from_airsim_geo_point(origin_geo_point_);
    }
    catch (rpc::rpc_error& e) {
        std::string msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API, something went wrong." << std::endl
                  << msg << std::endl;
    }
}

void AirsimRosWrapper::initializeRos()
{

    // ros params
    double update_airsim_control_every_n_sec;
    private_nh_.getParam("update_airsim_control_every_n_sec", update_airsim_control_every_n_sec);
    // private_nh_.getParam("publish_clock", publish_clock_);
    private_nh_.param("vehicle_frame_id", vehicle_frame_id_, vehicle_frame_id_);
    private_nh_.param("map_frame_id", map_frame_id_, map_frame_id_);
    private_nh_.param("world_frame_id", world_frame_id_, world_frame_id_);
    private_nh_.param("odom_frame_id", odom_frame_id_, odom_frame_id_);

    // ros subs
    odom_subscriber_ = nh_.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, &AirsimRosWrapper::odomCallback, this);
    gps_subscriber_ = nh_.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 10, &AirsimRosWrapper::gpsCallback, this);

    createRosPubsFromSettingsJson();
    airsim_control_update_timer_ = nh_.createTimer(ros::Duration(update_airsim_control_every_n_sec), &AirsimRosWrapper::droneStateTimerCallback, this);
}

void AirsimRosWrapper::createRosPubsFromSettingsJson() {
    airsim_img_request_vehicle_name_pair_vec_.clear();
    image_pub_vec_.clear();
    cam_info_pub_vec_.clear();
    camera_info_msg_vec_.clear();
    vehicle_name_ptr_map_.clear();
    size_t lidar_cnt = 0;

    image_transport::ImageTransport image_transporter(private_nh_);

    // iterate over std::map<std::string, std::unique_ptr<VehicleSetting>> vehicles;
    const auto& curr_vehicle_elem = AirSimSettings::singleton().vehicles.begin(); // Note this means it only works for a single vehicle
    // for (const auto& curr_vehicle_elem : AirSimSettings::singleton().vehicles) {
    auto& vehicle_setting = curr_vehicle_elem->second;
    auto curr_vehicle_name = curr_vehicle_elem->first;

    nh_.setParam("/vehicle_name", curr_vehicle_name);

    setNansToZerosInPose(*vehicle_setting);

    vehicle_ros_ = std::unique_ptr<VehicleROS>(new VehicleROS());

    vehicle_ros_->odom_frame_id = odom_frame_id_;
    vehicle_ros_->vehicle_name = curr_vehicle_name;

    appendStaticMap2WorldTf(*vehicle_setting);

    vehicle_ros_->odom_local_pub = private_nh_.advertise<nav_msgs::Odometry>(curr_vehicle_name + "/" + odom_frame_id_, 10);

    // vehicle_ros->env_pub = private_nh_.advertise<airsim_ros_pkgs::Environment>(curr_vehicle_name + "/environment", 10);

    // vehicle_ros->global_gps_pub = private_nh_.advertise<sensor_msgs::NavSatFix>(curr_vehicle_name + "/global_gps", 10); // Not needed- mavros handles

        // vehicle_ros.reset_srvr = private_nh_.advertiseService(curr_vehicle_name + "/reset",&AirsimROSWrapper::reset_srv_cb, this);

    // iterate over camera map std::map<std::string, CameraSetting> .cameras;
    for (auto& curr_camera_elem : vehicle_setting->cameras) {
        auto& camera_setting = curr_camera_elem.second;
        auto& curr_camera_name = curr_camera_elem.first;

        setNansToZerosInPose(*vehicle_setting, camera_setting);
        appendStaticCameraTf(vehicle_ros_.get(), curr_camera_name, camera_setting);
        // camera_setting.gimbal
        std::vector<ImageRequest> current_image_request_vec;

        // iterate over capture_setting std::map<int, CaptureSetting> capture_settings
        for (const auto& curr_capture_elem : camera_setting.capture_settings) {
            const auto& capture_setting = curr_capture_elem.second;

            // todo why does AirSimSettings::loadCaptureSettings calls AirSimSettings::initializeCaptureSettings()
            // which initializes default capture settings for _all_ NINE msr::airlib::ImageCaptureBase::ImageType
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

                image_pub_vec_.push_back(image_transporter.advertise(cam_image_topic, 1));
                cam_info_pub_vec_.push_back(private_nh_.advertise<sensor_msgs::CameraInfo>(cam_image_topic + "/camera_info", 10));
                camera_info_msg_vec_.push_back(generateCamInfo(curr_camera_name, camera_setting, capture_setting));
            }
        }
        // push back pair (vector of image captures, current vehicle name)
        airsim_img_request_vehicle_name_pair_vec_.push_back({ current_image_request_vec, curr_vehicle_name });
    }

    // iterate over sensors
    std::vector<SensorPublisher> sensors;
    for (const auto& [sensor_name, sensor_setting] : vehicle_setting->sensors) {
        if (sensor_setting->enabled) {
            SensorPublisher sensor_publisher;
            sensor_publisher.sensor_name = sensor_name;
            sensor_publisher.sensor_type = sensor_setting->sensor_type;
            switch (sensor_setting->sensor_type) {
            // case SensorBase::SensorType::Barometer: {
            //     ROS_INFO_STREAM(sensor_name << ": Barometer");
            //     sensor_publisher.publisher = private_nh_.advertise<airsim_ros_pkgs::Altimeter>(curr_vehicle_name + "/altimeter/" + sensor_name, 10);
            //     break;
            // }
            case SensorBase::SensorType::Imu: {
                ROS_INFO_STREAM(sensor_name << ": IMU");
                sensor_publisher.publisher = private_nh_.advertise<sensor_msgs::Imu>(curr_vehicle_name + "/imu/" + sensor_name, 10);
                break;
            }
            case SensorBase::SensorType::Gps: {
                ROS_INFO_STREAM(sensor_name << ": GPS");
                sensor_publisher.publisher = private_nh_.advertise<sensor_msgs::NavSatFix>(curr_vehicle_name + "/gps/" + sensor_name, 10);
                break;
            }
            case SensorBase::SensorType::Magnetometer: {
                ROS_INFO_STREAM(sensor_name << ": Magnetometer");
                sensor_publisher.publisher = private_nh_.advertise<sensor_msgs::MagneticField>(curr_vehicle_name + "/magnetometer/" + sensor_name, 10);
                break;
            }
            case SensorBase::SensorType::Distance: {
                ROS_INFO_STREAM(sensor_name << ": Distance sensor");
                sensor_publisher.publisher = private_nh_.advertise<sensor_msgs::Range>(curr_vehicle_name + "/distance/" + sensor_name, 10);
                break;
            }
            case SensorBase::SensorType::Lidar: {
                ROS_INFO_STREAM(sensor_name << ": Lidar");
                auto lidar_setting = *static_cast<LidarSetting*>(sensor_setting.get());
                msr::airlib::LidarSimpleParams params;
                params.initializeFromSettings(lidar_setting);
                appendStaticLidarTf(vehicle_ros_.get(), sensor_name, params);
                sensor_publisher.publisher = private_nh_.advertise<sensor_msgs::PointCloud2>(curr_vehicle_name + "/lidar/" + sensor_name, 10);
                break;
            }
            default: {
                // throw std::invalid_argument("Unexpected sensor type");
            }
            }
            sensors.emplace_back(sensor_publisher);
        }
    }

    // // we want fast access to the lidar sensors for callback handling, sort them out now
    auto isLidar = [](const SensorPublisher& pub) {
        return pub.sensor_type == SensorBase::SensorType::Lidar;
    };
    size_t cnt = std::count_if(sensors.begin(), sensors.end(), isLidar);
    lidar_cnt += cnt;
    vehicle_ros_->lidar_pubs.resize(cnt);
    vehicle_ros_->sensor_pubs.resize(sensors.size() - cnt);
    std::partition_copy(sensors.begin(), sensors.end(), vehicle_ros_->lidar_pubs.begin(), vehicle_ros_->sensor_pubs.begin(), isLidar);

    vehicle_name_ptr_map_.emplace(curr_vehicle_name, std::move(vehicle_ros_)); // allows fast lookup in command callbacks in case of a lot of drones
    // publishStaticTransforms(vehicle_ros.get()); // TODO I really think this should go here, or somewhere that it doesnt get in a loop, they're not updated. But here doesn't work. Figure out later.
    // }

    if (publish_clock_) {
        clock_pub_ = nh_.advertise<rosgraph_msgs::Clock>("/clock", 1);
    }

    // if >0 cameras, add one more thread for img_request_timer_cb
    if (!airsim_img_request_vehicle_name_pair_vec_.empty()) {
        double update_airsim_img_response_every_n_sec;
        private_nh_.getParam("update_airsim_img_response_every_n_sec", update_airsim_img_response_every_n_sec);
        ROS_INFO("starting img timer with %f", update_airsim_img_response_every_n_sec);
        airsim_img_response_timer_ = nh_.createTimer(ros::Duration(update_airsim_img_response_every_n_sec), &AirsimRosWrapper::imgResponseTimerCallback, this);
    }

    // lidars update on their own callback/thread at a given rate
    if (lidar_cnt > 0) {
        double update_lidar_every_n_sec;
        private_nh_.getParam("update_lidar_every_n_sec", update_lidar_every_n_sec);
        // private_nh_.setCallbackQueue(&lidar_timer_cb_queue_);
        airsim_lidar_update_timer_ = nh_.createTimer(ros::Duration(update_lidar_every_n_sec), &AirsimRosWrapper::lidarTimerCallback, this);
    }

    initializeAirsim();
}

// airsim uses nans for zeros in settings.json. we set them to zeros here for handling tfs in ROS
void AirsimRosWrapper::setNansToZerosInPose(VehicleSetting& vehicle_setting) const
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
void AirsimRosWrapper::setNansToZerosInPose(const VehicleSetting& vehicle_setting, CameraSetting& camera_setting) const
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

void AirsimRosWrapper::appendStaticMap2WorldTf(const VehicleSetting& vehicle_setting)
{
    // geometry_msgs::TransformStamped vehicle_tf_msg;
    // vehicle_tf_msg.header.frame_id = world_frame_id_;
    // vehicle_tf_msg.header.stamp = ros::Time::now();
    // vehicle_tf_msg.child_frame_id = map_frame_id_;
    // vehicle_tf_msg.transform.translation.x = vehicle_setting.position.x();
    // vehicle_tf_msg.transform.translation.y = vehicle_setting.position.y();
    // vehicle_tf_msg.transform.translation.z = vehicle_setting.position.z();
    // tf2::Quaternion quat;
    // quat.setRPY(vehicle_setting.rotation.roll, vehicle_setting.rotation.pitch, vehicle_setting.rotation.yaw);
    // vehicle_tf_msg.transform.rotation.x = quat.x();
    // vehicle_tf_msg.transform.rotation.y = quat.y();
    // vehicle_tf_msg.transform.rotation.z = quat.z();
    // vehicle_tf_msg.transform.rotation.w = quat.w();

    // // vehicle_ros->static_tf_msg_vec.emplace_back(vehicle_tf_msg);
    // ROS_WARN("would have sent static map to world tf with stamp %d", vehicle_tf_msg.header.stamp.toSec());

    // tf_broadcaster_.sendTransform(vehicle_tf_msg);
}

void AirsimRosWrapper::appendStaticCameraTf(VehicleROS* vehicle_ros, const std::string& camera_name, const CameraSetting& camera_setting)
{
    geometry_msgs::TransformStamped static_cam_tf_body_msg;
    static_cam_tf_body_msg.header.frame_id = vehicle_frame_id_;
    static_cam_tf_body_msg.child_frame_id = camera_name + "_body";
    static_cam_tf_body_msg.transform.translation.x = camera_setting.position.x();
    static_cam_tf_body_msg.transform.translation.y = -1 * camera_setting.position.y();
    static_cam_tf_body_msg.transform.translation.z = -1 * camera_setting.position.z();
    tf2::Quaternion quat;
    quat.setRPY(camera_setting.rotation.roll, camera_setting.rotation.pitch, camera_setting.rotation.yaw);
    static_cam_tf_body_msg.transform.rotation.x = quat.x();
    static_cam_tf_body_msg.transform.rotation.y = quat.y();
    static_cam_tf_body_msg.transform.rotation.z = quat.z();
    static_cam_tf_body_msg.transform.rotation.w = quat.w();

    // Optical frame is defined w.r.t. camera body: z forward, x,y are image convention (x right, y down)
    geometry_msgs::TransformStamped static_cam_tf_optical_msg;// = static_cam_tf_body_msg;
    static_cam_tf_optical_msg.header.frame_id = static_cam_tf_body_msg.child_frame_id;
    static_cam_tf_optical_msg.child_frame_id = camera_name + "_optical";
    // Same origin
    static_cam_tf_optical_msg.transform.translation.x = 0.0;
    static_cam_tf_optical_msg.transform.translation.y = 0.0;
    static_cam_tf_optical_msg.transform.translation.z = 0.0;

    // Visualize this way: First rotate by -Pi/2 around z-axis, then by same around x-axis
    tf2::Quaternion rotate_xaxis, rotate_zaxis;
    double rotate_angle = -0.5 * M_PI;
    rotate_zaxis.setRPY(0.0, 0.0, rotate_angle);
    rotate_xaxis.setRPY(rotate_angle, 0.0, 0.0);
    tf2::Quaternion quat_cam_optical = rotate_zaxis * rotate_xaxis;
    quat_cam_optical.normalize();
    tf2::convert(quat_cam_optical, static_cam_tf_optical_msg.transform.rotation);

    vehicle_ros->static_tf_msg_vec.emplace_back(static_cam_tf_body_msg);
    vehicle_ros->static_tf_msg_vec.emplace_back(static_cam_tf_optical_msg);
}

void AirsimRosWrapper::appendStaticLidarTf(VehicleROS* vehicle_ros, const std::string& lidar_name, const msr::airlib::LidarSimpleParams& lidar_setting)
{
    // Something is extremely questionable, dont know if its me or AirSim. When mounted standard (in AS json), needs attach to base link frd. When mounted vertically, needs attach to base link.
    geometry_msgs::TransformStamped lidar_tf_msg;
    lidar_tf_msg.header.frame_id = "base_link_frd"; // This is such a regrettable hacky hack bc it should be base_link, but for w/e reason, consistently upside down, whether rotate at 0, 180, -180, etc, all upside down. This fixed it.
    lidar_tf_msg.child_frame_id = vehicle_ros->vehicle_name + "/" + lidar_name;
    lidar_tf_msg.transform.translation.x = lidar_setting.relative_pose.position.x();
    lidar_tf_msg.transform.translation.y = lidar_setting.relative_pose.position.y();
    lidar_tf_msg.transform.translation.z = lidar_setting.relative_pose.position.z();
    lidar_tf_msg.transform.rotation.x = -1 * lidar_setting.relative_pose.orientation.x();
    lidar_tf_msg.transform.rotation.y = lidar_setting.relative_pose.orientation.y();
    lidar_tf_msg.transform.rotation.z = lidar_setting.relative_pose.orientation.z();
    lidar_tf_msg.transform.rotation.w = lidar_setting.relative_pose.orientation.w();

    vehicle_ros->static_tf_msg_vec.emplace_back(lidar_tf_msg);
}

// todo have a special stereo pair mode and get projection matrix by calculating offset wrt drone body frame?
sensor_msgs::CameraInfo AirsimRosWrapper::generateCamInfo(const std::string& camera_name,
                                                            const CameraSetting& camera_setting,
                                                            const CaptureSetting& capture_setting) const
{
    sensor_msgs::CameraInfo cam_info_msg;
    cam_info_msg.header.frame_id = camera_name + "_optical";
    cam_info_msg.height = capture_setting.height;
    cam_info_msg.width = capture_setting.width;
    cam_info_msg.distortion_model = "plumb_bob";
    float f_x = (capture_setting.width / 2.0) / tan(math_common::deg2rad(capture_setting.fov_degrees / 2.0));
    // todo focal length in Y direction should be same as X it seems. this can change in future a scene capture component which exactly correponds to a cine camera
    // float f_y = (capture_setting.height / 2.0) / tan(math_common::deg2rad(fov_degrees / 2.0));
    cam_info_msg.K = { f_x, 0.0, capture_setting.width / 2.0, 0.0, f_x, capture_setting.height / 2.0, 0.0, 0.0, 1.0 };
    cam_info_msg.P = { f_x, 0.0, capture_setting.width / 2.0, 0.0, 0.0, f_x, capture_setting.height / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0 };
    return cam_info_msg;
}

void AirsimRosWrapper::droneStateTimerCallback(const ros::TimerEvent& event)
{
    try {
        // todo this is global origin
        // origin_geo_point_pub_.publish(origin_geo_point_msg_);

        // get the basic vehicle pose and environmental state
        const auto now = updateState();

        // on init, will publish 0 to /clock as expected for use_sim_time compatibility
        if (!airsim_client_->simIsPaused()) {
            // airsim_client needs to provide the simulation time in a future version of the API
            ros_clock_.clock = now;
        }
        // publish the simulation clock
        if (publish_clock_) {
            clock_pub_.publish(ros_clock_);
        }

        // publish vehicle state, odom, and all basic sensor types
        publishVehicleState();

        // send any commands out to the vehicles
        // update_commands();
    }
    catch (rpc::rpc_error& e) {
        std::cout << "error" << std::endl;
        std::string msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API:" << std::endl
                  << msg << std::endl;
    }
}

void AirsimRosWrapper::imgResponseTimerCallback(const ros::TimerEvent& event)
{
    try {
        int image_response_idx = 0;
        const std::vector<ImageResponse>& img_response = airsim_client_images_.simGetImages(airsim_img_request_vec_);
        processAndPublishImgResponse(img_response, image_response_idx, vehicle_ros_->vehicle_name);
    }

    catch (rpc::rpc_error& e) {
        std::string msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API, didn't get image response." << std::endl
                  << msg << std::endl;
    }
}

//  void AirsimRosWrapper::getStereoAndDepthImages(const std::vector<ImageResponse>& img_response_vec) {
//     // using namespace msr::airlib;

//     // typedef VehicleCameraBase::ImageRequest ImageRequest;
//     // typedef VehicleCameraBase::ImageResponse ImageResponse;
//     // typedef VehicleCameraBase::ImageType ImageType;

//     // get right, left and depth images. First two as png, second as float16.
//     std::vector<ImageRequest> request = { 
//         //png format
//         ImageRequest("0", ImageType::Scene),
//         //uncompressed RGB array bytes
//         ImageRequest("1", ImageType::Scene, false, false),       
//         //floating point uncompressed image  
//         ImageRequest("1", ImageType::DepthPlanar, true) 
//     };

//     img_response_vec = airsim_client_images_.simGetImages(request);

//     // return img_response;
//     // do something with response which contains image data, pose, timestamp etc
// }

void AirsimRosWrapper::lidarTimerCallback(const ros::TimerEvent& event)
{
    try {
        for (auto& vehicle_name_ptr_pair : vehicle_name_ptr_map_) {
            if (!vehicle_name_ptr_pair.second->lidar_pubs.empty()) {
                for (auto& lidar_publisher : vehicle_name_ptr_pair.second->lidar_pubs) {
                    auto lidar_data = airsim_client_lidar_.getLidarData(lidar_publisher.sensor_name, vehicle_name_ptr_pair.first);
                    sensor_msgs::PointCloud2 lidar_msg = getLidarMsgFromAirsim(lidar_data, vehicle_name_ptr_pair.first, lidar_publisher.sensor_name);
                    lidar_publisher.publisher.publish(lidar_msg);
                }
            }
        }
    }
    catch (rpc::rpc_error& e) {
        std::string msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API, didn't get lidar response." << std::endl
                  << msg << std::endl;
    }
}

// https://docs.ros.org/jade/api/sensor_msgs/html/point__cloud__conversion_8h_source.html#l00066
// look at UnrealLidarSensor.cpp UnrealLidarSensor::getPointCloud() for math
// read this carefully https://docs.ros.org/kinetic/api/sensor_msgs/html/msg/PointCloud2.html
sensor_msgs::PointCloud2 AirsimRosWrapper::getLidarMsgFromAirsim(const msr::airlib::LidarData& lidar_data, const std::string& vehicle_name, const std::string& sensor_name) const
{
    sensor_msgs::PointCloud2 lidar_msg;
    lidar_msg.header.stamp = ros::Time::now();
    lidar_msg.header.frame_id = vehicle_name + "/" + sensor_name;

    if (lidar_data.point_cloud.size() > 3) {
        lidar_msg.height = 1;
        lidar_msg.width = lidar_data.point_cloud.size() / 3;

        lidar_msg.fields.resize(3);
        lidar_msg.fields[0].name = "x";
        lidar_msg.fields[1].name = "y";
        lidar_msg.fields[2].name = "z";

        int offset = 0;

        for (size_t d = 0; d < lidar_msg.fields.size(); ++d, offset += 4) {
            lidar_msg.fields[d].offset = offset;
            lidar_msg.fields[d].datatype = sensor_msgs::PointField::FLOAT32;
            lidar_msg.fields[d].count = 1;
        }

        lidar_msg.is_bigendian = false;
        lidar_msg.point_step = offset; // 4 * num fields
        lidar_msg.row_step = lidar_msg.point_step * lidar_msg.width;

        lidar_msg.is_dense = true; // todo
        std::vector<float> data_std = lidar_data.point_cloud;

        const unsigned char* bytes = reinterpret_cast<const unsigned char*>(data_std.data());
        std::vector<unsigned char> lidar_msg_data(bytes, bytes + sizeof(float) * data_std.size());
        lidar_msg.data = std::move(lidar_msg_data);
    }
    else {
        // msg = []
    }

    return lidar_msg;
}

ros::Time AirsimRosWrapper::updateState()
{
    bool got_sim_time = true;
    ros::Time curr_ros_time = ros::Time::now();

    //should be easier way to get the sim time through API, something like:
    //msr::airlib::Environment::State env = airsim_client_->simGetGroundTruthEnvironment("");
    //curr_ros_time = airsim_timestamp_to_ros(env.clock().nowNanos());

    // iterate over drones
    for (auto& vehicle_name_ptr_pair : vehicle_name_ptr_map_) {
        ros::Time vehicle_time;
        // get drone state from airsim
        auto& vehicle_ros = vehicle_name_ptr_pair.second;

        // vehicle environment, we can get ambient temperature here and other truths
        // auto env_data = airsim_client_->simGetGroundTruthEnvironment(vehicle_ros->vehicle_name);

        // auto drone = static_cast<MultiRotorROS*>(vehicle_ros.get());
        // drone->curr_drone_state = airsim_client_->getMultirotorState(vehicle_ros->vehicle_name);

        // vehicle_time = airsimTimestampToRos(drone->curr_drone_state.timestamp);
        // // ROS_INFO("in updatestate, time is %d, and ros time %d", vehicle_time.toSec(), curr_ros_time.toSec());
        // if (!got_sim_time) {
        //     curr_ros_time = vehicle_time;
        //     got_sim_time = true;
        // }

        vehicle_ros->gps_sensor_msg = getGpsSensorMsgFromMavros();
        vehicle_ros->gps_sensor_msg.header.stamp = curr_ros_time; //vehicle_time;

        // vehicle_ros->curr_odom = getOdomMsgFromMavros();

        vehicle_ros->stamp = curr_ros_time;// vehicle_time;

        // airsim_ros_pkgs::Environment env_msg = get_environment_msg_from_airsim(env_data);
        // env_msg.header.frame_id = vehicle_ros->vehicle_name;
        // env_msg.header.stamp = vehicle_time;
        // vehicle_ros->env_msg = env_msg;

        // convert airsim drone state to ROS msgs
        // vehicle_ros->curr_odom.header.frame_id = odom_frame_id_;
        // vehicle_ros->curr_odom.child_frame_id = vehicle_frame_id_;
        // vehicle_ros->curr_odom.header.stamp = curr_ros_time;// vehicle_time;
    }

    return curr_ros_time;
}

void AirsimRosWrapper::odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
    latest_odom_ = *odom_msg;
}

// nav_msgs::Odometry AirsimRosWrapper::getOdomMsgFromMavros() const {
//     return latest_odom_;
// }


void AirsimRosWrapper::publishVehicleState()
{
    for (auto& vehicle_name_ptr_pair : vehicle_name_ptr_map_) {
        auto& vehicle_ros = vehicle_name_ptr_pair.second;

        // simulation environment truth
        // vehicle_ros->env_pub.publish(vehicle_ros->env_msg);

        // odom and transforms
        nav_msgs::Odometry odom = latest_odom_;
        vehicle_ros->odom_local_pub.publish(odom);
        // publishOdomTf(odom);
        // publishMapTf();

        // ground truth GPS position from sim/HITL
        // vehicle_ros->global_gps_pub.publish(vehicle_ros->gps_sensor_msg);

        // Most vehicle data published through MAVROS
        publishStaticTransforms(vehicle_ros.get());
    }
}

void AirsimRosWrapper::publishStaticTransforms(VehicleROS* vehicle_ros)
{
    if (vehicle_ros && !vehicle_ros->static_tf_msg_vec.empty()) {
        for (auto& static_tf_msg : vehicle_ros->static_tf_msg_vec) {
            static_tf_msg.header.stamp = vehicle_ros->stamp;
            static_tf_pub_.sendTransform(static_tf_msg);
        }
    }
}

void AirsimRosWrapper::publishMapTf()
{
    //shd be map to odom link
    // TODO make this a real tf, not identity
    geometry_msgs::TransformStamped map_tf;
    map_tf.header.frame_id = map_frame_id_;
    map_tf.header.stamp = ros::Time::now();
    map_tf.child_frame_id = odom_frame_id_;
    map_tf.transform.translation.x = 0;
    map_tf.transform.translation.y = 0;
    map_tf.transform.translation.z = 0;
    map_tf.transform.rotation.x = 0;
    map_tf.transform.rotation.y = 0;
    map_tf.transform.rotation.z = 0;
    map_tf.transform.rotation.w = 1;
    tf_broadcaster_.sendTransform(map_tf);
}

void AirsimRosWrapper::publishOdomTf(const nav_msgs::Odometry& odom_msg)
{
    if (odom_msg.pose.pose.orientation.x == 0 && odom_msg.pose.pose.orientation.y == 0 && odom_msg.pose.pose.orientation.z == 0 && odom_msg.pose.pose.orientation.w == 0) {
        ROS_INFO_THROTTLE(30, "Odom msg invalid.");
        return;
    }
    //shd be odom to base link
    geometry_msgs::TransformStamped odom_tf;
    // TODO: why does MAVROS odom have map to odom instead of odom to base_link? mapping still works but verify all tf frames correct later
    odom_tf.header.frame_id = odom_frame_id_;// odom_msg.header;
    odom_tf.header.stamp = odom_msg.header.stamp;
    odom_tf.child_frame_id = vehicle_frame_id_;// odom_msg.child_frame_id;
    odom_tf.transform.translation.x = odom_msg.pose.pose.position.x;
    odom_tf.transform.translation.y = odom_msg.pose.pose.position.y;
    odom_tf.transform.translation.z = odom_msg.pose.pose.position.z;
    odom_tf.transform.rotation.x = odom_msg.pose.pose.orientation.x;
    odom_tf.transform.rotation.y = odom_msg.pose.pose.orientation.y;
    odom_tf.transform.rotation.z = odom_msg.pose.pose.orientation.z;
    odom_tf.transform.rotation.w = odom_msg.pose.pose.orientation.w;
    tf_broadcaster_.sendTransform(odom_tf);
}

sensor_msgs::ImagePtr AirsimRosWrapper::getImgMsgFromResponse(const ImageResponse& img_response,
                                                                  const ros::Time curr_ros_time,
                                                                  const std::string frame_id)
{
    sensor_msgs::ImagePtr img_msg_ptr = boost::make_shared<sensor_msgs::Image>();
    img_msg_ptr->data = img_response.image_data_uint8;
    img_msg_ptr->step = img_response.width * 3; // todo un-hardcode. image_width*num_bytes
    img_msg_ptr->header.stamp = curr_ros_time;// airsimTimestampToRos(img_response.time_stamp);
    img_msg_ptr->header.frame_id = frame_id;
    img_msg_ptr->height = img_response.height;
    img_msg_ptr->width = img_response.width;
    img_msg_ptr->encoding = "bgr8";
    img_msg_ptr->is_bigendian = 0;
    return img_msg_ptr;
}

sensor_msgs::ImagePtr AirsimRosWrapper::getDepthImgMsgFromResponse(const ImageResponse& img_response,
                                                                        const ros::Time curr_ros_time,
                                                                        const std::string frame_id)
{
    // todo using img_response.image_data_float direclty as done get_img_msg_from_response() throws an error,
    // hence the dependency on opencv and cv_bridge. however, this is an extremely fast op, so no big deal. <-- this note from microsoft original. verify with profiling later.
    cv::Mat depth_img = manualDecodeDepth(img_response);
    sensor_msgs::ImagePtr depth_img_msg = cv_bridge::CvImage(std_msgs::Header(), "32FC1", depth_img).toImageMsg();
    depth_img_msg->header.stamp = curr_ros_time;// airsimTimestampToRos(img_response.time_stamp);
    depth_img_msg->header.frame_id = frame_id;
    return depth_img_msg;
}

cv::Mat AirsimRosWrapper::manualDecodeDepth(const ImageResponse& img_response) const
{
    cv::Mat mat(img_response.height, img_response.width, CV_32FC1, cv::Scalar(0));
    int img_width = img_response.width;

    for (int row = 0; row < img_response.height; row++)
        for (int col = 0; col < img_width; col++)
            mat.at<float>(row, col) = img_response.image_data_float[row * img_width + col];
    return mat;
}

void AirsimRosWrapper::processAndPublishImgResponse(const std::vector<ImageResponse>& img_response_vec, const int img_response_idx, const std::string& vehicle_name)
{
    // todo add option to use airsim time (image_response.TTimePoint) like Gazebo /use_sim_time param
    ros::Time curr_ros_time = ros::Time::now();
    int img_response_idx_internal = 0;

    for (const auto& curr_img_response : img_response_vec) {
        // update timestamp of saved cam info msgs
        ros::Time img_time_stamp = airsimTimestampToRos(curr_img_response.time_stamp);
        camera_info_msg_vec_[img_response_idx_internal].header.stamp = img_time_stamp;
        cam_info_pub_vec_[img_response_idx_internal].publish(camera_info_msg_vec_[img_response_idx_internal]);

        // DepthPlanar / DepthPerspective / DepthVis / DisparityNormalized
        if (curr_img_response.pixels_as_float) {
            image_pub_vec_[img_response_idx_internal].publish(getDepthImgMsgFromResponse(curr_img_response,
                                                                                              img_time_stamp,
                                                                                              curr_img_response.camera_name + "_optical"));
        }
        // Scene / Segmentation / SurfaceNormals / Infrared
        else {
            image_pub_vec_[img_response_idx_internal].publish(getImgMsgFromResponse(curr_img_response,
                                                                                        img_time_stamp,
                                                                                        curr_img_response.camera_name + "_optical"));
        }
        img_response_idx_internal++;
    }
}

// publish camera transforms
// camera poses are obtained from airsim's client API which are in (local) NED frame.
// We first do a change of basis to camera optical frame (Z forward, X right, Y down)
// void AirsimRosWrapper::publishCameraTf(const ImageResponse& img_response, const ros::Time& ros_time, const std::string& frame_id, const std::string& child_frame_id)
// {
//     geometry_msgs::TransformStamped cam_tf_body_msg;
//     cam_tf_body_msg.header.stamp = airsimTimestampToRos(img_response.time_stamp);
//     cam_tf_body_msg.header.frame_id = frame_id;
//     cam_tf_body_msg.child_frame_id = child_frame_id + "_body";
//     cam_tf_body_msg.transform.translation.x = img_response.camera_position.x();
//     cam_tf_body_msg.transform.translation.y = img_response.camera_position.y();
//     cam_tf_body_msg.transform.translation.z = img_response.camera_position.z();
//     cam_tf_body_msg.transform.rotation.x = img_response.camera_orientation.x();
//     cam_tf_body_msg.transform.rotation.y = img_response.camera_orientation.y();
//     cam_tf_body_msg.transform.rotation.z = img_response.camera_orientation.z();
//     cam_tf_body_msg.transform.rotation.w = img_response.camera_orientation.w();

//     geometry_msgs::TransformStamped cam_tf_optical_msg;
//     cam_tf_optical_msg.header.stamp = airsimTimestampToRos(img_response.time_stamp);
//     cam_tf_optical_msg.header.frame_id = frame_id;
//     cam_tf_optical_msg.child_frame_id = child_frame_id + "_optical";
//     cam_tf_optical_msg.transform.translation.x = cam_tf_body_msg.transform.translation.x;
//     cam_tf_optical_msg.transform.translation.y = cam_tf_body_msg.transform.translation.y;
//     cam_tf_optical_msg.transform.translation.z = cam_tf_body_msg.transform.translation.z;

//     tf2::Quaternion quat_cam_body;
//     tf2::Quaternion quat_cam_optical;
//     tf2::convert(cam_tf_body_msg.transform.rotation, quat_cam_body);
//     tf2::Matrix3x3 mat_cam_body(quat_cam_body);
//     // tf2::Matrix3x3 mat_cam_optical = matrix_cam_body_to_optical_ * mat_cam_body * matrix_cam_body_to_optical_inverse_;
//     // tf2::Matrix3x3 mat_cam_optical = matrix_cam_body_to_optical_ * mat_cam_body;
//     tf2::Matrix3x3 mat_cam_optical;
//     mat_cam_optical.setValue(mat_cam_body.getColumn(1).getX(), mat_cam_body.getColumn(2).getX(), mat_cam_body.getColumn(0).getX(), mat_cam_body.getColumn(1).getY(), mat_cam_body.getColumn(2).getY(), mat_cam_body.getColumn(0).getY(), mat_cam_body.getColumn(1).getZ(), mat_cam_body.getColumn(2).getZ(), mat_cam_body.getColumn(0).getZ());
//     mat_cam_optical.getRotation(quat_cam_optical);
//     quat_cam_optical.normalize();
//     tf2::convert(quat_cam_optical, cam_tf_optical_msg.transform.rotation);
//     ROS_WARN("would have sent cam body tf with stamp %d", cam_tf_body_msg.header.stamp.toSec());
//     ROS_WARN("would have sent cam optical tf with stamp %d", cam_tf_optical_msg.header.stamp.toSec());

//     tf_broadcaster_.sendTransform(cam_tf_body_msg);
//     tf_broadcaster_.sendTransform(cam_tf_optical_msg);
// }

ros::Time AirsimRosWrapper::airsimTimestampToRos(const msr::airlib::TTimePoint& stamp) const
{
    // airsim appears to use chrono::system_clock with nanosecond precision
    std::chrono::nanoseconds dur(stamp);
    std::chrono::time_point<std::chrono::system_clock> tp(dur);
    ros::Time cur_time = chronoTimestampToRos(tp);
    return cur_time;
}

ros::Time AirsimRosWrapper::chronoTimestampToRos(const std::chrono::system_clock::time_point& stamp) const
{
    auto dur = std::chrono::duration<double>(stamp.time_since_epoch());
    ros::Time cur_time;
    cur_time.fromSec(dur.count());
    return cur_time;
}

void AirsimRosWrapper::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &gps_msg) {
    latest_gps_ = *gps_msg;
}

sensor_msgs::NavSatFix AirsimRosWrapper::getGpsSensorMsgFromMavros() const
{
    return latest_gps_;
}

}
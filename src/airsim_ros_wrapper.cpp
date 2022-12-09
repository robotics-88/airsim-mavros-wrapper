/* 
© 2022 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#include "airsim_ros_wrapper/airsim_ros_wrapper.h"


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
  : private_nh_("~")
  , nh_(node)
  , tf_listener_(tf_buffer_)
  , host_ip_("127.0.0.1")
  , odom_frame_id_("odom")
  , world_frame_id_("world")
  , map_frame_id_("map")
  , airsim_client_images_(host_ip_)
  , airsim_client_lidar_(host_ip_)
{
    initializeRos();
}

AirsimRosWrapper::~AirsimRosWrapper(){}

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
    // private_nh_.param("world_frame_id", world_frame_id_, world_frame_id_);
    // odom_frame_id_ = world_frame_id_ == AIRSIM_FRAME_ID ? AIRSIM_ODOM_FRAME_ID : ENU_ODOM_FRAME_ID;
    // private_nh_.param("odom_frame_id", odom_frame_id_, odom_frame_id_);
    // isENU_ = !(odom_frame_id_ == AIRSIM_ODOM_FRAME_ID);
    // private_nh_.param("coordinate_system_enu", isENU_, isENU_);
    // vel_cmd_duration_ = 0.05; // todo rosparam

    createRosPubsFromSettingsJson();
    airsim_control_update_timer_ = private_nh_.createTimer(ros::Duration(update_airsim_control_every_n_sec), &AirsimRosWrapper::droneStateTimerCallback, this);
}

void AirsimRosWrapper::createRosPubsFromSettingsJson() {
    airsim_img_request_vehicle_name_pair_vec_.clear();
    image_pub_vec_.clear();
    cam_info_pub_vec_.clear();
    camera_info_msg_vec_.clear();
    vehicle_name_ptr_map_.clear();
    size_t lidar_cnt = 0;

    image_transport::ImageTransport image_transporter(private_nh_);
    ROS_INFO("in create ros pubs from settings, AS singleton veh size: %d", AirSimSettings::singleton().vehicles.size());

    // iterate over std::map<std::string, std::unique_ptr<VehicleSetting>> vehicles;
    for (const auto& curr_vehicle_elem : AirSimSettings::singleton().vehicles) {
        auto& vehicle_setting = curr_vehicle_elem.second;
        auto curr_vehicle_name = curr_vehicle_elem.first;

        nh_.setParam("/vehicle_name", curr_vehicle_name);

        setNansToZerosInPose(*vehicle_setting);

        std::unique_ptr<VehicleROS> vehicle_ros = std::unique_ptr<MultiRotorROS>(new MultiRotorROS());

        vehicle_ros->odom_frame_id = curr_vehicle_name + "/" + odom_frame_id_;
        vehicle_ros->vehicle_name = curr_vehicle_name;

        appendStaticVehicleTf(vehicle_ros.get(), *vehicle_setting);

        vehicle_ros->odom_local_pub = private_nh_.advertise<nav_msgs::Odometry>(curr_vehicle_name + "/" + odom_frame_id_, 10);

        // vehicle_ros->env_pub = private_nh_.advertise<airsim_ros_pkgs::Environment>(curr_vehicle_name + "/environment", 10);

        // vehicle_ros->global_gps_pub = private_nh_.advertise<sensor_msgs::NavSatFix>(curr_vehicle_name + "/global_gps", 10);

        // auto drone = static_cast<MultiRotorROS*>(vehicle_ros.get());

        // bind to a single callback. todo optimal subs queue length
        // bind multiple topics to a single callback, but keep track of which vehicle name it was by passing curr_vehicle_name as the 2nd argument
        // drone->vel_cmd_body_frame_sub = private_nh_.subscribe<airsim_ros_pkgs::VelCmd>(
        //     curr_vehicle_name + "/vel_cmd_body_frame",
        //     1,
        //     boost::bind(&AirsimROSWrapper::vel_cmd_body_frame_cb, this, _1, vehicle_ros->vehicle_name));
        // // TODO: ros::TransportHints().tcpNoDelay();

        // drone->vel_cmd_world_frame_sub = private_nh_.subscribe<airsim_ros_pkgs::VelCmd>(
        //     curr_vehicle_name + "/vel_cmd_world_frame",
        //     1,
        //     boost::bind(&AirsimROSWrapper::vel_cmd_world_frame_cb, this, _1, vehicle_ros->vehicle_name));

        // drone->takeoff_srvr = private_nh_.advertiseService<airsim_ros_pkgs::Takeoff::Request, airsim_ros_pkgs::Takeoff::Response>(
        //     curr_vehicle_name + "/takeoff",
        //     boost::bind(&AirsimROSWrapper::takeoff_srv_cb, this, _1, _2, vehicle_ros->vehicle_name));

        // drone->land_srvr = private_nh_.advertiseService<airsim_ros_pkgs::Land::Request, airsim_ros_pkgs::Land::Response>(
        //     curr_vehicle_name + "/land",
        //     boost::bind(&AirsimROSWrapper::land_srv_cb, this, _1, _2, vehicle_ros->vehicle_name));

            // vehicle_ros.reset_srvr = private_nh_.advertiseService(curr_vehicle_name + "/reset",&AirsimROSWrapper::reset_srv_cb, this);

        // iterate over camera map std::map<std::string, CameraSetting> .cameras;
        for (auto& curr_camera_elem : vehicle_setting->cameras) {
            auto& camera_setting = curr_camera_elem.second;
            auto& curr_camera_name = curr_camera_elem.first;

            setNansToZerosInPose(*vehicle_setting, camera_setting);
            appendStaticCameraTf(vehicle_ros.get(), curr_camera_name, camera_setting);
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
                        current_image_request_vec.push_back(ImageRequest(curr_camera_name, curr_image_type, false, false));
                    }
                    // if {DepthPlanar, DepthPerspective,DepthVis, DisparityNormalized}, get float image
                    else {
                        current_image_request_vec.push_back(ImageRequest(curr_camera_name, curr_image_type, true));
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

        // // iterate over sensors
        // std::vector<SensorPublisher> sensors;
        // for (const auto& [sensor_name, sensor_setting] : vehicle_setting->sensors) {
        //     if (sensor_setting->enabled) {
        //         SensorPublisher sensor_publisher;
        //         sensor_publisher.sensor_name = sensor_name;
        //         sensor_publisher.sensor_type = sensor_setting->sensor_type;
        //         switch (sensor_setting->sensor_type) {
        //         case SensorBase::SensorType::Barometer: {
        //             ROS_INFO_STREAM(sensor_name << ": Barometer");
        //             sensor_publisher.publisher = private_nh_.advertise<airsim_ros_pkgs::Altimeter>(curr_vehicle_name + "/altimeter/" + sensor_name, 10);
        //             break;
        //         }
        //         case SensorBase::SensorType::Imu: {
        //             ROS_INFO_STREAM(sensor_name << ": IMU");
        //             sensor_publisher.publisher = private_nh_.advertise<sensor_msgs::Imu>(curr_vehicle_name + "/imu/" + sensor_name, 10);
        //             break;
        //         }
        //         case SensorBase::SensorType::Gps: {
        //             ROS_INFO_STREAM(sensor_name << ": GPS");
        //             sensor_publisher.publisher = private_nh_.advertise<sensor_msgs::NavSatFix>(curr_vehicle_name + "/gps/" + sensor_name, 10);
        //             break;
        //         }
        //         case SensorBase::SensorType::Magnetometer: {
        //             ROS_INFO_STREAM(sensor_name << ": Magnetometer");
        //             sensor_publisher.publisher = private_nh_.advertise<sensor_msgs::MagneticField>(curr_vehicle_name + "/magnetometer/" + sensor_name, 10);
        //             break;
        //         }
        //         case SensorBase::SensorType::Distance: {
        //             ROS_INFO_STREAM(sensor_name << ": Distance sensor");
        //             sensor_publisher.publisher = private_nh_.advertise<sensor_msgs::Range>(curr_vehicle_name + "/distance/" + sensor_name, 10);
        //             break;
        //         }
        //         case SensorBase::SensorType::Lidar: {
        //             ROS_INFO_STREAM(sensor_name << ": Lidar");
        //             auto lidar_setting = *static_cast<LidarSetting*>(sensor_setting.get());
        //             msr::airlib::LidarSimpleParams params;
        //             params.initializeFromSettings(lidar_setting);
        //             append_static_lidar_tf(vehicle_ros.get(), sensor_name, params);
        //             sensor_publisher.publisher = private_nh_.advertise<sensor_msgs::PointCloud2>(curr_vehicle_name + "/lidar/" + sensor_name, 10);
        //             break;
        //         }
        //         default: {
        //             throw std::invalid_argument("Unexpected sensor type");
        //         }
        //         }
        //         sensors.emplace_back(sensor_publisher);
        //     }
        // }

        // // we want fast access to the lidar sensors for callback handling, sort them out now
        // auto isLidar = [](const SensorPublisher& pub) {
        //     return pub.sensor_type == SensorBase::SensorType::Lidar;
        // };
        // size_t cnt = std::count_if(sensors.begin(), sensors.end(), isLidar);
        // lidar_cnt += cnt;
        // vehicle_ros->lidar_pubs.resize(cnt);
        // vehicle_ros->sensor_pubs.resize(sensors.size() - cnt);
        // std::partition_copy(sensors.begin(), sensors.end(), vehicle_ros->lidar_pubs.begin(), vehicle_ros->sensor_pubs.begin(), isLidar);

        vehicle_name_ptr_map_.emplace(curr_vehicle_name, std::move(vehicle_ros)); // allows fast lookup in command callbacks in case of a lot of drones
    }

    // // add takeoff and land all services if more than 2 drones
    // if (vehicle_name_ptr_map_.size() > 1 && airsim_mode_ == AIRSIM_MODE::DRONE) {
    //     takeoff_all_srvr_ = private_nh_.advertiseService("all_robots/takeoff", &AirsimROSWrapper::takeoff_all_srv_cb, this);
    //     land_all_srvr_ = private_nh_.advertiseService("all_robots/land", &AirsimROSWrapper::land_all_srv_cb, this);

    //     // gimbal_angle_quat_cmd_sub_ = nh_.subscribe("gimbal_angle_quat_cmd", 50, &AirsimROSWrapper::gimbal_angle_quat_cmd_cb, this);

    //     vel_cmd_all_body_frame_sub_ = private_nh_.subscribe("all_robots/vel_cmd_body_frame", 1, &AirsimROSWrapper::vel_cmd_all_body_frame_cb, this);
    //     vel_cmd_all_world_frame_sub_ = private_nh_.subscribe("all_robots/vel_cmd_world_frame", 1, &AirsimROSWrapper::vel_cmd_all_world_frame_cb, this);

    //     vel_cmd_group_body_frame_sub_ = private_nh_.subscribe("group_of_robots/vel_cmd_body_frame", 1, &AirsimROSWrapper::vel_cmd_group_body_frame_cb, this);
    //     vel_cmd_group_world_frame_sub_ = private_nh_.subscribe("group_of_robots/vel_cmd_world_frame", 1, &AirsimROSWrapper::vel_cmd_group_world_frame_cb, this);

    //     takeoff_group_srvr_ = private_nh_.advertiseService("group_of_robots/takeoff", &AirsimROSWrapper::takeoff_group_srv_cb, this);
    //     land_group_srvr_ = private_nh_.advertiseService("group_of_robots/land", &AirsimROSWrapper::land_group_srv_cb, this);
    // }

    // // todo add per vehicle reset in AirLib API
    // reset_srvr_ = private_nh_.advertiseService("reset", &AirsimROSWrapper::reset_srv_cb, this);

    // if (publish_clock_) {
    //     clock_pub_ = private_nh_.advertise<rosgraph_msgs::Clock>("clock", 1);
    // }

    // if >0 cameras, add one more thread for img_request_timer_cb
    // ROS_INFO("in ros settings setup airsim, sz of img vehicle pair list %d", airsim_img_request_vehicle_name_pair_vec_.size());
    if (!airsim_img_request_vehicle_name_pair_vec_.empty()) {
        double update_airsim_img_response_every_n_sec;
        private_nh_.getParam("update_airsim_img_response_every_n_sec", update_airsim_img_response_every_n_sec);

        ros::TimerOptions timer_options(ros::Duration(update_airsim_img_response_every_n_sec),
                                        boost::bind(&AirsimRosWrapper::imgResponseTimerCallback, this, _1),
                                        &img_timer_cb_queue_);

        airsim_img_response_timer_ = private_nh_.createTimer(timer_options);
        is_used_img_timer_cb_queue_ = true;
    }

    // // lidars update on their own callback/thread at a given rate
    // if (lidar_cnt > 0) {
    //     double update_lidar_every_n_sec;
    //     private_nh_.getParam("update_lidar_every_n_sec", update_lidar_every_n_sec);
    //     // private_nh_.setCallbackQueue(&lidar_timer_cb_queue_);

    //     ros::TimerOptions timer_options(ros::Duration(update_lidar_every_n_sec),
    //                                     boost::bind(&AirsimROSWrapper::lidar_timer_cb, this, _1),
    //                                     &lidar_timer_cb_queue_);

    //     airsim_lidar_update_timer_ = private_nh_.createTimer(timer_options);
    //     is_used_lidar_timer_cb_queue_ = true;
    // }

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

void AirsimRosWrapper::appendStaticVehicleTf(VehicleROS* vehicle_ros, const VehicleSetting& vehicle_setting)
{
    geometry_msgs::TransformStamped vehicle_tf_msg;
    vehicle_tf_msg.header.frame_id = world_frame_id_;
    vehicle_tf_msg.header.stamp = ros::Time(0);
    vehicle_tf_msg.child_frame_id = vehicle_ros->vehicle_name;
    vehicle_tf_msg.transform.translation.x = vehicle_setting.position.x();
    vehicle_tf_msg.transform.translation.y = vehicle_setting.position.y();
    vehicle_tf_msg.transform.translation.z = vehicle_setting.position.z();
    tf2::Quaternion quat;
    quat.setRPY(vehicle_setting.rotation.roll, vehicle_setting.rotation.pitch, vehicle_setting.rotation.yaw);
    vehicle_tf_msg.transform.rotation.x = quat.x();
    vehicle_tf_msg.transform.rotation.y = quat.y();
    vehicle_tf_msg.transform.rotation.z = quat.z();
    vehicle_tf_msg.transform.rotation.w = quat.w();

    vehicle_ros->static_tf_msg_vec.emplace_back(vehicle_tf_msg);
}


void AirsimRosWrapper::appendStaticCameraTf(VehicleROS* vehicle_ros, const std::string& camera_name, const CameraSetting& camera_setting)
{
    geometry_msgs::TransformStamped static_cam_tf_body_msg;
    static_cam_tf_body_msg.header.frame_id = vehicle_ros->vehicle_name + "/" + odom_frame_id_;
    static_cam_tf_body_msg.child_frame_id = camera_name + "_body/static";
    static_cam_tf_body_msg.transform.translation.x = camera_setting.position.x();
    static_cam_tf_body_msg.transform.translation.y = camera_setting.position.y();
    static_cam_tf_body_msg.transform.translation.z = camera_setting.position.z();
    tf2::Quaternion quat;
    quat.setRPY(camera_setting.rotation.roll, camera_setting.rotation.pitch, camera_setting.rotation.yaw);
    static_cam_tf_body_msg.transform.rotation.x = quat.x();
    static_cam_tf_body_msg.transform.rotation.y = quat.y();
    static_cam_tf_body_msg.transform.rotation.z = quat.z();
    static_cam_tf_body_msg.transform.rotation.w = quat.w();

    geometry_msgs::TransformStamped static_cam_tf_optical_msg = static_cam_tf_body_msg;
    static_cam_tf_optical_msg.child_frame_id = camera_name + "_optical/static";

    tf2::Quaternion quat_cam_body;
    tf2::Quaternion quat_cam_optical;
    tf2::convert(static_cam_tf_body_msg.transform.rotation, quat_cam_body);
    tf2::Matrix3x3 mat_cam_body(quat_cam_body);
    tf2::Matrix3x3 mat_cam_optical;
    mat_cam_optical.setValue(mat_cam_body.getColumn(1).getX(), mat_cam_body.getColumn(2).getX(), mat_cam_body.getColumn(0).getX(), mat_cam_body.getColumn(1).getY(), mat_cam_body.getColumn(2).getY(), mat_cam_body.getColumn(0).getY(), mat_cam_body.getColumn(1).getZ(), mat_cam_body.getColumn(2).getZ(), mat_cam_body.getColumn(0).getZ());
    mat_cam_optical.getRotation(quat_cam_optical);
    quat_cam_optical.normalize();
    tf2::convert(quat_cam_optical, static_cam_tf_optical_msg.transform.rotation);

    vehicle_ros->static_tf_msg_vec.emplace_back(static_cam_tf_body_msg);
    vehicle_ros->static_tf_msg_vec.emplace_back(static_cam_tf_optical_msg);
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

        // // on init, will publish 0 to /clock as expected for use_sim_time compatibility
        // if (!airsim_client_->simIsPaused()) {
        //     // airsim_client needs to provide the simulation time in a future version of the API
        //     ros_clock_.clock = now;
        // }
        // // publish the simulation clock
        // if (publish_clock_) {
        //     clock_pub_.publish(ros_clock_);
        // }

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
    // ROS_INFO("entered img resposne timer cb, vec len is %d", airsim_img_request_vehicle_name_pair_vec_.size());
    try {
        int image_response_idx = 0;
        for (const auto& airsim_img_request_vehicle_name_pair : airsim_img_request_vehicle_name_pair_vec_) {
            const std::vector<ImageResponse>& img_response = airsim_client_images_.simGetImages(airsim_img_request_vehicle_name_pair.first, airsim_img_request_vehicle_name_pair.second, true);

            if (img_response.size() == airsim_img_request_vehicle_name_pair.first.size()) {
                processAndPublishImgResponse(img_response, image_response_idx, airsim_img_request_vehicle_name_pair.second);
                image_response_idx += img_response.size();
            }
        }
    }

    catch (rpc::rpc_error& e) {
        std::string msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API, didn't get image response." << std::endl
                  << msg << std::endl;
    }
}

ros::Time AirsimRosWrapper::updateState()
{
    bool got_sim_time = false;
    ros::Time curr_ros_time = ros::Time(0);

    //should be easier way to get the sim time through API, something like:
    //msr::airlib::Environment::State env = airsim_client_->simGetGroundTruthEnvironment("");
    //curr_ros_time = airsim_timestamp_to_ros(env.clock().nowNanos());

    // iterate over drones
    for (auto& vehicle_name_ptr_pair : vehicle_name_ptr_map_) {
        ros::Time vehicle_time;
        // get drone state from airsim
        auto& vehicle_ros = vehicle_name_ptr_pair.second;

        // vehicle environment, we can get ambient temperature here and other truths
        auto env_data = airsim_client_->simGetGroundTruthEnvironment(vehicle_ros->vehicle_name);

        auto drone = static_cast<MultiRotorROS*>(vehicle_ros.get());
        drone->curr_drone_state = airsim_client_->getMultirotorState(vehicle_ros->vehicle_name);

        vehicle_time = airsimTimestampToRos(drone->curr_drone_state.timestamp);
        if (!got_sim_time) {
            curr_ros_time = vehicle_time;
            got_sim_time = true;
        }

        vehicle_ros->gps_sensor_msg = getGpsSensorMsgFromAirsimGeopoint(drone->curr_drone_state.gps_location);
        vehicle_ros->gps_sensor_msg.header.stamp = vehicle_time;

        // vehicle_ros->curr_odom = get_odom_msg_from_multirotor_state(drone->curr_drone_state);

        vehicle_ros->stamp = vehicle_time;

        // airsim_ros_pkgs::Environment env_msg = get_environment_msg_from_airsim(env_data);
        // env_msg.header.frame_id = vehicle_ros->vehicle_name;
        // env_msg.header.stamp = vehicle_time;
        // vehicle_ros->env_msg = env_msg;

        // convert airsim drone state to ROS msgs
        // vehicle_ros->curr_odom.header.frame_id = vehicle_ros->vehicle_name;
        // vehicle_ros->curr_odom.child_frame_id = vehicle_ros->odom_frame_id;
        // vehicle_ros->curr_odom.header.stamp = vehicle_time;
    }

    return curr_ros_time;
}


void AirsimRosWrapper::publishVehicleState()
{
    for (auto& vehicle_name_ptr_pair : vehicle_name_ptr_map_) {
        auto& vehicle_ros = vehicle_name_ptr_pair.second;

        // simulation environment truth
        // vehicle_ros->env_pub.publish(vehicle_ros->env_msg);

        // odom and transforms
        vehicle_ros->odom_local_pub.publish(vehicle_ros->curr_odom);
        // publish_odom_tf(vehicle_ros->curr_odom);

        // ground truth GPS position from sim/HITL
        // vehicle_ros->global_gps_pub.publish(vehicle_ros->gps_sensor_msg);

        // for (auto& sensor_publisher : vehicle_ros->sensor_pubs) {
        //     switch (sensor_publisher.sensor_type) {
        //     case SensorBase::SensorType::Barometer: {
        //         auto baro_data = airsim_client_->getBarometerData(sensor_publisher.sensor_name, vehicle_ros->vehicle_name);
        //         airsim_ros_pkgs::Altimeter alt_msg = get_altimeter_msg_from_airsim(baro_data);
        //         alt_msg.header.frame_id = vehicle_ros->vehicle_name;
        //         sensor_publisher.publisher.publish(alt_msg);
        //         break;
        //     }
        //     case SensorBase::SensorType::Imu: {
        //         auto imu_data = airsim_client_->getImuData(sensor_publisher.sensor_name, vehicle_ros->vehicle_name);
        //         sensor_msgs::Imu imu_msg = get_imu_msg_from_airsim(imu_data);
        //         imu_msg.header.frame_id = vehicle_ros->vehicle_name;
        //         sensor_publisher.publisher.publish(imu_msg);
        //         break;
        //     }
        //     case SensorBase::SensorType::Distance: {
        //         auto distance_data = airsim_client_->getDistanceSensorData(sensor_publisher.sensor_name, vehicle_ros->vehicle_name);
        //         sensor_msgs::Range dist_msg = get_range_from_airsim(distance_data);
        //         dist_msg.header.frame_id = vehicle_ros->vehicle_name;
        //         sensor_publisher.publisher.publish(dist_msg);
        //         break;
        //     }
        //     case SensorBase::SensorType::Gps: {
        //         auto gps_data = airsim_client_->getGpsData(sensor_publisher.sensor_name, vehicle_ros->vehicle_name);
        //         sensor_msgs::NavSatFix gps_msg = get_gps_msg_from_airsim(gps_data);
        //         gps_msg.header.frame_id = vehicle_ros->vehicle_name;
        //         sensor_publisher.publisher.publish(gps_msg);
        //         break;
        //     }
        //     case SensorBase::SensorType::Lidar: {
        //         // handled via callback
        //         break;
        //     }
        //     case SensorBase::SensorType::Magnetometer: {
        //         auto mag_data = airsim_client_->getMagnetometerData(sensor_publisher.sensor_name, vehicle_ros->vehicle_name);
        //         sensor_msgs::MagneticField mag_msg = get_mag_msg_from_airsim(mag_data);
        //         mag_msg.header.frame_id = vehicle_ros->vehicle_name;
        //         sensor_publisher.publisher.publish(mag_msg);
        //         break;
        //     }
        //     }
        // }

        // update_and_publish_static_transforms(vehicle_ros.get());
    }
}

sensor_msgs::ImagePtr AirsimRosWrapper::getImgMsgFromResponse(const ImageResponse& img_response,
                                                                  const ros::Time curr_ros_time,
                                                                  const std::string frame_id)
{
    sensor_msgs::ImagePtr img_msg_ptr = boost::make_shared<sensor_msgs::Image>();
    img_msg_ptr->data = img_response.image_data_uint8;
    img_msg_ptr->step = img_response.width * 3; // todo un-hardcode. image_width*num_bytes
    img_msg_ptr->header.stamp = airsimTimestampToRos(img_response.time_stamp);
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
    // hence the dependency on opencv and cv_bridge. however, this is an extremely fast op, so no big deal.
    cv::Mat depth_img = manualDecodeDepth(img_response);
    sensor_msgs::ImagePtr depth_img_msg = cv_bridge::CvImage(std_msgs::Header(), "32FC1", depth_img).toImageMsg();
    depth_img_msg->header.stamp = airsimTimestampToRos(img_response.time_stamp);
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
    ROS_INFO("entered proc and pub img");
    // todo add option to use airsim time (image_response.TTimePoint) like Gazebo /use_sim_time param
    ros::Time curr_ros_time = ros::Time(0);
    int img_response_idx_internal = img_response_idx;

    for (const auto& curr_img_response : img_response_vec) {
        // todo publishing a tf for each capture type seems stupid. but it foolproofs us against render thread's async stuff, I hope.
        // Ideally, we should loop over cameras and then captures, and publish only one tf.
        // publish_camera_tf(curr_img_response, curr_ros_time, vehicle_name, curr_img_response.camera_name);

        // todo simGetCameraInfo is wrong + also it's only for image type -1.
        // msr::airlib::CameraInfo camera_info = airsim_client_.simGetCameraInfo(curr_img_response.camera_name);

        // update timestamp of saved cam info msgs

        camera_info_msg_vec_[img_response_idx_internal].header.stamp = airsimTimestampToRos(curr_img_response.time_stamp);
        cam_info_pub_vec_[img_response_idx_internal].publish(camera_info_msg_vec_[img_response_idx_internal]);

        // DepthPlanar / DepthPerspective / DepthVis / DisparityNormalized
        if (curr_img_response.pixels_as_float) {
            image_pub_vec_[img_response_idx_internal].publish(getDepthImgMsgFromResponse(curr_img_response,
                                                                                              curr_ros_time,
                                                                                              curr_img_response.camera_name + "_optical"));
        }
        // Scene / Segmentation / SurfaceNormals / Infrared
        else {
            image_pub_vec_[img_response_idx_internal].publish(getImgMsgFromResponse(curr_img_response,
                                                                                        curr_ros_time,
                                                                                        curr_img_response.camera_name + "_optical"));
        }
        img_response_idx_internal++;
    }
}

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

sensor_msgs::NavSatFix AirsimRosWrapper::getGpsSensorMsgFromAirsimGeopoint(const msr::airlib::GeoPoint& geo_point) const
{
    sensor_msgs::NavSatFix gps_msg;
    gps_msg.latitude = geo_point.latitude;
    gps_msg.longitude = geo_point.longitude;
    gps_msg.altitude = geo_point.altitude;
    return gps_msg;
}

}
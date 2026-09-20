#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float32.hpp>

#include <tf2_eigen/tf2_eigen.hpp>
#include <deque>
#include <atomic>
#include <condition_variable>
#include <limits>
#include <cmath>
// #include <pcl/common/transforms.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <open3d/Open3D.h>

#include "open3d_registration/open3d_registration.h"
#include "open3d_conversions/open3d_conversions.h"

#define PI 3.1415926

class KalmanFilter
{
public:
    KalmanFilter() : processVar_(0.0), estimatedMeasVar_(0.0),
                     posteriEstimate_(0.0), posteriErrorEstimate_(1.0)
    {
    }

    void KalmanFilterInit(double processVar, double estimatedMeasVar, double posteriEstimate = 0.0, double posteriErrorEstimate = 1.0)
    {
        processVar_ = processVar;
        estimatedMeasVar_ = estimatedMeasVar;
        posteriEstimate_ = posteriEstimate;
        posteriErrorEstimate_ = posteriErrorEstimate;
    }
    void inputLatestNoisyMeasurement(double measurement)
    {
        double prioriEstimate = posteriEstimate_;
        double prioriErrorEstimate = posteriErrorEstimate_ + processVar_;

        double denominator = prioriErrorEstimate + estimatedMeasVar_;

        // 防止除零导致 NaN
        if (std::abs(denominator) < 1e-10)
        {
            // 如果分母接近零，直接使用测量值
            posteriEstimate_ = measurement;
            posteriErrorEstimate_ = 1.0;
            return;
        }

        double blendingFactor = prioriErrorEstimate / denominator;
        posteriEstimate_ = prioriEstimate + blendingFactor * (measurement - prioriEstimate);
        posteriErrorEstimate_ = (1 - blendingFactor) * prioriErrorEstimate;
    }

    double getLatestEstimatedMeasurement()
    {
        return posteriEstimate_;
    }

private:
    double processVar_;
    double estimatedMeasVar_;
    double posteriEstimate_;
    double posteriErrorEstimate_;
};

class GloabalLocalization : public rclcpp::Node
{
private:
    /* data */
public:
    GloabalLocalization();
    ~GloabalLocalization();

    /// @brief 初始化定位
    bool RegisterScan(bool initializing);
    bool WaitForWork(double seconds);
    void PublishHealth();

    /// @brief 订阅fast_lio里程计信息
    void CallbackPrediction(const nav_msgs::msg::Odometry::SharedPtr message);
    void CallbackBaselink2Odom(const nav_msgs::msg::Odometry::SharedPtr baselink2odom);
    /// @brief 订阅在baselink下的点云
    void CallbackScan(const sensor_msgs::msg::PointCloud2::SharedPtr scan_in_baselink);

    /// @brief 订阅在初始位姿
    void CallbackInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr initialpose);

    void StartLoc();

    void Localization();

    /// @brief 欧拉角转mat3x3
    /// @param euler
    /// @return
    Eigen::Matrix3d Euler2Matrix3d(const Eigen::Vector3d euler);

    /// @brief 获取tf关系到矩阵
    /// @param frame_id
    /// @param child_frame_id
    /// @param matrix
    /// @return
    bool GetTfTransformToMatrix(
        std::string frame_id, std::string child_frame_id, Eigen::Matrix4d &matrix);

    /// @brief compute 3d distance between two points
    /// @param a
    /// @param b
    /// @return 距离值
    double ComputeMotionDis(const Eigen::Vector3d &a, const Eigen::Vector3d &b);

private:
    /// @brief 订阅baselink2odom,即fast_lio的里程计信息
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_baselink2odom_, sub_prediction_;
    bool use_prediction_tf_{false};
    int64_t last_prediction_stamp_{0};

    /// @brief 订阅当前帧点云
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_scan_cur_;

    /// @brief 订阅初始位姿
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initialpose_;

    /// @brief baselink到odom的pose表达
    nav_msgs::msg::Odometry pose_baselink2odom_;

    /// @brief bselink到odom的变换矩阵表达
    Eigen::Matrix4d mat_baselink2odom_;
    /// @brief odom到map的矩阵
    Eigen::Matrix4d mat_odom2map_;
    Eigen::Matrix4d mat_odom2map_kalman_;
    /// @brief baselink到map = mat_odom2map * mat_baselink2odom
    Eigen::Matrix4d mat_baselink2map_;
    /// @brief initialpose初始位姿
    Eigen::Matrix4d mat_initialpose_;

    std::mutex lock_mat_odom2map_;

    /// @brief baselink和运动中心
    Eigen::Matrix4d mat_baselink2motionlink_;

    /// @brief imulink到baselink
    Eigen::Matrix4d mat_imulink2baselink_;

    /// @brief 初始位姿, x, y, z, roll, pitch, yaw (单位:度degrees)
    std::vector<double> initialpose_;

    /// @brief 原始地图点云
    std::shared_ptr<open3d::geometry::PointCloud> pcd_map_ori_;
    std::shared_ptr<open3d::geometry::PointCloud> pcd_map_coarse_;
    std::shared_ptr<open3d::geometry::PointCloud> pcd_map_fine_;
    std::shared_ptr<open3d::geometry::PointCloud> pcd_map_cur_;
    std::shared_ptr<open3d::geometry::PointCloud> pcd_scan_cur_;

    struct ScanFrame {
        std::shared_ptr<const open3d::geometry::PointCloud> cloud;
        int64_t stamp_ns;
    };
    std::deque<ScanFrame> que_pcd_scan_;
    int64_t last_scan_stamp_ns_{0};
    int64_t last_processed_scan_ns_{0};
    uint64_t state_generation_{0};
    int initialization_successes_{0};
    std::shared_ptr<open3d::geometry::PointCloud> registration_submap_;
    Eigen::Vector3d submap_center_ = Eigen::Vector3d::Zero();
    uint64_t submap_generation_{std::numeric_limits<uint64_t>::max()};
    std::string scan_save_directory_;
    uint64_t saved_scan_count_{0};
    // All pose matrices, filter state and health fields are protected by
    // lock_mat_odom2map_. Heavy registration never holds this mutex.
    int64_t last_result_stamp_ns_{0};
    std::chrono::steady_clock::time_point last_success_time_{};
    bool have_success_{false};
    double registration_duration_ms_{0.0};
    double registration_timeout_{6.0};
    double max_input_age_{0.5};
    rclcpp::CallbackGroup::SharedPtr cloud_callback_group_;
    rclcpp::TimerBase::SharedPtr health_timer_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_registration_age_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_registration_duration_;
    int queue_maxsize_;
    double voxelsize_coarse_;
    double voxelsize_fine_;

    /// @brief 定位配准fitness(overlap)阈值
    double threshold_fitness_;
    /// @brief 配准fitness(overlap)阈值
    double threshold_fitness_init_;

    std::thread thread_loc_;
    std::mutex lock_scan_;
    std::mutex lock_exit_;
    std::atomic<bool> flag_exit_{false};
    std::condition_variable work_cv_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_baselink2map_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_baselink2map_kalman_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_motionlink2map_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom2map_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom2map_kalman_;
    rclcpp::Time timestamp_odom_;


    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_scan_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_scan2map_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_submap_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_localization_3d_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_localization_3d_confidence_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_localization_3d_delay_ms_;

    geometry_msgs::msg::PoseStamped localization_3d_;
    std_msgs::msg::Float32 localization_3d_confidence_;
    std_msgs::msg::Float32 localization_3d_delay_ms_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> br_odom2map_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

    bool save_scan_;

    /// @brief 定位频率(定位间隔时间，多少秒1次)
    double loc_frequence_;

    /// @brief source点云最大点数量
    int maxpoints_source_ = 50000;
    /// @brief target点云最大点数量
    int maxpoints_target_ = 200000;

    /// @brief 初始化成功标志
    bool loc_initialized_ = false;

    /// @brief 当前定位overlap，confidence
    double loc_fitness_;

    /// @brief 定位置信度阈值
    double confidence_loc_th_;

    /// 卡尔曼滤波器
    KalmanFilter kf_baselink_x_;
    KalmanFilter kf_baselink_y_;
    KalmanFilter kf_baselink_z_;
    KalmanFilter kalman_filter_odom2map_;

    // 0:kf_processVar 1:kf_estimatedMeasVar
    std::vector<double> kf_param_x_;
    std::vector<double> kf_param_y_;
    std::vector<double> kf_param_z_;

    /// @brief 对odom2map进行kalman滤波
    bool filter_odom2map_ = false;
    double kalman_processVar2_ = 0.0;
    double kalman_estimatedMeasVar2_ = 0.0;

    /// 1202
    /// @brief 上次更新定位时的定位值
    Eigen::Vector3d last_loc_;
    // Eigen::Vector3d cur_loc_;
    /// @brief 更新地图子图的距离,超过则更新地图子图
    double dis_updatemap_;

    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

GloabalLocalization::GloabalLocalization() : Node("global_loc_node"),
                                             tf_buffer_(this->get_clock()),
                                             tf_listener_(std::make_shared<tf2_ros::TransformListener>(tf_buffer_))
{
    flag_exit_ = false;
    loc_initialized_ = false;
    mat_baselink2odom_ = Eigen::Matrix4d::Identity();
    mat_odom2map_ = Eigen::Matrix4d::Identity();
    mat_initialpose_ = Eigen::Matrix4d::Identity();
    mat_baselink2map_ = Eigen::Matrix4d::Identity();
    mat_odom2map_kalman_ = Eigen::Matrix4d::Identity();
    last_loc_ = Eigen::Vector3d(0, 0, -5000);

    pcd_map_ori_.reset(new open3d::geometry::PointCloud);
    pcd_map_coarse_.reset(new open3d::geometry::PointCloud);
    pcd_map_cur_.reset(new open3d::geometry::PointCloud);
    pcd_scan_cur_.reset(new open3d::geometry::PointCloud);
    pcd_map_fine_.reset(new open3d::geometry::PointCloud);
    queue_maxsize_ = 5;

    pub_baselink2map_ = this->create_publisher<nav_msgs::msg::Odometry>("/baselink2map", 5);
    pub_baselink2map_kalman_ = this->create_publisher<nav_msgs::msg::Odometry>("/baselink2map_kalman", 5);
    pub_motionlink2map_ = this->create_publisher<nav_msgs::msg::Odometry>("/motionlink2map", 5);
    pub_odom2map_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom2map", 5);
    pub_odom2map_kalman_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom2map_kalman", 5);

    // The global map is published once during initialization.  Keep the last
    // sample so late subscribers such as RViz can receive it after startup.
    pub_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    pub_submap_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/submap", 1);
    pub_scan2map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/scan2map", 1);
    pub_scan_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/scan", 1);
    pub_localization_3d_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/localization_3d", 1);
    pub_localization_3d_confidence_ = this->create_publisher<std_msgs::msg::Float32>("/localization_3d_confidence", 1);
    pub_localization_3d_delay_ms_ = this->create_publisher<std_msgs::msg::Float32>("/localization_3d_delay_ms", 1);

    loc_frequence_ = 2.0; //
    loc_fitness_ = 0.0;
    // 注册回调函数
    sub_baselink2odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/Odometry_loc", rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&GloabalLocalization::CallbackBaselink2Odom, this, std::placeholders::_1));
    use_prediction_tf_ = declare_parameter<bool>("use_prediction_tf", false);
    if (use_prediction_tf_) sub_prediction_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&GloabalLocalization::CallbackPrediction, this, std::placeholders::_1));
    cloud_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions cloud_options;
    cloud_options.callback_group = cloud_callback_group_;
    sub_scan_cur_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/cloud_registered_1", rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&GloabalLocalization::CallbackScan, this, std::placeholders::_1), cloud_options);
    sub_initialpose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 50, std::bind(&GloabalLocalization::CallbackInitialPose, this, std::placeholders::_1));

    pose_baselink2odom_ = nav_msgs::msg::Odometry();
    pose_baselink2odom_.header.frame_id = "odom";
    pose_baselink2odom_.child_frame_id = "base_link";
    // geometry_msgs的Quaternion会被初始化为0,0,0,0,而不是正确的0,0,0,1
    pose_baselink2odom_.pose.pose.orientation.w = 1;
    RCLCPP_INFO(this->get_logger(), "pose baselink2odom:\nx: %f, y: %f, z: %f, qx: %f, \
                            qy: %f, qz: %f, qw: %f",
                pose_baselink2odom_.pose.pose.position.x,
                pose_baselink2odom_.pose.pose.position.y,
                pose_baselink2odom_.pose.pose.position.z,
                pose_baselink2odom_.pose.pose.orientation.x,
                pose_baselink2odom_.pose.pose.orientation.y,
                pose_baselink2odom_.pose.pose.orientation.z,
                pose_baselink2odom_.pose.pose.orientation.w);

    // 队列最大数量
    this->declare_parameter<int>("pcd_queue_maxsize", 5);
    this->declare_parameter<bool>("save_scan", false);
    /// 最大点数量限制
    this->declare_parameter<int>("maxpoints_source", 50000);
    this->declare_parameter<int>("maxpoints_target", 200000);

    // 定位间隔时间
    this->declare_parameter<double>("loc_frequence", 2.0);

    /// 定位阈值
    this->declare_parameter<double>("confidence_loc_th", 0.6);

    /// 卡尔曼参数
    this->declare_parameter<std::vector<double>>("kf_baselink2map/x", std::vector<double>(2));
    this->declare_parameter<std::vector<double>>("kf_baselink2map/y", std::vector<double>(2));
    this->declare_parameter<std::vector<double>>("kf_baselink2map/z", std::vector<double>(2));

    this->declare_parameter<bool>("filter_odom2map", false);
    this->declare_parameter<double>("kalman_processVar2", 0.02);
    this->declare_parameter<double>("kalman_estimatedMeasVar2", 0.04);
    // voxelsize
    this->declare_parameter<double>("voxelsize_coarse", 0.2);
    this->declare_parameter<double>("voxelsize_fine", 0.05);
    this->declare_parameter<double>("threshold_fitness_init", 0.9);
    this->declare_parameter<double>("threshold_fitness", 0.9);
    this->declare_parameter<std::vector<double>>("initialpose", std::vector<double>());
    this->declare_parameter<double>("dis_updatemap", 1);

    this->get_parameter("pcd_queue_maxsize", queue_maxsize_);
    this->get_parameter("save_scan", save_scan_);
    scan_save_directory_ = declare_parameter<std::string>("scan_save_directory", "/tmp");
    this->get_parameter("maxpoints_source", maxpoints_source_);
    this->get_parameter("maxpoints_target", maxpoints_target_);
    this->get_parameter("loc_frequence", loc_frequence_);
    // Backwards-compatible alias: loc_frequence has always meant seconds, not Hz.
    loc_frequence_ = declare_parameter<double>("loc_update_period", loc_frequence_);
    registration_timeout_ = declare_parameter<double>("registration_timeout", 6.0);
    max_input_age_ = declare_parameter<double>("max_input_age", 0.5);
    if (!std::isfinite(loc_frequence_) || loc_frequence_ <= 0.0 ||
        !std::isfinite(registration_timeout_) || registration_timeout_ <= loc_frequence_ ||
        !std::isfinite(max_input_age_) || max_input_age_ <= 0.0 || queue_maxsize_ < 1) {
        throw std::invalid_argument("Invalid localization timing or cloud queue parameters");
    }
    this->get_parameter("confidence_loc_th", confidence_loc_th_);
    this->get_parameter("kf_baselink2map/x", kf_param_x_);
    this->get_parameter("kf_baselink2map/y", kf_param_y_);
    this->get_parameter("kf_baselink2map/z", kf_param_z_);
    this->get_parameter("filter_odom2map", filter_odom2map_);
    this->get_parameter("kalman_processVar2", kalman_processVar2_);
    this->get_parameter("kalman_estimatedMeasVar2", kalman_estimatedMeasVar2_);

    RCLCPP_INFO(this->get_logger(), "Kalman filter parameters:");
    RCLCPP_INFO(this->get_logger(), "  kf_x: [%.6f, %.6f], size: %zu",
                kf_param_x_.size() >= 1 ? kf_param_x_[0] : 0.0,
                kf_param_x_.size() >= 2 ? kf_param_x_[1] : 0.0,
                kf_param_x_.size());
    RCLCPP_INFO(this->get_logger(), "  kf_y: [%.6f, %.6f], size: %zu",
                kf_param_y_.size() >= 1 ? kf_param_y_[0] : 0.0,
                kf_param_y_.size() >= 2 ? kf_param_y_[1] : 0.0,
                kf_param_y_.size());
    RCLCPP_INFO(this->get_logger(), "  kf_z: [%.6f, %.6f], size: %zu",
                kf_param_z_.size() >= 1 ? kf_param_z_[0] : 0.0,
                kf_param_z_.size() >= 2 ? kf_param_z_[1] : 0.0,
                kf_param_z_.size());
    RCLCPP_INFO(this->get_logger(), "  filter_odom2map: %s", filter_odom2map_ ? "true" : "false");
    this->get_parameter("voxelsize_coarse", voxelsize_coarse_);
    this->get_parameter("voxelsize_fine", voxelsize_fine_);
    this->get_parameter("threshold_fitness_init", threshold_fitness_init_);
    this->get_parameter("threshold_fitness", threshold_fitness_);
    this->get_parameter("initialpose", initialpose_);
    this->get_parameter("dis_updatemap", dis_updatemap_);

    for (auto i : initialpose_)
    {
        std::cout << i << " ";
    }
    std::cout << std::endl;
    mat_initialpose_.block<3, 3>(0, 0) = Euler2Matrix3d(Eigen::Vector3d(initialpose_[3], initialpose_[4], initialpose_[5]));
    mat_initialpose_.block<3, 1>(0, 3) = Eigen::Vector3d(initialpose_[0], initialpose_[1], initialpose_[2]);

    if (kf_param_x_.size() != 2 || kf_param_y_.size() != 2 || kf_param_z_.size() != 2 ||
        maxpoints_source_ < 10 || maxpoints_target_ < 10 || voxelsize_fine_ <= 0.0) {
        throw std::invalid_argument("Invalid localization filter or point-cloud parameters");
    }

    // 读取地图
    std::string path_map = "";
    this->declare_parameter<std::string>("path_map", "");
    this->get_parameter("path_map", path_map);
    open3d::io::ReadPointCloud(path_map, *pcd_map_ori_);
    if (pcd_map_ori_ == nullptr || pcd_map_ori_->IsEmpty())
    {
        RCLCPP_ERROR(this->get_logger(), "read map from path: %s failed", path_map.c_str());
        throw std::runtime_error("Cannot load localization PCD map");
    }

    if (!pcd_map_ori_->HasColors())
    {
        pcd_map_ori_->PaintUniformColor({1, 0, 0});
    }
    // pcd_map_ori_->PaintUniformColor({1, 0, 0});

    pcd_map_coarse_ = pcd_map_ori_->VoxelDownSample(voxelsize_coarse_);
    pcd_map_coarse_->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(voxelsize_coarse_ * 2, 30));

    /// publish map, 用粗地图可视化，减少资源占用
    sensor_msgs::msg::PointCloud2 pc2_map;
    open3d_conversions::open3dToRos(*pcd_map_coarse_, pc2_map);
    pc2_map.header.frame_id = "map";
    pc2_map.header.stamp = this->now();
    pub_map_->publish(pc2_map);

    pcd_map_fine_ = pcd_map_ori_->VoxelDownSample(voxelsize_fine_);
    pcd_map_fine_->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(voxelsize_fine_ * 2, 30));

    if (!GetTfTransformToMatrix("base_link", "imu_link", mat_imulink2baselink_))
        throw std::runtime_error("Missing base_link -> imu_link mounting transform");
    std::cout << "mat_imulink2baselink_:\n"
              << mat_imulink2baselink_ << std::endl;

    if (!GetTfTransformToMatrix("motion_link", "base_link", mat_baselink2motionlink_))
        throw std::runtime_error("Missing motion_link -> base_link transform");
    std::cout << "mat_baselink2motionlink_:\n"
              << mat_baselink2motionlink_ << std::endl;

    RCLCPP_WARN(this->get_logger(), "initialize finished");

    br_odom2map_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    pub_registration_age_ = create_publisher<std_msgs::msg::Float32>(
        "/localization_3d_registration_age_ms", 1);
    pub_registration_duration_ = create_publisher<std_msgs::msg::Float32>(
        "/localization_3d_registration_duration_ms", 1);
    health_timer_ = create_wall_timer(std::chrono::milliseconds(100),
        std::bind(&GloabalLocalization::PublishHealth, this));
    mat_odom2map_ = mat_initialpose_;
    StartLoc();
}

GloabalLocalization::~GloabalLocalization()
{
    flag_exit_.store(true);
    work_cv_.notify_all();
    if (thread_loc_.joinable())
    {
        thread_loc_.join();
    }
}

Eigen::Matrix3d GloabalLocalization::Euler2Matrix3d(const Eigen::Vector3d euler)
{
    Eigen::Matrix3d mat3d;
    // convert degrees to radians
    auto eulerAngle = euler / 180 * M_PI;
    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle[0], Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle[1], Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle[2], Eigen::Vector3d::UnitZ()));
    mat3d = rollAngle * pitchAngle * yawAngle;
    return mat3d;
}
bool GloabalLocalization::GetTfTransformToMatrix(std::string frame_id, std::string child_frame_id, Eigen::Matrix4d &matrix)
{
    // 获取pose
    geometry_msgs::msg::TransformStamped pose_;
    try
    {
        pose_ = tf_buffer_.lookupTransform(frame_id, child_frame_id, rclcpp::Time(0), rclcpp::Duration::from_seconds(5.0));
    }
    catch (tf2::TransformException &e)
    {
        RCLCPP_ERROR(this->get_logger(), "[GetTransformMatrix]: %s", e.what());
        return false;
    }

    Eigen::Vector3d translation = Eigen::Vector3d(pose_.transform.translation.x, pose_.transform.translation.y, pose_.transform.translation.z);
    Eigen::Quaterniond quat = Eigen::Quaterniond::Identity();

    quat = Eigen::Quaterniond(pose_.transform.rotation.w,
                              pose_.transform.rotation.x,
                              pose_.transform.rotation.y,
                              pose_.transform.rotation.z);
    Eigen::Matrix3d rotation = quat.matrix();

    matrix = Eigen::Matrix4d::Identity();
    matrix.block<3, 3>(0, 0) = rotation;
    matrix.matrix().block<3, 1>(0, 3) = translation;
    return true;
}

// Global correction is held between accepted registrations. Its freshness is
// still checked independently by PublishHealth; high-rate TF does not renew it.
void GloabalLocalization::CallbackPrediction(const nav_msgs::msg::Odometry::SharedPtr message)
{
    const rclcpp::Time stamp(message->header.stamp);
    const double age=(now()-stamp).seconds();
    if (message->header.frame_id!="odom" || message->child_frame_id!="base_link" ||
        age<-.005 || age>.1 || stamp.nanoseconds()<=last_prediction_stamp_) return;
    std::lock_guard<std::mutex> guard(lock_mat_odom2map_);
    if (!loc_initialized_ || !have_success_ ||
        (now().nanoseconds()-last_result_stamp_ns_)*1e-9>registration_timeout_ ||
        std::chrono::duration<double>(std::chrono::steady_clock::now()-last_success_time_).count()>registration_timeout_ ||
        (now()-timestamp_odom_).seconds()>max_input_age_) return;
    last_prediction_stamp_=stamp.nanoseconds();
    Eigen::Isometry3d correction=Eigen::Isometry3d::Identity();
    correction.matrix()=mat_odom2map_;
    auto transform=tf2::eigenToTransform(correction);
    transform.header.stamp=message->header.stamp;
    transform.header.frame_id="map";transform.child_frame_id="odom";
    br_odom2map_->sendTransform(transform);
}

void GloabalLocalization::CallbackBaselink2Odom(const nav_msgs::msg::Odometry::SharedPtr baselink2odom)
{
    const rclcpp::Time stamp(baselink2odom->header.stamp);
    const double age = (now() - stamp).seconds();
    const auto & pose = baselink2odom->pose.pose;
    const auto & q = pose.orientation;
    const double qnorm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    if (stamp.nanoseconds() <= 0 || age < -0.05 || age > max_input_age_ ||
        !std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
        !std::isfinite(pose.position.z) || !std::isfinite(qnorm) ||
        std::abs(qnorm - 1.0) > 0.01) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "Rejecting invalid/stale localization odometry (age %.3f s)", age);
        return;
    }
    std::lock_guard<std::mutex> state_guard(lock_mat_odom2map_);
    if (stamp.nanoseconds() <= timestamp_odom_.nanoseconds()) return;
    timestamp_odom_ = stamp;
    Eigen::Isometry3d mat_current = Eigen::Isometry3d::Identity();
    tf2::fromMsg(baselink2odom->pose.pose, mat_current);
    auto mat_imulink2odom = mat_current.matrix();

    mat_baselink2odom_ = mat_imulink2odom * mat_imulink2baselink_.inverse();

    Eigen::Isometry3d Isometry3d_baselink2map;
    mat_baselink2map_ = mat_odom2map_ * mat_baselink2odom_;
    Isometry3d_baselink2map.matrix() = mat_baselink2map_;
    nav_msgs::msg::Odometry baselink2map;
    baselink2map.pose.pose = tf2::toMsg(Isometry3d_baselink2map);
    baselink2map.header.frame_id = "map";
    baselink2map.child_frame_id = "base_link";
    baselink2map.header.stamp = baselink2odom->header.stamp;
    pub_baselink2map_->publish(baselink2map);

    Eigen::Isometry3d Isometry3d_odom2map;
    Isometry3d_odom2map.matrix() = mat_odom2map_;
    nav_msgs::msg::Odometry odom2map;
    odom2map.pose.pose = tf2::toMsg(Isometry3d_odom2map);
    odom2map.header.frame_id = "map";
    odom2map.child_frame_id = "odom";
    odom2map.header.stamp = baselink2odom->header.stamp;
    pub_odom2map_->publish(odom2map);

    /// 发布tf关系
    geometry_msgs::msg::TransformStamped transform_odom2map;
    transform_odom2map.header.frame_id = "map";
    transform_odom2map.child_frame_id = "odom";
    transform_odom2map.header.stamp = baselink2odom->header.stamp;
    transform_odom2map.transform.translation.x = odom2map.pose.pose.position.x;
    transform_odom2map.transform.translation.y = odom2map.pose.pose.position.y;
    transform_odom2map.transform.translation.z = odom2map.pose.pose.position.z;
    transform_odom2map.transform.rotation = odom2map.pose.pose.orientation;
    if (!use_prediction_tf_) br_odom2map_->sendTransform(transform_odom2map);

    /// 卡尔曼滤波 - 只在定位初始化完成后执行
    if (loc_initialized_)
    {
        Eigen::Matrix4d mat_baselink2map_kalman = Eigen::Matrix4d::Identity();

        if (filter_odom2map_)
        {
            Eigen::Isometry3d Isometry3d_odom2map_kalman;
            Isometry3d_odom2map_kalman.matrix() = mat_odom2map_kalman_;
            nav_msgs::msg::Odometry odom2map_kalman;
            odom2map_kalman.pose.pose = tf2::toMsg(Isometry3d_odom2map_kalman);
            odom2map_kalman.header.frame_id = "map";
            odom2map_kalman.child_frame_id = "odom_kalman";
            odom2map_kalman.header.stamp = baselink2odom->header.stamp;
            pub_odom2map_kalman_->publish(odom2map_kalman);

            kf_baselink_z_.inputLatestNoisyMeasurement((mat_odom2map_kalman_ * mat_baselink2odom_)(2, 3));
            mat_baselink2map_kalman = mat_odom2map_kalman_ * mat_baselink2odom_;
        }
        else
        {
            double input_x = mat_baselink2map_(0, 3);
            double input_y = mat_baselink2map_(1, 3);
            double input_z = mat_baselink2map_(2, 3);

            kf_baselink_x_.inputLatestNoisyMeasurement(input_x);
            kf_baselink_y_.inputLatestNoisyMeasurement(input_y);
            kf_baselink_z_.inputLatestNoisyMeasurement(input_z);
            mat_baselink2map_kalman = mat_baselink2map_;

            RCLCPP_DEBUG(this->get_logger(), "KF input: x=%.3f, y=%.3f, z=%.3f", input_x, input_y, input_z);
        }

        double filtered_z = kf_baselink_z_.getLatestEstimatedMeasurement();

        // 验证结果是否有效（检查 NaN）
        if (std::isnan(filtered_z))
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Kalman filter returned NaN (input was: %.3f), using unfiltered value",
                                 mat_baselink2map_kalman(2, 3));
            mat_baselink2map_kalman(2, 3) = mat_baselink2map_(2, 3);
        }
        else
        {
            mat_baselink2map_kalman(2, 3) = filtered_z;
        }
        Eigen::Isometry3d Isometry3d_baselink2map_kalman;
        Isometry3d_baselink2map_kalman.matrix() = mat_baselink2map_kalman;
        nav_msgs::msg::Odometry baselink2map_kalman;
        baselink2map_kalman.pose.pose = tf2::toMsg(Isometry3d_baselink2map_kalman);
        baselink2map_kalman.header.frame_id = "map";
        // baselink2map_kalman.child_frame_id = "base_link_kalman";
        baselink2map_kalman.header.stamp = baselink2odom->header.stamp;
        pub_baselink2map_kalman_->publish(baselink2map_kalman);

        Eigen::Matrix4d mat_motionlink2map = mat_baselink2map_kalman * mat_baselink2motionlink_.inverse();
        Eigen::Isometry3d Isometry3d_motionlink2map;
        Isometry3d_motionlink2map.matrix() = mat_motionlink2map;
        nav_msgs::msg::Odometry motionlink2map;
        motionlink2map.pose.pose = tf2::toMsg(Isometry3d_motionlink2map);
        motionlink2map.header.frame_id = "map";
        // baselink2map_kalman.child_frame_id = "base_link_kalman";
        motionlink2map.header.stamp = baselink2odom->header.stamp;
        pub_motionlink2map_->publish(motionlink2map);

        // Do not publish map -> motion_link here.  motion_link is already a
        // child of base_link, and publishing it from map as well gives the TF
        // frame two parents.  The map -> odom -> base_link chain above carries
        // the global localization transform; keep /motionlink2map as the
        // optional pose output for consumers that need it.

        localization_3d_delay_ms_.data = (this->now() - baselink2odom->header.stamp).seconds() * 1000.0;
        pub_localization_3d_delay_ms_->publish(localization_3d_delay_ms_);
        localization_3d_.header.frame_id = "map";
        localization_3d_.header.stamp = baselink2odom->header.stamp;
        localization_3d_.pose = motionlink2map.pose.pose;
        pub_localization_3d_->publish(localization_3d_);
    }
}
void GloabalLocalization::CallbackScan(
    const sensor_msgs::msg::PointCloud2::SharedPtr message)
{
    const int64_t stamp = rclcpp::Time(message->header.stamp).nanoseconds();
    const double age = (now().nanoseconds() - stamp) * 1e-9;
    // /cloud_registered is expressed in FAST-LIO's fixed odometry frame.
    if (stamp <= 0 || age < -0.05 || age > max_input_age_ ||
        message->header.frame_id != "camera_init" || message->width * message->height == 0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "Rejecting stale/invalid localization cloud (age %.3f s, frame %s)",
            age, message->header.frame_id.c_str());
        return;
    }
    // Conversion is isolated from the odometry callback group. Concatenation
    // and ICP run in the worker, not in either subscription callback.
    auto cloud = std::make_shared<open3d::geometry::PointCloud>();
    sensor_msgs::msg::PointCloud2::ConstSharedPtr input = message;
    open3d_conversions::rosToOpen3d(input, *cloud);
    cloud->RemoveNonFinitePoints();
    if (cloud->IsEmpty()) return;
    std::lock_guard<std::mutex> guard(lock_scan_);
    if (stamp <= last_scan_stamp_ns_) return;
    last_scan_stamp_ns_ = stamp;
    que_pcd_scan_.push_back({cloud, stamp});
    while (que_pcd_scan_.size() > static_cast<size_t>(queue_maxsize_))
        que_pcd_scan_.pop_front();
}

bool GloabalLocalization::WaitForWork(double seconds)
{
    std::unique_lock<std::mutex> lock(lock_exit_);
    work_cv_.wait_for(lock, std::chrono::duration<double>(seconds),
        [this] { return flag_exit_.load(); });
    return !flag_exit_.load() && rclcpp::ok();
}

void GloabalLocalization::PublishHealth()
{
    const auto steady_now = std::chrono::steady_clock::now();
    const int64_t ros_now = now().nanoseconds();
    std_msgs::msg::Float32 confidence, age, duration;
    {
        std::lock_guard<std::mutex> guard(lock_mat_odom2map_);
        const double data_age = (ros_now - last_result_stamp_ns_) * 1e-9;
        const double success_age = std::chrono::duration<double>(steady_now - last_success_time_).count();
        const double odom_age = (ros_now - timestamp_odom_.nanoseconds()) * 1e-9;
        const bool fresh = loc_initialized_ && have_success_ &&
            last_result_stamp_ns_ > 0 && data_age >= -0.05 &&
            data_age <= registration_timeout_ && success_age <= registration_timeout_ &&
            odom_age >= -0.05 && odom_age <= max_input_age_;
        confidence.data = fresh ? static_cast<float>(loc_fitness_) : 0.0f;
        age.data = last_result_stamp_ns_ > 0 ? static_cast<float>(data_age * 1000.0) : -1.0f;
        duration.data = static_cast<float>(registration_duration_ms_);
    }
    // This heartbeat represents validity of the last actual registration.
    // New odometry alone cannot renew it. The motion bridge's heartbeat
    // timeout stays short; expired registration produces confidence zero.
    pub_localization_3d_confidence_->publish(confidence);
    pub_registration_age_->publish(age);
    pub_registration_duration_->publish(duration);
}

bool GloabalLocalization::RegisterScan(bool initializing)
{
    std::deque<ScanFrame> scans;
    {
        std::lock_guard<std::mutex> guard(lock_scan_);
        if (que_pcd_scan_.empty()) return false;
        scans = que_pcd_scan_; // immutable cloud pointers; cheap snapshot
    }
    const int64_t scan_stamp = scans.back().stamp_ns;
    if (scan_stamp <= last_processed_scan_ns_) return false;
    const double age = (now().nanoseconds() - scan_stamp) * 1e-9;
    if (age < -0.05 || age > max_input_age_) return false;

    Eigen::Matrix4d odom_T_base, map_T_odom;
    uint64_t generation;
    {
        std::lock_guard<std::mutex> guard(lock_mat_odom2map_);
        const double odom_age = (now().nanoseconds() - timestamp_odom_.nanoseconds()) * 1e-9;
        if (timestamp_odom_.nanoseconds() <= 0 || odom_age < -0.05 || odom_age > max_input_age_)
            return false;
        initializing = !loc_initialized_;
        odom_T_base = mat_baselink2odom_;
        map_T_odom = mat_odom2map_;
        generation = state_generation_;
    }
    last_processed_scan_ns_ = scan_stamp;
    const auto started = std::chrono::steady_clock::now();
    auto source = std::make_shared<open3d::geometry::PointCloud>();
    for (const auto & frame : scans) *source += *frame.cloud;
    const Eigen::Matrix4d map_T_base = map_T_odom * odom_T_base;
    open3d::geometry::OrientedBoundingBox map_box, scan_box;
    map_box.extent_ = scan_box.extent_ = Eigen::Vector3d(60, 60, 40);
    map_box.center_ = map_T_base.block<3, 1>(0, 3);
    map_box.R_ = map_T_base.block<3, 3>(0, 0);
    scan_box.center_ = odom_T_base.block<3, 1>(0, 3);
    scan_box.R_ = odom_T_base.block<3, 3>(0, 0);
    if (!registration_submap_ || generation != submap_generation_ || initializing ||
        (map_box.center_ - submap_center_).norm() > dis_updatemap_) {
        registration_submap_ = pcd_map_fine_->Crop(map_box);
        submap_center_ = map_box.center_;
        submap_generation_ = generation;
    }
    auto target = registration_submap_;
    source = source->Crop(scan_box)->VoxelDownSample(voxelsize_fine_);
    if (target->points_.size() > static_cast<size_t>(maxpoints_target_))
        target = target->RandomDownSample(double(maxpoints_target_) / target->points_.size());
    if (source->points_.size() > static_cast<size_t>(maxpoints_source_))
        source = source->RandomDownSample(double(maxpoints_source_) / source->points_.size());
    double fitness = 0.0;
    Eigen::Matrix4d result = map_T_odom;
    if (source->points_.size() >= 10 && target->points_.size() >= 10) {
        if (initializing) {
            source->Transform(map_T_odom);
            const auto correction = pcd_tools::RegistrationMultiScaleIcp(
                source, target, voxelsize_fine_, 1, {1, 2, 3});
            result = correction * map_T_odom;
            fitness = open3d::pipelines::registration::EvaluateRegistration(
                *source, *target, voxelsize_fine_ * 3, correction).fitness_;
        } else {
            const auto registration = pcd_tools::RegistrationIcp(
                source, target, voxelsize_fine_ * 2, map_T_odom, 1);
            result = registration.transformation_ * map_T_odom;
            fitness = open3d::pipelines::registration::EvaluateRegistration(
                *source, *target, voxelsize_fine_ * 4, result).fitness_;
        }
    }
    const auto finished = std::chrono::steady_clock::now();
    const double duration_ms = std::chrono::duration<double, std::milli>(finished - started).count();
    {
        std::lock_guard<std::mutex> guard(lock_mat_odom2map_);
        // An initial-pose reset supersedes any registration already in flight.
        if (generation != state_generation_) return false;
        registration_duration_ms_ = duration_ms;
        const double result_age = (now().nanoseconds() - scan_stamp) * 1e-9;
        const bool accepted = result.allFinite() && std::isfinite(fitness) &&
            fitness > (initializing ? threshold_fitness_init_ : threshold_fitness_) &&
            result_age >= -0.05 && result_age <= registration_timeout_;
        loc_fitness_ = accepted ? fitness : 0.0;
        if (accepted) {
            mat_odom2map_ = result;
            last_result_stamp_ns_ = scan_stamp;
            last_success_time_ = finished;
            have_success_ = true;
            if (initializing && ++initialization_successes_ >= 2) {
                const Eigen::Matrix4d initial = result * mat_baselink2odom_;
                kf_baselink_x_.KalmanFilterInit(kf_param_x_[0], kf_param_x_[1], initial(0, 3), 1);
                kf_baselink_y_.KalmanFilterInit(kf_param_y_[0], kf_param_y_[1], initial(1, 3), 1);
                kf_baselink_z_.KalmanFilterInit(kf_param_z_[0], kf_param_z_[1], initial(2, 3), 1);
                kalman_filter_odom2map_.KalmanFilterInit(
                    kalman_processVar2_, kalman_estimatedMeasVar2_, result(2, 3), 1);
                loc_initialized_ = true;
            }
            mat_odom2map_kalman_ = result;
            if (filter_odom2map_ && loc_initialized_) {
                kalman_filter_odom2map_.inputLatestNoisyMeasurement(result(2, 3));
                mat_odom2map_kalman_(2, 3) = kalman_filter_odom2map_.getLatestEstimatedMeasurement();
            }
        } else if (initializing) {
            initialization_successes_ = 0;
        }
    }
    if (save_scan_) {
        // Optional debugging output stays in the worker. Directory must exist.
        const auto path = scan_save_directory_ + "/localization_" +
            std::to_string(saved_scan_count_++) + ".ply";
        if (!open3d::io::WritePointCloud(path, *source))
            RCLCPP_WARN(get_logger(), "Unable to save scan: %s", path.c_str());
    }
    RCLCPP_INFO(get_logger(), "Registration: %.1f ms, fitness %.3f, initializing %s",
        duration_ms, fitness, initializing ? "true" : "false");
    return true;
}

void GloabalLocalization::Localization()
{
    while (rclcpp::ok() && !flag_exit_.load()) {
        bool initializing;
        {
            std::lock_guard<std::mutex> guard(lock_mat_odom2map_);
            initializing = !loc_initialized_;
        }
        const auto started = std::chrono::steady_clock::now();
        bool processed = false;
        try {
            processed = RegisterScan(initializing);
        } catch (const std::exception & error) {
            {
                std::lock_guard<std::mutex> guard(lock_mat_odom2map_);
                loc_fitness_ = 0.0;
                initialization_successes_ = 0;
            }
            RCLCPP_ERROR(get_logger(), "Registration failed: %s", error.what());
        }
        const double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - started).count();
        // No busy spin on unchanged odometry or a missing cloud. Initializing
        // requires two different scans; steady-state scheduling uses seconds.
        const double period = processed && !initializing ? loc_frequence_ : 0.1;
        if (!WaitForWork(std::max(0.02, period - elapsed))) break;
    }
}

void GloabalLocalization::StartLoc()
{
    thread_loc_ = std::thread(&GloabalLocalization::Localization, this);
}

void GloabalLocalization::CallbackInitialPose(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr initialpose)
{
    const auto & p = initialpose->pose.pose;
    Eigen::Quaterniond q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
    if (initialpose->header.frame_id != "map" || !q.coeffs().allFinite() || q.norm() < 1e-6 ||
        !std::isfinite(p.position.x) || !std::isfinite(p.position.y) || !std::isfinite(p.position.z)) {
        RCLCPP_WARN(get_logger(), "Ignoring invalid initial pose; expected a finite pose in map");
        return;
    }
    Eigen::Matrix4d map_T_base = Eigen::Matrix4d::Identity();
    map_T_base.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
    map_T_base.block<3, 1>(0, 3) = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
    std::lock_guard<std::mutex> guard(lock_mat_odom2map_);
    const double odom_age = (now().nanoseconds() - timestamp_odom_.nanoseconds()) * 1e-9;
    if (timestamp_odom_.nanoseconds() <= 0 || odom_age < -0.05 || odom_age > max_input_age_) {
        RCLCPP_WARN(get_logger(), "Initial pose requires fresh odometry");
        return;
    }
    mat_initialpose_ = map_T_base * mat_baselink2odom_.inverse();
    mat_odom2map_ = mat_initialpose_;
    ++state_generation_;
    loc_initialized_ = false;
    initialization_successes_ = 0;
    loc_fitness_ = 0.0;
    have_success_ = false;
    last_result_stamp_ns_ = 0;
    RCLCPP_INFO(get_logger(), "Initial pose reset; waiting for two fresh registration results");
}

double GloabalLocalization::ComputeMotionDis(const Eigen::Vector3d &a, const Eigen::Vector3d &b)
{
    return std::sqrt(std::pow(a.x() - b.x(), 2) + std::pow(a.y() - b.y(), 2) + std::pow(a.z() - b.z(), 2));
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GloabalLocalization>();

    // 使用多线程执行器，可以指定线程数
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}

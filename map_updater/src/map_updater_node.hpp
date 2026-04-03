#pragma once

#include "map_updater_core.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "interface/srv/save_maps.hpp"

struct MapUpdaterConfig
{
    std::string initial_pcd_path;
    std::string map_frame = "camera_init";

    std::string scan_topic = "/fastlio2/world_cloud";
    std::string odom_topic = "/fastlio2/lio_odom";
    std::string scan_frame = "camera_init";  // frame of scan_topic & odom_topic
    double scan_process_interval_s = 0.5;

    double map_resolution = 0.2;
    double scan_resolution = 0.15;
    double removal_resolution = 0.05;   // VoxelHashMap独立分辨率，细于ikd-tree防止voxel重叠

    double publish_rate_hz = 1.0;
    double diag_interval_s = 10.0;
    double diag_sector_radius_m = 6.0;
    double diag_sector_fov_deg = 90.0;

    bool enable_point_removal = false;
    int ray_cast_skip = 10;

    // Log-odds occupancy model (OctoMap-inspired)
    float log_odds_hit  = 0.847f;
    float log_odds_miss = 0.405f;
    float log_odds_clamp_max = 3.5f;
    float log_odds_clamp_min = -2.0f;
    float log_odds_free_threshold = -1.0f;
    float log_odds_initial = 2.0f;

    double endpoint_protection_base         = 0.1;
    double endpoint_protection_angle_factor = 0.003;
};

class MapUpdaterNode : public rclcpp::Node
{
public:
    explicit MapUpdaterNode(const rclcpp::NodeOptions& options);
    ~MapUpdaterNode() override;

private:
    void loadConfig();
    void loadInitialMap();

    void onLidarScan(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);

    void processLoop();
    void doIncrementalAdd(PointVec& points);
    void doPointRemoval(const Eigen::Vector3f& origin,
                        const Eigen::Vector3f& forward,
                        const PointVec& endpoints);

    void publishUpdatedMap();
    void logDiagnostics();

    void handleSaveMap(
        const std::shared_ptr<interface::srv::SaveMaps::Request> req,
        std::shared_ptr<interface::srv::SaveMaps::Response> resp);
    void handleReloadMap(
        const std::shared_ptr<interface::srv::SaveMaps::Request> req,
        std::shared_ptr<interface::srv::SaveMaps::Response> resp);

    MapUpdaterConfig config_;
    std::unique_ptr<MapUpdaterCore> core_;

    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::Service<interface::srv::SaveMaps>::SharedPtr save_srv_;
    rclcpp::Service<interface::srv::SaveMaps>::SharedPtr reload_srv_;

    // TF (for scan_frame → map_frame transform when they differ)
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    bool need_tf_transform_ = false;  // true when scan_frame != map_frame

    // Odometry cache (protected by odom_mutex_)
    std::mutex odom_mutex_;
    Eigen::Vector3f latest_sensor_origin_{Eigen::Vector3f::Zero()};
    Eigen::Quaternionf latest_sensor_orientation_{Eigen::Quaternionf::Identity()};
    bool has_odom_ = false;

    // Threading
    std::thread process_thread_;
    std::atomic<bool> running_{true};
    std::mutex data_mutex_;
    std::condition_variable data_cv_;

    // Pending data (protected by data_mutex_)
    PointVec pending_points_;
    Eigen::Vector3f pending_sensor_origin_{Eigen::Vector3f::Zero()};
    Eigen::Vector3f pending_sensor_forward_{Eigen::Vector3f::UnitX()};
    bool new_scan_available_ = false;

    // Service commands (protected by data_mutex_)
    enum class ServiceCmd
    {
        NONE,
        SAVE,
        RELOAD
    };
    ServiceCmd pending_cmd_ = ServiceCmd::NONE;
    std::string pending_cmd_path_;
    bool pending_cmd_result_ = false;
    std::condition_variable cmd_done_cv_;

    // Timing
    using Clock = std::chrono::steady_clock;
    Clock::time_point last_scan_time_;
    Clock::time_point last_publish_time_;
    Clock::time_point last_diag_time_;

    // Diagnostics (atomic: written in callback thread, read in process thread)
    std::atomic<uint64_t> scans_received_{0};
    std::atomic<uint64_t> scans_processed_{0};
    std::atomic<uint64_t> scans_dropped_no_odom_{0};
    std::atomic<uint64_t> points_added_total_{0};
    std::atomic<uint64_t> points_removed_total_{0};
};

#include "map_updater_node.hpp"

#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <yaml-cpp/yaml.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <filesystem>
#include <functional>

MapUpdaterNode::MapUpdaterNode(const rclcpp::NodeOptions& options)
    : Node("map_updater_node", options)
{
    loadConfig();

    CoreConfig core_cfg;
    core_cfg.map_resolution                    = config_.map_resolution;
    core_cfg.scan_resolution                   = config_.scan_resolution;
    core_cfg.removal_resolution                = config_.removal_resolution;
    core_cfg.log_odds.log_odds_hit             = config_.log_odds_hit;
    core_cfg.log_odds.log_odds_miss            = config_.log_odds_miss;
    core_cfg.log_odds.clamp_max                = config_.log_odds_clamp_max;
    core_cfg.log_odds.clamp_min                = config_.log_odds_clamp_min;
    core_cfg.log_odds.free_threshold           = config_.log_odds_free_threshold;
    core_cfg.log_odds.initial_log_odds         = config_.log_odds_initial;
    core_cfg.endpoint_protection_base          = config_.endpoint_protection_base;
    core_cfg.endpoint_protection_angle_factor  = config_.endpoint_protection_angle_factor;
    core_ = std::make_unique<MapUpdaterCore>(core_cfg);

    loadInitialMap();

    // Initialize TF if scan_frame != map_frame
    need_tf_transform_ = (config_.scan_frame != config_.map_frame);
    if (need_tf_transform_)
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        RCLCPP_INFO(get_logger(),
                    "TF transform enabled: %s -> %s",
                    config_.scan_frame.c_str(), config_.map_frame.c_str());
    }

    map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "updated_map", rclcpp::QoS(1).transient_local());

    scan_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        config_.scan_topic, rclcpp::QoS(5),
        std::bind(&MapUpdaterNode::onLidarScan, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        config_.odom_topic, rclcpp::QoS(5),
        std::bind(&MapUpdaterNode::onOdometry, this, std::placeholders::_1));

    save_srv_ = create_service<interface::srv::SaveMaps>(
        "save_map",
        std::bind(&MapUpdaterNode::handleSaveMap, this,
                  std::placeholders::_1, std::placeholders::_2));
    reload_srv_ = create_service<interface::srv::SaveMaps>(
        "reload_map",
        std::bind(&MapUpdaterNode::handleReloadMap, this,
                  std::placeholders::_1, std::placeholders::_2));

    auto now = Clock::now();
    last_scan_time_ = now;
    last_publish_time_ = now;
    last_diag_time_ = now;

    process_thread_ = std::thread(&MapUpdaterNode::processLoop, this);

    RCLCPP_INFO(get_logger(),
                "MapUpdaterNode started. scan_topic=%s odom_topic=%s "
                "scan_frame=%s map_frame=%s "
                "map_resolution=%.2f removal_resolution=%.3f publish_rate=%.1fHz",
                config_.scan_topic.c_str(), config_.odom_topic.c_str(),
                config_.scan_frame.c_str(), config_.map_frame.c_str(),
                config_.map_resolution, config_.removal_resolution,
                config_.publish_rate_hz);
}

MapUpdaterNode::~MapUpdaterNode()
{
    running_ = false;
    data_cv_.notify_all();
    if (process_thread_.joinable())
    {
        process_thread_.join();
    }
}

void MapUpdaterNode::loadConfig()
{
    declare_parameter("config_path", "");
    std::string config_path;
    get_parameter("config_path", config_path);

    if (config_path.empty() || !std::filesystem::exists(config_path))
    {
        RCLCPP_WARN(get_logger(),
                    "No config_path or file not found (%s), using defaults",
                    config_path.c_str());
        return;
    }

    YAML::Node cfg = YAML::LoadFile(config_path);

    auto get = [&](const std::string& key, auto default_val)
    {
        using T = decltype(default_val);
        if (cfg[key])
        {
            return cfg[key].as<T>();
        }
        return default_val;
    };

    config_.initial_pcd_path = get("initial_pcd_path", std::string(""));
    config_.map_frame = get("map_frame", std::string("camera_init"));
    config_.scan_topic =
        get("scan_topic", std::string("/fastlio2/world_cloud"));
    config_.odom_topic =
        get("odom_topic", std::string("/fastlio2/lio_odom"));
    config_.scan_frame =
        get("scan_frame", std::string("camera_init"));
    config_.scan_process_interval_s = get("scan_process_interval_s", 0.5);
    config_.map_resolution = get("map_resolution", 0.2);
    config_.scan_resolution = get("scan_resolution", 0.15);
    config_.removal_resolution = get("removal_voxel_resolution", 0.05);
    config_.publish_rate_hz = get("publish_rate_hz", 1.0);
    config_.diag_interval_s = get("diag_interval_s", 10.0);
    config_.diag_sector_radius_m = get("diag_sector_radius_m", 6.0);
    config_.diag_sector_fov_deg = get("diag_sector_fov_deg", 90.0);
    config_.enable_point_removal = get("enable_point_removal", false);
    config_.ray_cast_skip           = get("ray_cast_skip", 10);
    config_.log_odds_hit            = get("log_odds_hit", 0.847f);
    config_.log_odds_miss           = get("log_odds_miss", 0.405f);
    config_.log_odds_clamp_max      = get("log_odds_clamp_max", 3.5f);
    config_.log_odds_clamp_min      = get("log_odds_clamp_min", -2.0f);
    config_.log_odds_free_threshold = get("log_odds_free_threshold", -1.0f);
    config_.log_odds_initial        = get("log_odds_initial", 2.0f);
    config_.endpoint_protection_base         = get("endpoint_protection_base_m", 0.1);
    config_.endpoint_protection_angle_factor = get("endpoint_protection_angle_factor", 0.003);

    RCLCPP_INFO(get_logger(), "Config loaded from %s", config_path.c_str());
}

void MapUpdaterNode::loadInitialMap()
{
    if (config_.initial_pcd_path.empty())
    {
        RCLCPP_INFO(get_logger(), "No initial PCD configured, starting empty");
        return;
    }
    if (!std::filesystem::exists(config_.initial_pcd_path))
    {
        RCLCPP_WARN(get_logger(), "Initial PCD not found: %s",
                    config_.initial_pcd_path.c_str());
        return;
    }

    if (core_->loadPCD(config_.initial_pcd_path))
    {
        RCLCPP_INFO(get_logger(), "Loaded initial map: %d points",
                    core_->validNum());
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "Failed to load initial PCD: %s",
                     config_.initial_pcd_path.c_str());
    }
}

void MapUpdaterNode::onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(odom_mutex_);
    latest_sensor_origin_ = Eigen::Vector3f(
        static_cast<float>(msg->pose.pose.position.x),
        static_cast<float>(msg->pose.pose.position.y),
        static_cast<float>(msg->pose.pose.position.z));
    latest_sensor_orientation_ = Eigen::Quaternionf(
        static_cast<float>(msg->pose.pose.orientation.w),
        static_cast<float>(msg->pose.pose.orientation.x),
        static_cast<float>(msg->pose.pose.orientation.y),
        static_cast<float>(msg->pose.pose.orientation.z));
    has_odom_ = true;
}

void MapUpdaterNode::onLidarScan(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    scans_received_++;

    auto now = Clock::now();
    double elapsed =
        std::chrono::duration<double>(now - last_scan_time_).count();
    if (elapsed < config_.scan_process_interval_s)
    {
        return;
    }

    // Get sensor origin and orientation from cached odometry.
    // lio_odom publishes t_wi (IMU position in world frame).
    // For mid-360 the IMU-LiDAR offset is ~5cm, negligible for ray-casting.
    Eigen::Vector3f sensor_origin;
    Eigen::Quaternionf sensor_orientation;
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        if (!has_odom_)
        {
            scans_dropped_no_odom_++;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "No odometry received yet, dropping scan");
            return;
        }
        sensor_origin = latest_sensor_origin_;
        sensor_orientation = latest_sensor_orientation_;
    }

    // world_cloud is PointXYZINormal in camera_init frame.
    // Parse directly as PointXYZI — pcl::fromROSMsg maps fields by name,
    // extra fields (normal_x/y/z, curvature) are ignored.
    CloudType::Ptr cloud_raw(new CloudType);
    pcl::fromROSMsg(*msg, *cloud_raw);
    if (cloud_raw->empty())
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Received empty scan");
        return;
    }

    // Transform cloud and sensor origin from scan_frame to map_frame if needed.
    if (need_tf_transform_)
    {
        geometry_msgs::msg::TransformStamped tf_stamped;
        try
        {
            tf_stamped = tf_buffer_->lookupTransform(
                config_.map_frame, config_.scan_frame,
                tf2::TimePointZero);
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "TF %s->%s not available: %s",
                                 config_.scan_frame.c_str(),
                                 config_.map_frame.c_str(), ex.what());
            return;
        }

        Eigen::Isometry3d tf_eigen =
            tf2::transformToEigen(tf_stamped.transform);
        Eigen::Affine3f tf_f = tf_eigen.cast<float>();

        pcl::transformPointCloud(*cloud_raw, *cloud_raw, tf_f);

        // Transform sensor origin and orientation to map_frame.
        sensor_origin = tf_f * sensor_origin;
        Eigen::Quaternionf tf_rot(tf_f.rotation());
        sensor_orientation = tf_rot * sensor_orientation;
    }

    // Optional pre-filter (VoxelGrid downsampling).
    // world_cloud is already filtered by livox2PCL's filter_num,
    // but not by scan_resolution VoxelGrid.
    CloudType::Ptr cloud_filtered = core_->preFilterScan(cloud_raw);

    // Points are now in map_frame (transformed above if scan_frame != map_frame).
    PointVec map_points;
    map_points.reserve(cloud_filtered->size());
    for (const auto& pt : cloud_filtered->points)
    {
        PointType mp;
        mp.x = pt.x;
        mp.y = pt.y;
        mp.z = pt.z;
        mp.intensity = pt.intensity;
        map_points.push_back(mp);
    }

    // Compute sensor forward direction from odometry orientation.
    Eigen::Vector3f sensor_forward =
        sensor_orientation * Eigen::Vector3f::UnitX();
    if (sensor_forward.squaredNorm() < 1e-6f)
    {
        sensor_forward = Eigen::Vector3f::UnitX();
    }
    else
    {
        sensor_forward.normalize();
    }

    RCLCPP_INFO(get_logger(),
                "Scan processed: raw=%zu filtered=%zu "
                "sensor_origin=(%.2f,%.2f,%.2f)",
                cloud_raw->size(), map_points.size(),
                sensor_origin.x(), sensor_origin.y(),
                sensor_origin.z());

    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        pending_points_ = std::move(map_points);
        pending_sensor_origin_ = sensor_origin;
        pending_sensor_forward_ = sensor_forward;
        new_scan_available_ = true;
    }
    data_cv_.notify_one();
    last_scan_time_ = now;
}

void MapUpdaterNode::processLoop()
{
    while (running_)
    {
        PointVec points_to_add;
        Eigen::Vector3f sensor_origin;
        Eigen::Vector3f sensor_forward;
        ServiceCmd cmd = ServiceCmd::NONE;
        std::string cmd_path;

        {
            std::unique_lock<std::mutex> lock(data_mutex_);
            auto interval = std::chrono::duration<double>(
                1.0 / std::max(config_.publish_rate_hz, 0.1));
            data_cv_.wait_for(lock, interval, [this]()
                              { return !running_ || new_scan_available_ ||
                                       pending_cmd_ != ServiceCmd::NONE; });
            if (!running_)
            {
                break;
            }

            if (new_scan_available_)
            {
                points_to_add = std::move(pending_points_);
                sensor_origin = pending_sensor_origin_;
                sensor_forward = pending_sensor_forward_;
                new_scan_available_ = false;
            }

            if (pending_cmd_ != ServiceCmd::NONE)
            {
                cmd = pending_cmd_;
                cmd_path = pending_cmd_path_;
            }
        }

        // Handle service commands (thread-safe: ikd-tree only on this thread)
        if (cmd == ServiceCmd::SAVE)
        {
            bool ok = core_->savePCD(cmd_path);
            std::lock_guard<std::mutex> lock(data_mutex_);
            pending_cmd_ = ServiceCmd::NONE;
            pending_cmd_result_ = ok;
            cmd_done_cv_.notify_all();
        }
        else if (cmd == ServiceCmd::RELOAD)
        {
            core_->reset();
            bool ok = core_->loadPCD(cmd_path);
            std::lock_guard<std::mutex> lock(data_mutex_);
            pending_cmd_ = ServiceCmd::NONE;
            pending_cmd_result_ = ok;
            cmd_done_cv_.notify_all();
        }

        if (!points_to_add.empty())
        {
            doIncrementalAdd(points_to_add);
            if (config_.enable_point_removal)
            {
                doPointRemoval(sensor_origin, sensor_forward, points_to_add);
            }
        }

        auto now = Clock::now();
        double since_pub =
            std::chrono::duration<double>(now - last_publish_time_).count();
        if (since_pub >= 1.0 / std::max(config_.publish_rate_hz, 0.1))
        {
            publishUpdatedMap();
            last_publish_time_ = now;
        }

        double since_diag =
            std::chrono::duration<double>(now - last_diag_time_).count();
        if (since_diag >= config_.diag_interval_s)
        {
            logDiagnostics();
            last_diag_time_ = now;
        }
    }
}

void MapUpdaterNode::doIncrementalAdd(PointVec& points)
{
    int before = core_->validNum();
    int added = core_->addPoints(points);
    int after = core_->validNum();
    scans_processed_++;
    points_added_total_ += static_cast<uint64_t>(added);

    RCLCPP_INFO(get_logger(),
                "addPoints: input=%zu ikd_added=%d "
                "map_before=%d map_after=%d delta=%d",
                points.size(), added, before, after,
                after - before);
}

void MapUpdaterNode::doPointRemoval(
    const Eigen::Vector3f& origin,
    const Eigen::Vector3f& forward,
    const PointVec& endpoints)
{
    // Set frontal sector for targeted diagnostics.
    core_->setVoxelFrontalSector(
        origin, forward,
        static_cast<float>(config_.diag_sector_radius_m),
        static_cast<float>(config_.diag_sector_fov_deg));

    int before = core_->validNum();
    int removed = core_->rayCastAndRemove(
        origin, endpoints,
        config_.ray_cast_skip);
    int after = core_->validNum();
    points_removed_total_ += static_cast<uint64_t>(removed);

    auto stats = core_->getVoxelStats();

    RCLCPP_INFO(get_logger(),
                "pointRemoval: endpoints=%zu rays=%zu "
                "expired_removed=%d map_before=%d map_after=%d "
                "min_logodds=%.2f voxels=%zu",
                endpoints.size(),
                endpoints.size() / std::max(config_.ray_cast_skip, 1),
                removed, before, after,
                core_->minLogOdds(), core_->voxelMapSize());

    RCLCPP_INFO(get_logger(),
                "  rayStats: hit(new=%d exist=%d protect=%d neighbor=%d) "
                "rays(cast=%d same_key=%d productive=%d) "
                "steps(total=%d decr=%d cross_zero=%d not_in_map=%d dedup=%d)",
                stats.hit_new, stats.hit_existing, stats.hits_protecting,
                stats.hit_neighbor,
                stats.ray_count, stats.ray_same_key, stats.rays_productive,
                stats.ray_steps, stats.ray_incremented, stats.ray_from_zero,
                stats.ray_not_in_map, stats.ray_dedup_skips);

    // Sector stats: front radius/FOV region for obstacle-area cycling analysis.
    RCLCPP_INFO(get_logger(),
                "  sector(%.1fm,%.0fdeg): hit(new=%d exist=%d protect=%d neighbor=%d) "
                "ray(incr=%d from_zero=%d) "
                "protect_ratio=%.0f%%",
                config_.diag_sector_radius_m, config_.diag_sector_fov_deg,
                stats.sector_hit_new, stats.sector_hit_existing,
                stats.sector_hits_protecting, stats.sector_hit_neighbor,
                stats.sector_ray_incremented, stats.sector_ray_from_zero,
                stats.sector_hit_existing > 0
                    ? 100.0 * stats.sector_hits_protecting / stats.sector_hit_existing
                    : 0.0);
}

void MapUpdaterNode::publishUpdatedMap()
{
    if (map_pub_->get_subscription_count() < 1)
    {
        return;
    }

    CloudType::Ptr cloud = core_->flattenToCloud();
    if (!cloud || cloud->empty())
    {
        return;
    }

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.frame_id = config_.map_frame;
    msg.header.stamp = now();
    map_pub_->publish(msg);
}

void MapUpdaterNode::logDiagnostics()
{
    RCLCPP_INFO(get_logger(),
                "Diag: valid=%d tree=%d voxels=%zu min_logodds=%.2f "
                "scans(recv=%lu proc=%lu drop_no_odom=%lu) "
                "pts_added=%lu pts_removed=%lu",
                core_->validNum(), core_->treeSize(), core_->voxelMapSize(),
                core_->minLogOdds(),
                scans_received_, scans_processed_, scans_dropped_no_odom_,
                points_added_total_, points_removed_total_);

    auto hist = core_->getLogOddsHistogram();
    RCLCPP_INFO(get_logger(),
                "  logOdds: [>=2.0]=%d [1-2)=%d [0-1)=%d [neg]=%d [<free]=%d "
                "total=%d min=%.2f",
                hist.bin_strong_occ, hist.bin_occ, hist.bin_weak_occ,
                hist.bin_uncertain, hist.bin_free,
                hist.total, hist.min_log_odds);
}

void MapUpdaterNode::handleSaveMap(
    const std::shared_ptr<interface::srv::SaveMaps::Request> req,
    std::shared_ptr<interface::srv::SaveMaps::Response> resp)
{
    RCLCPP_INFO(get_logger(), "Save map request: %s", req->file_path.c_str());

    std::unique_lock<std::mutex> lock(data_mutex_);
    pending_cmd_ = ServiceCmd::SAVE;
    pending_cmd_path_ = req->file_path;
    data_cv_.notify_one();

    cmd_done_cv_.wait(lock, [this]()
                      { return pending_cmd_ == ServiceCmd::NONE; });

    resp->success = pending_cmd_result_;
    resp->message = resp->success ? "Map saved" : "Save failed";
}

void MapUpdaterNode::handleReloadMap(
    const std::shared_ptr<interface::srv::SaveMaps::Request> req,
    std::shared_ptr<interface::srv::SaveMaps::Response> resp)
{
    RCLCPP_INFO(get_logger(), "Reload map request: %s",
                req->file_path.c_str());

    std::unique_lock<std::mutex> lock(data_mutex_);
    pending_cmd_ = ServiceCmd::RELOAD;
    pending_cmd_path_ = req->file_path;
    data_cv_.notify_one();

    cmd_done_cv_.wait(lock, [this]()
                      { return pending_cmd_ == ServiceCmd::NONE; });

    resp->success = pending_cmd_result_;
    resp->message = resp->success ? "Map reloaded" : "Reload failed";
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(MapUpdaterNode)

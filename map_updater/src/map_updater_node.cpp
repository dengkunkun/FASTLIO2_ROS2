#include "map_updater_node.hpp"

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <functional>

MapUpdaterNode::MapUpdaterNode(const rclcpp::NodeOptions& options)
    : Node("map_updater_node", options)
{
    loadConfig();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    CoreConfig core_cfg;
    core_cfg.map_resolution = config_.map_resolution;
    core_cfg.scan_resolution = config_.scan_resolution;
    core_ = std::make_unique<MapUpdaterCore>(core_cfg);

    loadInitialMap();

    map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "updated_map", rclcpp::QoS(1).transient_local());

    scan_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        config_.scan_topic, rclcpp::QoS(5),
        std::bind(&MapUpdaterNode::onLidarScan, this, std::placeholders::_1));

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
                "MapUpdaterNode started. scan_topic=%s map_frame=%s "
                "map_resolution=%.2f publish_rate=%.1fHz",
                config_.scan_topic.c_str(), config_.map_frame.c_str(),
                config_.map_resolution, config_.publish_rate_hz);
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
    config_.map_frame = get("map_frame", std::string("map_camera_init"));
    config_.scan_topic =
        get("scan_topic", std::string("/livox/lidar_base_link_filtered"));
    config_.tf_timeout_s = get("tf_timeout_s", 0.2);
    config_.scan_process_interval_s = get("scan_process_interval_s", 0.5);
    config_.map_resolution = get("map_resolution", 0.2);
    config_.scan_resolution = get("scan_resolution", 0.15);
    config_.publish_rate_hz = get("publish_rate_hz", 1.0);
    config_.diag_interval_s = get("diag_interval_s", 10.0);
    config_.enable_point_removal = get("enable_point_removal", false);
    config_.miss_count_threshold = get("miss_count_threshold", 20);
    config_.ray_cast_skip = get("ray_cast_skip", 10);

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

    RCLCPP_DEBUG(get_logger(),
                 "Scan received: frame=%s points=%u×%u",
                 msg->header.frame_id.c_str(), msg->width, msg->height);

    geometry_msgs::msg::TransformStamped tf_stamped;
    try
    {
        tf_stamped = tf_buffer_->lookupTransform(
            config_.map_frame, msg->header.frame_id,
            tf2::TimePointZero,
            tf2::durationFromSec(config_.tf_timeout_s));
    }
    catch (const tf2::TransformException& ex)
    {
        scans_dropped_tf_++;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "TF lookup failed (%s->%s): %s",
                             config_.map_frame.c_str(),
                             msg->header.frame_id.c_str(), ex.what());
        return;
    }

    // Receive as PointXYZ to avoid "Failed to find match for field 'intensity'"
    // when source topic has no intensity field (e.g. mid360_filter output)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud_xyz);
    if (cloud_xyz->empty())
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Received empty scan");
        return;
    }

    // Convert PointXYZ → PointXYZI for internal use
    CloudType::Ptr cloud_raw(new CloudType);
    cloud_raw->points.resize(cloud_xyz->size());
    for (size_t i = 0; i < cloud_xyz->size(); ++i)
    {
        cloud_raw->points[i].x = cloud_xyz->points[i].x;
        cloud_raw->points[i].y = cloud_xyz->points[i].y;
        cloud_raw->points[i].z = cloud_xyz->points[i].z;
        cloud_raw->points[i].intensity = 0.0f;
    }
    cloud_raw->width = static_cast<uint32_t>(cloud_raw->points.size());
    cloud_raw->height = 1;
    cloud_raw->is_dense = cloud_xyz->is_dense;

    CloudType::Ptr cloud_filtered = core_->preFilterScan(cloud_raw);

    Eigen::Isometry3d tf_eigen = tf2::transformToEigen(tf_stamped);
    Eigen::Affine3f transform = tf_eigen.cast<float>();

    PointVec map_points;
    map_points.reserve(cloud_filtered->size());
    for (const auto& pt : cloud_filtered->points)
    {
        Eigen::Vector3f p(pt.x, pt.y, pt.z);
        Eigen::Vector3f pm = transform * p;
        PointType mp;
        mp.x = pm.x();
        mp.y = pm.y();
        mp.z = pm.z();
        mp.intensity = pt.intensity;
        map_points.push_back(mp);
    }

    Eigen::Vector3f sensor_origin = transform.translation();

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Scan processed: raw=%zu filtered=%zu "
                         "sensor_origin=(%.2f,%.2f,%.2f)",
                         cloud_xyz->size(), map_points.size(),
                         sensor_origin.x(), sensor_origin.y(),
                         sensor_origin.z());

    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        pending_points_ = std::move(map_points);
        pending_sensor_origin_ = sensor_origin;
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
                doPointRemoval(sensor_origin, points_to_add);
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

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                         "addPoints: input=%zu ikd_added=%d "
                         "map_before=%d map_after=%d delta=%d",
                         points.size(), added, before, after,
                         after - before);
}

void MapUpdaterNode::doPointRemoval(
    const Eigen::Vector3f& origin,
    const PointVec& endpoints)
{
    int before = core_->validNum();
    int removed = core_->rayCastAndRemove(
        origin, endpoints,
        config_.ray_cast_skip, config_.miss_count_threshold);
    int after = core_->validNum();
    points_removed_total_ += static_cast<uint64_t>(removed);

    auto stats = core_->getVoxelStats();

    RCLCPP_INFO(get_logger(),
                "pointRemoval: endpoints=%zu rays=%zu "
                "expired_removed=%d map_before=%d map_after=%d "
                "max_miss=%u voxels=%zu",
                endpoints.size(),
                endpoints.size() / std::max(config_.ray_cast_skip, 1),
                removed, before, after,
                core_->maxMissCount(), core_->voxelMapSize());

    RCLCPP_INFO(get_logger(),
                "  rayStats: hit(new=%d exist=%d) "
                "rays(cast=%d same_key=%d productive=%d) "
                "steps(total=%d incremented=%d not_in_map=%d)",
                stats.hit_new, stats.hit_existing,
                stats.ray_count, stats.ray_same_key, stats.rays_productive,
                stats.ray_steps, stats.ray_incremented, stats.ray_not_in_map);
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
                "Diag: valid=%d tree=%d voxels=%zu max_miss=%u "
                "scans(recv=%lu proc=%lu drop_tf=%lu) "
                "pts_added=%lu pts_removed=%lu",
                core_->validNum(), core_->treeSize(), core_->voxelMapSize(),
                core_->maxMissCount(),
                scans_received_, scans_processed_, scans_dropped_tf_,
                points_added_total_, points_removed_total_);

    auto hist = core_->getMissHistogram();
    RCLCPP_INFO(get_logger(),
                "  missHist: [0]=%d [1-3]=%d [4-6]=%d [7-9]=%d [>=10]=%d "
                "total=%d max=%u",
                hist.bin_0, hist.bin_1_3, hist.bin_4_6, hist.bin_7_9,
                hist.bin_ge10, hist.total, hist.max_miss);
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

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <queue>
#include <filesystem>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <cassert>
#include <cmath>
#include "pgos/commons.h"
#include "pgos/simple_pgo.h"
#include "interface/srv/save_maps.hpp"
#include "interface/srv/load_pgo_session.hpp"
#include <pcl/common/io.h>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

// For component registration
#include <rclcpp_components/register_node_macro.hpp>

using namespace std::chrono_literals;

struct NodeConfig
{
    std::string cloud_topic = "/lio/body_cloud";
    std::string odom_topic = "/lio/odom";
    std::string map_frame = "map";
    std::string local_frame = "lidar";
    bool publish_tf = true;  // 是否发布 TF，在 remap_mode 下可禁用避免与 localizer 冲突
};

struct NodeState
{
    std::mutex message_mutex;
    std::queue<CloudWithPose> cloud_buffer;
    double last_message_time;
    bool new_data_available = false;
};

class PGONode : public rclcpp::Node
{
public:
    explicit PGONode(const rclcpp::NodeOptions & options)
    : Node("pgo_node", rclcpp::NodeOptions(options).enable_logger_service(true))
    , m_running(true)
    {
        RCLCPP_INFO(this->get_logger(), "PGO node started (Composed)");
        m_state.last_message_time = -1.0;
        loadParameters();
        m_pgo = std::make_shared<SimplePGO>(m_pgo_config);
        m_param_cb = this->add_on_set_parameters_callback(
            std::bind(&PGONode::onParametersChanged, this, std::placeholders::_1));
        rclcpp::QoS qos = rclcpp::QoS(10);
        m_cloud_sub.subscribe(this, m_node_config.cloud_topic, qos.get_rmw_qos_profile());
        m_odom_sub.subscribe(this, m_node_config.odom_topic, qos.get_rmw_qos_profile());
        m_loop_marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/pgo/loop_markers", 10000);
        m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
        m_sync = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>>(message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>(10), m_cloud_sub, m_odom_sub);
        m_sync->setAgePenalty(0.1);
        m_sync->registerCallback(std::bind(&PGONode::syncCB, this, std::placeholders::_1, std::placeholders::_2));
        
        // Create callback groups for concurrent execution
        m_service_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        m_timer_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        // Timer for lightweight TF broadcasting (uses timer callback group)
        m_tf_timer = this->create_wall_timer(20ms, std::bind(&PGONode::tfTimerCB, this), m_timer_cb_group);
        
        // Services use service callback group for concurrent handling
        m_save_map_srv = this->create_service<interface::srv::SaveMaps>(
            "/pgo/save_maps",
            std::bind(&PGONode::saveMapsCB, this, std::placeholders::_1, std::placeholders::_2),
            rclcpp::ServicesQoS().get_rmw_qos_profile(),
            m_service_cb_group);

        m_load_session_srv = this->create_service<interface::srv::LoadPGOSession>(
            "/pgo/load_session",
            std::bind(&PGONode::loadSessionCB, this, std::placeholders::_1, std::placeholders::_2),
            rclcpp::ServicesQoS().get_rmw_qos_profile(),
            m_service_cb_group);
        RCLCPP_INFO(this->get_logger(), "PGO services ready: /pgo/save_maps, /pgo/load_session");
        
        // Start PGO processing thread (heavy computation)
        m_process_thread = std::thread(&PGONode::pgoProcessLoop, this);
    }
    
    ~PGONode()
    {
        m_running = false;
        m_cv.notify_all();
        if (m_process_thread.joinable()) {
            m_process_thread.join();
        }
        RCLCPP_INFO(this->get_logger(), "PGO Node Shutdown");
    }

    void loadParameters()
    {
        this->declare_parameter("config_path", "");
        std::string config_path;
        this->get_parameter<std::string>("config_path", config_path);
        if (!config_path.empty())
        {
            YAML::Node config = YAML::LoadFile(config_path);
            if (!config)
            {
                RCLCPP_WARN(this->get_logger(), "FAIL TO LOAD YAML FILE: %s", config_path.c_str());
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "LOAD FROM YAML CONFIG PATH: %s", config_path.c_str());
                m_node_config.cloud_topic = config["cloud_topic"].as<std::string>();
                m_node_config.odom_topic = config["odom_topic"].as<std::string>();
                m_node_config.map_frame = config["map_frame"].as<std::string>();
                m_node_config.local_frame = config["local_frame"].as<std::string>();

                if (config["publish_tf"])
                {
                    m_node_config.publish_tf = config["publish_tf"].as<bool>();
                }

                m_pgo_config.key_pose_delta_deg = config["key_pose_delta_deg"].as<double>();
                m_pgo_config.key_pose_delta_trans = config["key_pose_delta_trans"].as<double>();
                m_pgo_config.loop_search_radius = config["loop_search_radius"].as<double>();
                m_pgo_config.loop_time_tresh = config["loop_time_tresh"].as<double>();
                m_pgo_config.loop_score_tresh = config["loop_score_tresh"].as<double>();
                m_pgo_config.loop_submap_half_range = config["loop_submap_half_range"].as<int>();
                m_pgo_config.submap_resolution = config["submap_resolution"].as<double>();
                m_pgo_config.min_loop_detect_duration = config["min_loop_detect_duration"].as<double>();
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "config_path is empty; using default parameters");
        }

        declareParameterIfNotDeclared("cloud_topic", m_node_config.cloud_topic);
        declareParameterIfNotDeclared("odom_topic", m_node_config.odom_topic);
        declareParameterIfNotDeclared("map_frame", m_node_config.map_frame);
        declareParameterIfNotDeclared("local_frame", m_node_config.local_frame);
        declareParameterIfNotDeclared("key_pose_delta_deg", m_pgo_config.key_pose_delta_deg);
        declareParameterIfNotDeclared("key_pose_delta_trans", m_pgo_config.key_pose_delta_trans);
        declareParameterIfNotDeclared("loop_search_radius", m_pgo_config.loop_search_radius);
        declareParameterIfNotDeclared("loop_time_tresh", m_pgo_config.loop_time_tresh);
        declareParameterIfNotDeclared("loop_score_tresh", m_pgo_config.loop_score_tresh);
        declareParameterIfNotDeclared("loop_submap_half_range", m_pgo_config.loop_submap_half_range);
        declareParameterIfNotDeclared("submap_resolution", m_pgo_config.submap_resolution);
        declareParameterIfNotDeclared("min_loop_detect_duration", m_pgo_config.min_loop_detect_duration);

        this->get_parameter("cloud_topic", m_node_config.cloud_topic);
        this->get_parameter("odom_topic", m_node_config.odom_topic);
        this->get_parameter("map_frame", m_node_config.map_frame);
        this->get_parameter("local_frame", m_node_config.local_frame);
        this->get_parameter("key_pose_delta_deg", m_pgo_config.key_pose_delta_deg);
        this->get_parameter("key_pose_delta_trans", m_pgo_config.key_pose_delta_trans);
        this->get_parameter("loop_search_radius", m_pgo_config.loop_search_radius);
        this->get_parameter("loop_time_tresh", m_pgo_config.loop_time_tresh);
        this->get_parameter("loop_score_tresh", m_pgo_config.loop_score_tresh);
        this->get_parameter("loop_submap_half_range", m_pgo_config.loop_submap_half_range);
        this->get_parameter("submap_resolution", m_pgo_config.submap_resolution);
        this->get_parameter("min_loop_detect_duration", m_pgo_config.min_loop_detect_duration);

        RCLCPP_INFO(this->get_logger(),
                    "PGO params: cloud_topic=%s odom_topic=%s map_frame=%s local_frame=%s publish_tf=%s",
                    m_node_config.cloud_topic.c_str(),
                    m_node_config.odom_topic.c_str(),
                    m_node_config.map_frame.c_str(),
                    m_node_config.local_frame.c_str(),
                    m_node_config.publish_tf ? "true" : "false");
        RCLCPP_INFO(this->get_logger(),
                    "PGO config: key_pose_delta_deg=%.3f key_pose_delta_trans=%.3f loop_search_radius=%.3f",
                    m_pgo_config.key_pose_delta_deg,
                    m_pgo_config.key_pose_delta_trans,
                    m_pgo_config.loop_search_radius);
        RCLCPP_INFO(this->get_logger(),
                    "PGO config: loop_time_tresh=%.3f loop_score_tresh=%.3f loop_submap_half_range=%d",
                    m_pgo_config.loop_time_tresh,
                    m_pgo_config.loop_score_tresh,
                    m_pgo_config.loop_submap_half_range);
        RCLCPP_INFO(this->get_logger(),
                    "PGO config: submap_resolution=%.3f min_loop_detect_duration=%.3f",
                    m_pgo_config.submap_resolution,
                    m_pgo_config.min_loop_detect_duration);
    }
    void syncCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg, const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg)
    {

        std::lock_guard<std::mutex>(m_state.message_mutex);
        CloudWithPose cp;
        cp.pose.setTime(cloud_msg->header.stamp.sec, cloud_msg->header.stamp.nanosec);
        if (cp.pose.second < m_state.last_message_time)
        {
            RCLCPP_WARN(this->get_logger(), "Received out of order message");
            return;
        }
        m_state.last_message_time = cp.pose.second;

        cp.pose.r = Eigen::Quaterniond(odom_msg->pose.pose.orientation.w,
                                       odom_msg->pose.pose.orientation.x,
                                       odom_msg->pose.pose.orientation.y,
                                       odom_msg->pose.pose.orientation.z)
                        .toRotationMatrix();
        cp.pose.t = V3D(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z);
        cp.cloud = CloudType::Ptr(new CloudType);
        pcl::fromROSMsg(*cloud_msg, *cp.cloud);
        m_state.cloud_buffer.push(cp);
        m_state.new_data_available = true;
        
        // Notify processing thread
        m_cv.notify_one();
    }

    /**
     * @brief Lightweight TF timer - broadcasts map->local transform
     * @note Uses try_lock to avoid blocking, skips if lock unavailable
     */
    void tfTimerCB()
    {
        if (!m_node_config.publish_tf) return;  // 如果禁用 TF 发布，直接返回
        
        std::unique_lock<std::mutex> lock(m_pgo_mutex, std::try_to_lock);
        if (!lock.owns_lock()) return;
        if (m_pgo->keyPoses().empty()) return;
        
        // Build and send TF while holding the lock
        builtin_interfaces::msg::Time cur_time = this->get_clock()->now();
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.frame_id = m_node_config.map_frame;
        transformStamped.child_frame_id = m_node_config.local_frame;
        transformStamped.header.stamp = cur_time;
        Eigen::Quaterniond q(m_pgo->offsetR());
        V3D t = m_pgo->offsetT();
        transformStamped.transform.translation.x = t.x();
        transformStamped.transform.translation.y = t.y();
        transformStamped.transform.translation.z = t.z();
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        m_tf_broadcaster->sendTransform(transformStamped);
    }
    
    // PGO processing loop runs in separate thread
    void pgoProcessLoop()
    {
        while (m_running) {
            // Wait for new data or timeout
            {
                std::unique_lock<std::mutex> lock(m_cv_mutex);
                m_cv.wait_for(lock, 50ms, [this]() {
                    return !m_running || m_state.new_data_available;
                });
            }
            
            if (!m_running) break;
            
            processPGO();
        }
    }
    
    void processPGO()
    {
        if (m_state.cloud_buffer.empty())
            return;
            
        CloudWithPose cp;
        {
            std::lock_guard<std::mutex> lock(m_state.message_mutex);
            if (m_state.cloud_buffer.empty()) return;
            cp = m_state.cloud_buffer.front();
            // Clear queue
            while (!m_state.cloud_buffer.empty())
            {
                m_state.cloud_buffer.pop();
            }
            m_state.new_data_available = false;
        }
        
        builtin_interfaces::msg::Time cur_time;
        cur_time.sec = cp.pose.sec;
        cur_time.nanosec = cp.pose.nsec;
        
        std::vector<KeyPoseWithCloud> key_poses_snapshot;
        std::vector<std::pair<size_t, size_t>> history_pairs_snapshot;
        Config config_snapshot;
        {
            std::lock_guard<std::mutex> lock(m_pgo_mutex);
            if (!m_pgo->addKeyPose(cp))
            {
                return;
            }
            key_poses_snapshot = m_pgo->keyPoses();
            history_pairs_snapshot = m_pgo->historyPairs();
            config_snapshot = m_pgo_config;
        }

        LoopPair loop_pair;
        bool has_loop = detectLoopPair(key_poses_snapshot, history_pairs_snapshot, config_snapshot, loop_pair);

        {
            std::lock_guard<std::mutex> lock(m_pgo_mutex);
            if (m_pgo->keyPoses().size() != key_poses_snapshot.size())
            {
                return;
            }
            if (has_loop)
            {
                m_pgo->appendLoopPair(loop_pair);
            }
            m_pgo->smoothAndUpdate();
        }

        publishLoopMarkers(cur_time);
    }

    /**
     * @brief Build and send TF transform from map_frame to local_frame
     * @param time Timestamp for the transform
     * @note Caller must NOT hold m_pgo_mutex (this function acquires it internally)
     * @deprecated Use tfTimerCB() directly for TF broadcasting
     */
    void sendBroadCastTF(builtin_interfaces::msg::Time &time)
    {
        std::lock_guard<std::mutex> lock(m_pgo_mutex);
        if (m_pgo->keyPoses().empty()) return;
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.frame_id = m_node_config.map_frame;
        transformStamped.child_frame_id = m_node_config.local_frame;
        transformStamped.header.stamp = time;
        Eigen::Quaterniond q(m_pgo->offsetR());
        V3D t = m_pgo->offsetT();
        transformStamped.transform.translation.x = t.x();
        transformStamped.transform.translation.y = t.y();
        transformStamped.transform.translation.z = t.z();
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        m_tf_broadcaster->sendTransform(transformStamped);
    }

    void publishLoopMarkers(builtin_interfaces::msg::Time &time)
    {
        if (m_loop_marker_pub->get_subscription_count() == 0)
            return;
        std::vector<KeyPoseWithCloud> poses_copy;
        std::vector<std::pair<size_t, size_t>> pairs_copy;
        {
            std::lock_guard<std::mutex> lock(m_pgo_mutex);
            if (m_pgo->historyPairs().empty())
                return;
            poses_copy = m_pgo->keyPoses();
            pairs_copy = m_pgo->historyPairs();
        }

        visualization_msgs::msg::MarkerArray marker_array;
        visualization_msgs::msg::Marker nodes_marker;
        visualization_msgs::msg::Marker edges_marker;
        nodes_marker.header.frame_id = m_node_config.map_frame;
        nodes_marker.header.stamp = time;
        nodes_marker.ns = "pgo_nodes";
        nodes_marker.id = 0;
        nodes_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        nodes_marker.action = visualization_msgs::msg::Marker::ADD;
        nodes_marker.pose.orientation.w = 1.0;
        nodes_marker.scale.x = 0.3;
        nodes_marker.scale.y = 0.3;
        nodes_marker.scale.z = 0.3;
        nodes_marker.color.r = 1.0;
        nodes_marker.color.g = 0.8;
        nodes_marker.color.b = 0.0;
        nodes_marker.color.a = 1.0;

        edges_marker.header.frame_id = m_node_config.map_frame;
        edges_marker.header.stamp = time;
        edges_marker.ns = "pgo_edges";
        edges_marker.id = 1;
        edges_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        edges_marker.action = visualization_msgs::msg::Marker::ADD;
        edges_marker.pose.orientation.w = 1.0;
        edges_marker.scale.x = 0.1;
        edges_marker.color.r = 0.0;
        edges_marker.color.g = 0.8;
        edges_marker.color.b = 0.0;
        edges_marker.color.a = 1.0;

        for (size_t i = 0; i < pairs_copy.size(); i++)
        {
            size_t i1 = pairs_copy[i].first;
            size_t i2 = pairs_copy[i].second;
            geometry_msgs::msg::Point p1, p2;
            p1.x = poses_copy[i1].t_global.x();
            p1.y = poses_copy[i1].t_global.y();
            p1.z = poses_copy[i1].t_global.z();

            p2.x = poses_copy[i2].t_global.x();
            p2.y = poses_copy[i2].t_global.y();
            p2.z = poses_copy[i2].t_global.z();

            nodes_marker.points.push_back(p1);
            nodes_marker.points.push_back(p2);
            edges_marker.points.push_back(p1);
            edges_marker.points.push_back(p2);
        }

        marker_array.markers.push_back(nodes_marker);
        marker_array.markers.push_back(edges_marker);
        m_loop_marker_pub->publish(marker_array);
    }

    void saveMapsCB(const std::shared_ptr<interface::srv::SaveMaps::Request> request, std::shared_ptr<interface::srv::SaveMaps::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "SaveMaps request: path=%s save_patches=%s",
                    request->file_path.c_str(), request->save_patches ? "true" : "false");
        if (!std::filesystem::exists(request->file_path))
        {
            response->success = false;
            response->message = request->file_path + " IS NOT EXISTS!";
            RCLCPP_WARN(this->get_logger(), "SaveMaps failed: %s", response->message.c_str());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "SaveMaps: output dir exists");

        std::vector<KeyPoseWithCloud> poses_copy;
        {
            std::lock_guard<std::mutex> lock(m_pgo_mutex);
            if (m_pgo->keyPoses().empty())
            {
                response->success = false;
                response->message = "NO POSES!";
                RCLCPP_WARN(this->get_logger(), "SaveMaps failed: %s", response->message.c_str());
                return;
            }
            poses_copy = m_pgo->keyPoses();
        }

        if (poses_copy.empty())
        {
            response->success = false;
            response->message = "NO POSES!";
            RCLCPP_WARN(this->get_logger(), "SaveMaps failed: %s", response->message.c_str());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "SaveMaps: poses_count=%zu", poses_copy.size());

        std::filesystem::path p_dir(request->file_path);
        std::filesystem::path patches_dir = p_dir / "patches";
        std::filesystem::path poses_txt_path = p_dir / "poses.txt";
        std::filesystem::path map_path = p_dir / "map.pcd";

        if (request->save_patches)
        {
            if (std::filesystem::exists(patches_dir))
            {
                std::filesystem::remove_all(patches_dir);
            }

            std::filesystem::create_directories(patches_dir);

            if (std::filesystem::exists(poses_txt_path))
            {
                std::filesystem::remove(poses_txt_path);
            }
            RCLCPP_INFO(this->get_logger(), "Patches Path: %s", patches_dir.string().c_str());
        }
        RCLCPP_INFO(this->get_logger(), "SAVE MAP TO %s", map_path.string().c_str());

        std::ofstream txt_file;
        if (request->save_patches)
        {
            txt_file.open(poses_txt_path);
        }

        CloudType::Ptr ret(new CloudType);
        for (size_t i = 0; i < poses_copy.size(); i++)
        {

            CloudType::Ptr body_cloud = poses_copy[i].body_cloud;
            if (request->save_patches)
            {
                std::string patch_name = std::to_string(i) + ".pcd";
                std::filesystem::path patch_path = patches_dir / patch_name;
                pcl::io::savePCDFileBinary(patch_path.string(), *body_cloud);
                const Eigen::Quaterniond q_local(poses_copy[i].r_local);
                const V3D t_local = poses_copy[i].t_local;
                const Eigen::Quaterniond q_global(poses_copy[i].r_global);
                const V3D t_global = poses_copy[i].t_global;

                // New format (v2):
                // patch_name
                // local_tx local_ty local_tz local_qw local_qx local_qy local_qz
                // global_tx global_ty global_tz global_qw global_qx global_qy global_qz
                txt_file << patch_name << " "
                         << t_local.x() << " " << t_local.y() << " " << t_local.z() << " "
                         << q_local.w() << " " << q_local.x() << " " << q_local.y() << " " << q_local.z() << " "
                         << t_global.x() << " " << t_global.y() << " " << t_global.z() << " "
                         << q_global.w() << " " << q_global.x() << " " << q_global.y() << " " << q_global.z()
                         << std::endl;
            }
            CloudType::Ptr world_cloud(new CloudType);
            pcl::transformPointCloud(*body_cloud, *world_cloud, poses_copy[i].t_global, Eigen::Quaterniond(poses_copy[i].r_global));
            *ret += *world_cloud;
        }
        if (txt_file.is_open())
        {
            txt_file.close();
        }
        pcl::io::savePCDFileBinary(map_path.string(), *ret);
        response->success = true;
        response->message = "SAVE SUCCESS!";
        RCLCPP_INFO(this->get_logger(), "SaveMaps success: %s", map_path.string().c_str());
    }

    void loadSessionCB(const std::shared_ptr<interface::srv::LoadPGOSession::Request> request,
                       std::shared_ptr<interface::srv::LoadPGOSession::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "LoadSession request: maps_path=%s load_patches=%s",
                    request->maps_path.c_str(), request->load_patches ? "true" : "false");
        if (!request->load_patches)
        {
            response->success = true;
            response->message = "load_patches=false, nothing to do";
            RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
            return;
        }

        bool need_reset = false;
        {
            std::lock_guard<std::mutex> lock(m_pgo_mutex);
            need_reset = !m_pgo->keyPoses().empty();
        }
        if (need_reset)
        {
            resetPGOState("load_session requested with existing keyposes");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "LoadSession: no existing keyposes, skip reset");
        }

        const std::filesystem::path maps_dir(request->maps_path);
        const std::filesystem::path poses_txt = maps_dir / "poses.txt";
        const std::filesystem::path patches_dir = maps_dir / "patches";
        RCLCPP_INFO(this->get_logger(), "LoadSession paths: poses=%s patches=%s",
                    poses_txt.string().c_str(), patches_dir.string().c_str());

        if (!std::filesystem::exists(poses_txt) || !std::filesystem::exists(patches_dir))
        {
            response->success = false;
            response->message = "poses.txt or patches/ not found under: " + maps_dir.string();
            RCLCPP_WARN(this->get_logger(), "LoadSession failed: %s", response->message.c_str());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "LoadSession: poses/patches exist");

        std::ifstream f(poses_txt);
        if (!f.is_open())
        {
            response->success = false;
            response->message = "failed to open: " + poses_txt.string();
            RCLCPP_WARN(this->get_logger(), "LoadSession failed: %s", response->message.c_str());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "LoadSession: reading poses.txt");

        // Append historical keyposes.
        // This expects poses.txt v2 format written by the updated save_maps:
        //   patch_name
        //   local_tx local_ty local_tz local_qw local_qx local_qy local_qz
        //   global_tx global_ty global_tz global_qw global_qx global_qy global_qz
        size_t loaded = 0;
        std::string patch_name;
        double l_tx, l_ty, l_tz, l_qw, l_qx, l_qy, l_qz;
        double g_tx, g_ty, g_tz, g_qw, g_qx, g_qy, g_qz;

        {
            std::lock_guard<std::mutex> lock(m_pgo_mutex);
            while (true)
            {
                if (!(f >> patch_name))
                {
                    break;
                }
            if (!(f >> l_tx >> l_ty >> l_tz >> l_qw >> l_qx >> l_qy >> l_qz
                  >> g_tx >> g_ty >> g_tz >> g_qw >> g_qx >> g_qy >> g_qz))
            {
                response->success = false;
                response->message = "poses.txt format is not v2; please re-save maps with the updated pgo_node (save_patches=true)";
                RCLCPP_WARN(this->get_logger(), "LoadSession failed: %s", response->message.c_str());
                return;
            }

                const std::filesystem::path patch_path = patches_dir / patch_name;
                if (!std::filesystem::exists(patch_path))
                {
                    RCLCPP_WARN(this->get_logger(), "Patch file missing: %s", patch_path.string().c_str());
                    continue;
                }

                CloudType::Ptr body_cloud(new CloudType);
                if (pcl::io::loadPCDFile<PointType>(patch_path.string(), *body_cloud) == -1)
                {
                    RCLCPP_WARN(this->get_logger(), "Failed to load patch: %s", patch_path.string().c_str());
                    continue;
                }

                Eigen::Quaterniond q_local(l_qw, l_qx, l_qy, l_qz);
                Eigen::Quaterniond q_global(g_qw, g_qx, g_qy, g_qz);
                if (!q_local.coeffs().allFinite() || !q_global.coeffs().allFinite())
                {
                    RCLCPP_WARN(this->get_logger(), "Invalid quaternion in poses.txt for %s", patch_name.c_str());
                    continue;
                }
                q_local.normalize();
                q_global.normalize();

                const M3D r_local = q_local.toRotationMatrix();
                const V3D t_local(l_tx, l_ty, l_tz);
                const M3D r_global = q_global.toRotationMatrix();
                const V3D t_global(g_tx, g_ty, g_tz);

                // time is not stored in poses.txt; set to 0.0
                if (!m_pgo->appendPriorKeyPose(r_local, t_local, r_global, t_global, 0.0, body_cloud))
                {
                    RCLCPP_WARN(this->get_logger(), "appendPriorKeyPose failed for %s", patch_name.c_str());
                    continue;
                }
                loaded++;
            }
            m_pgo->commitAppendedPriors();
            RCLCPP_INFO(this->get_logger(), "LoadSession: committed priors, loaded=%zu", loaded);
        }
        f.close();

        response->success = true;
        response->message = "Loaded session patches: " + std::to_string(loaded);
        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    }

private:
    CloudType::Ptr buildSubMap(const std::vector<KeyPoseWithCloud> &poses, int idx, int half_range, double resolution)
    {
        assert(idx >= 0 && idx < static_cast<int>(poses.size()));
        int min_idx = std::max(0, idx - half_range);
        int max_idx = std::min(static_cast<int>(poses.size()) - 1, idx + half_range);

        CloudType::Ptr ret(new CloudType);
        for (int i = min_idx; i <= max_idx; i++)
        {
            const CloudType::Ptr &body_cloud = poses[i].body_cloud;
            CloudType::Ptr global_cloud(new CloudType);
            pcl::transformPointCloud(*body_cloud, *global_cloud, poses[i].t_global, Eigen::Quaterniond(poses[i].r_global));
            *ret += *global_cloud;
        }
        if (resolution > 0)
        {
            pcl::VoxelGrid<PointType> voxel_grid;
            voxel_grid.setLeafSize(resolution, resolution, resolution);
            voxel_grid.setInputCloud(ret);
            voxel_grid.filter(*ret);
        }
        return ret;
    }

    bool detectLoopPair(const std::vector<KeyPoseWithCloud> &poses,
                        const std::vector<std::pair<size_t, size_t>> &history_pairs,
                        const Config &config,
                        LoopPair &out_pair)
    {
        if (poses.size() < 10)
            return false;
        if (config.min_loop_detect_duration > 0.0)
        {
            if (!history_pairs.empty())
            {
                double current_time = poses.back().time;
                double last_time = poses[history_pairs.back().second].time;
                if (current_time - last_time < config.min_loop_detect_duration)
                    return false;
            }
        }

        size_t cur_idx = poses.size() - 1;
        const KeyPoseWithCloud &last_item = poses.back();
        pcl::PointXYZ last_pose_pt;
        last_pose_pt.x = last_item.t_global(0);
        last_pose_pt.y = last_item.t_global(1);
        last_pose_pt.z = last_item.t_global(2);

        pcl::PointCloud<pcl::PointXYZ>::Ptr key_poses_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        key_poses_cloud->reserve(poses.size() - 1);
        for (size_t i = 0; i < poses.size() - 1; i++)
        {
            pcl::PointXYZ pt;
            pt.x = poses[i].t_global(0);
            pt.y = poses[i].t_global(1);
            pt.z = poses[i].t_global(2);
            key_poses_cloud->push_back(pt);
        }
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(key_poses_cloud);
        std::vector<int> ids;
        std::vector<float> sqdists;
        int neighbors = kdtree.radiusSearch(last_pose_pt, config.loop_search_radius, ids, sqdists);
        if (neighbors == 0)
            return false;

        int loop_idx = -1;
        for (size_t i = 0; i < ids.size(); i++)
        {
            int idx = ids[i];
            if (std::abs(last_item.time - poses[idx].time) > config.loop_time_tresh)
            {
                loop_idx = idx;
                break;
            }
        }

        if (loop_idx == -1)
            return false;

        CloudType::Ptr target_cloud = buildSubMap(poses, loop_idx, config.loop_submap_half_range, config.submap_resolution);
        CloudType::Ptr source_cloud = buildSubMap(poses, static_cast<int>(cur_idx), 0, config.submap_resolution);
        CloudType::Ptr align_cloud(new CloudType);

        pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setMaximumIterations(50);
        icp.setMaxCorrespondenceDistance(10);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setRANSACIterations(0);
        icp.setInputSource(source_cloud);
        icp.setInputTarget(target_cloud);
        icp.align(*align_cloud);

        if (!icp.hasConverged() || icp.getFitnessScore() > config.loop_score_tresh)
            return false;

        M4F loop_transform = icp.getFinalTransformation();

        out_pair.source_id = cur_idx;
        out_pair.target_id = loop_idx;
        out_pair.score = icp.getFitnessScore();
        M3D r_refined = loop_transform.block<3, 3>(0, 0).cast<double>() * poses[cur_idx].r_global;
        V3D t_refined = loop_transform.block<3, 3>(0, 0).cast<double>() * poses[cur_idx].t_global + loop_transform.block<3, 1>(0, 3).cast<double>();
        out_pair.r_offset = poses[loop_idx].r_global.transpose() * r_refined;
        out_pair.t_offset = poses[loop_idx].r_global.transpose() * (t_refined - poses[loop_idx].t_global);
        return true;
    }

    template <typename T>
    void declareParameterIfNotDeclared(const std::string &name, const T &value)
    {
        if (!this->has_parameter(name))
        {
            this->declare_parameter(name, value);
        }
    }

    rcl_interfaces::msg::SetParametersResult onParametersChanged(
        const std::vector<rclcpp::Parameter> &params)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        NodeConfig new_node_config = m_node_config;
        Config new_pgo_config = m_pgo_config;
        bool pgo_config_changed = false;
        bool frames_changed = false;

        for (const auto &param : params)
        {
            const std::string &name = param.get_name();
            if (name == "cloud_topic" || name == "odom_topic")
            {
                result.successful = false;
                result.reason = "cloud_topic/odom_topic changes require restart";
                return result;
            }
            if (name == "map_frame")
            {
                new_node_config.map_frame = param.as_string();
                frames_changed = true;
            }
            else if (name == "local_frame")
            {
                new_node_config.local_frame = param.as_string();
                frames_changed = true;
            }
            else if (name == "key_pose_delta_deg")
            {
                new_pgo_config.key_pose_delta_deg = param.as_double();
                pgo_config_changed = true;
            }
            else if (name == "key_pose_delta_trans")
            {
                new_pgo_config.key_pose_delta_trans = param.as_double();
                pgo_config_changed = true;
            }
            else if (name == "loop_search_radius")
            {
                new_pgo_config.loop_search_radius = param.as_double();
                pgo_config_changed = true;
            }
            else if (name == "loop_time_tresh")
            {
                new_pgo_config.loop_time_tresh = param.as_double();
                pgo_config_changed = true;
            }
            else if (name == "loop_score_tresh")
            {
                new_pgo_config.loop_score_tresh = param.as_double();
                pgo_config_changed = true;
            }
            else if (name == "loop_submap_half_range")
            {
                new_pgo_config.loop_submap_half_range = param.as_int();
                pgo_config_changed = true;
            }
            else if (name == "submap_resolution")
            {
                new_pgo_config.submap_resolution = param.as_double();
                pgo_config_changed = true;
            }
            else if (name == "min_loop_detect_duration")
            {
                new_pgo_config.min_loop_detect_duration = param.as_double();
                pgo_config_changed = true;
            }
        }

        if (pgo_config_changed || frames_changed)
        {
            std::lock_guard<std::mutex> lock(m_pgo_mutex);
            if (frames_changed)
            {
                m_node_config.map_frame = new_node_config.map_frame;
                m_node_config.local_frame = new_node_config.local_frame;
                RCLCPP_INFO(this->get_logger(), "Updated frames: map_frame=%s local_frame=%s",
                            m_node_config.map_frame.c_str(), m_node_config.local_frame.c_str());
            }
            if (pgo_config_changed)
            {
                m_pgo_config = new_pgo_config;
                m_pgo->updateConfig(m_pgo_config);
                RCLCPP_INFO(this->get_logger(),
                            "Updated PGO config: key_pose_delta_deg=%.3f key_pose_delta_trans=%.3f loop_search_radius=%.3f",
                            m_pgo_config.key_pose_delta_deg,
                            m_pgo_config.key_pose_delta_trans,
                            m_pgo_config.loop_search_radius);
            }
        }

        return result;
    }

    void resetPGOState(const std::string &reason)
    {
        {
            std::lock_guard<std::mutex> lock(m_pgo_mutex);
            m_pgo = std::make_shared<SimplePGO>(m_pgo_config);
        }
        {
            std::lock_guard<std::mutex> lock(m_state.message_mutex);
            while (!m_state.cloud_buffer.empty())
            {
                m_state.cloud_buffer.pop();
            }
            m_state.new_data_available = false;
            m_state.last_message_time = -1.0;
        }
        RCLCPP_WARN(this->get_logger(), "PGO state reset: %s", reason.c_str());
    }

    NodeConfig m_node_config;
    Config m_pgo_config;
    NodeState m_state;
    std::shared_ptr<SimplePGO> m_pgo;
    std::mutex m_pgo_mutex;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr m_param_cb;
    rclcpp::TimerBase::SharedPtr m_tf_timer;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_loop_marker_pub;
    rclcpp::Service<interface::srv::SaveMaps>::SharedPtr m_save_map_srv;
    rclcpp::Service<interface::srv::LoadPGOSession>::SharedPtr m_load_session_srv;
    rclcpp::CallbackGroup::SharedPtr m_service_cb_group;
    rclcpp::CallbackGroup::SharedPtr m_timer_cb_group;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> m_cloud_sub;
    message_filters::Subscriber<nav_msgs::msg::Odometry> m_odom_sub;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>> m_sync;
    
    // Thread management for async PGO processing
    std::thread m_process_thread;
    std::atomic<bool> m_running;
    std::mutex m_cv_mutex;
    std::condition_variable m_cv;
};

// Register as composed node
RCLCPP_COMPONENTS_REGISTER_NODE(PGONode)

#include <queue>
#include <mutex>
#include <filesystem>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <cmath>
#include <algorithm>  // for std::clamp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "localizers/commons.h"
#include "localizers/icp_localizer.h"
#include "interface/srv/relocalize.hpp"
#include "interface/srv/is_valid.hpp"
#include <yaml-cpp/yaml.h>

// For component registration
#include <rclcpp_components/register_node_macro.hpp>

using namespace std::chrono_literals;

struct NodeConfig
{
    std::string cloud_topic = "/fastlio2/body_cloud";
    std::string odom_topic = "/fastlio2/lio_odom";
    std::string map_frame = "map";
    std::string local_frame = "lidar";
    double update_hz = 1.0;
    
    // ICP 结果过滤 - 防止 ICP 错误收敛导致的大跳变
    double max_offset_jump_t = 0.3;      // 允许的最大单帧 offset 位移变化 (m)
    double max_offset_jump_yaw = 5.0;    // 允许的最大单帧 offset yaw 变化 (deg)
};

// ============================================================================
// 诊断统计结构 (Diagnostic Statistics)
// ============================================================================
struct DiagnosticStats
{
    // ICP 质量追踪
    uint64_t icp_total_count = 0;
    uint64_t icp_success_count = 0;
    uint64_t icp_fail_count = 0;
    uint64_t icp_consecutive_fails = 0;
    uint64_t icp_max_consecutive_fails = 0;
    double icp_last_rough_score = 0.0;
    double icp_last_refine_score = 0.0;
    double icp_score_sum = 0.0;           // 累计 refine score (用于计算平均)
    double icp_score_max = 0.0;           // 最大 refine score
    double icp_score_min = 1e10;          // 最小 refine score
    
    // Offset 变化追踪
    double max_delta_offset_t = 0.0;      // 最大单次 offset 平移变化量
    double max_delta_offset_yaw = 0.0;    // 最大单次 offset yaw 变化量
    double cumulative_offset_drift = 0.0; // 累积 offset 漂移
    
    // FAST-LIO2 里程计追踪
    V3D first_odom_t = V3D::Zero();
    double first_odom_yaw = 0.0;
    bool first_odom_recorded = false;
    double odom_total_distance = 0.0;
    V3D prev_odom_t = V3D::Zero();
    
    // 时间追踪
    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point last_icp_success_time = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point last_diag_print_time = std::chrono::steady_clock::now();
    double max_icp_gap_seconds = 0.0;     // 最长无成功 ICP 的间隔
    
    // 输入数据质量
    size_t min_cloud_size = SIZE_MAX;
    size_t max_cloud_size = 0;
};

struct NodeState
{
    std::mutex message_mutex;
    std::mutex service_mutex;

    bool message_received = false;
    bool service_received = false;
    bool localize_success = false;
    rclcpp::Time last_send_tf_time = rclcpp::Clock().now();
    builtin_interfaces::msg::Time last_message_time;
    CloudType::Ptr last_cloud = std::make_shared<CloudType>();
    M3D last_r = M3D::Identity();        // localmap_body_r (初始化为单位矩阵)
    V3D last_t = V3D::Zero();            // localmap_body_t (初始化为零向量)
    M3D last_offset_r = M3D::Identity(); // map_localmap_r
    V3D last_offset_t = V3D::Zero();     // map_localmap_t
    M4F initial_guess = M4F::Identity();
    
    // 记录计算 offset 时的 local 位姿，用于增量计算
    // 这样可以避免 ICP 计算延迟导致的漂移
    M3D offset_ref_local_r = M3D::Identity();  // 计算 offset 时的 local_body_r
    V3D offset_ref_local_t = V3D::Zero();      // 计算 offset 时的 local_body_t
    
    // ICP 结果过滤状态
    bool has_valid_offset = false;              // 是否已有有效的 offset (第一次 ICP 后为 true)
    uint64_t icp_reject_count = 0;              // 因 offset 跳变过大被拒绝的 ICP 结果计数
    uint64_t icp_reject_consecutive = 0;        // 连续被拒绝的计数
    
    // For async processing
    bool new_data_available = false;
};

class LocalizerNode : public rclcpp::Node
{
public:
    explicit LocalizerNode(const rclcpp::NodeOptions & options)
    : Node("localizer_node", rclcpp::NodeOptions(options).enable_logger_service(true))
    , m_running(true)
    {
        RCLCPP_INFO(this->get_logger(), "Localizer Node Started (Composed)");
        loadParameters();
        rclcpp::QoS qos = rclcpp::QoS(10);
        m_cloud_sub.subscribe(this, m_config.cloud_topic, qos.get_rmw_qos_profile());
        m_odom_sub.subscribe(this, m_config.odom_topic, qos.get_rmw_qos_profile());

        m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        m_sync = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>>(message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>(10), m_cloud_sub, m_odom_sub);
        m_sync->setAgePenalty(0.1);
        m_sync->registerCallback(std::bind(&LocalizerNode::syncCB, this, std::placeholders::_1, std::placeholders::_2));
        m_localizer = std::make_shared<ICPLocalizer>(m_localizer_config, this->get_logger());

        m_reloc_srv = this->create_service<interface::srv::Relocalize>("relocalize", std::bind(&LocalizerNode::relocCB, this, std::placeholders::_1, std::placeholders::_2));

        m_reloc_check_srv = this->create_service<interface::srv::IsValid>("relocalize_check", std::bind(&LocalizerNode::relocCheckCB, this, std::placeholders::_1, std::placeholders::_2));

        m_map_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("map_cloud", 10);

        // Timer for TF broadcasting (lightweight, stays in callback thread)
        m_tf_timer = this->create_wall_timer(10ms, std::bind(&LocalizerNode::tfTimerCB, this));
        
        // Start ICP processing thread (heavy computation)
        m_process_thread = std::thread(&LocalizerNode::icpProcessLoop, this);
    }
    
    ~LocalizerNode()
    {
        m_running = false;
        m_cv.notify_all();
        if (m_process_thread.joinable()) {
            m_process_thread.join();
        }
        RCLCPP_INFO(this->get_logger(), "Localizer Node Shutdown");
    }

    void loadParameters()
    {
        this->declare_parameter("config_path", "");
        std::string config_path;
        this->get_parameter<std::string>("config_path", config_path);
        YAML::Node config = YAML::LoadFile(config_path);
        if (!config)
        {
            RCLCPP_WARN(this->get_logger(), "FAIL TO LOAD YAML FILE!");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "LOAD FROM YAML CONFIG PATH: %s", config_path.c_str());

        m_config.cloud_topic = config["cloud_topic"].as<std::string>();
        m_config.odom_topic = config["odom_topic"].as<std::string>();
        m_config.map_frame = config["map_frame"].as<std::string>();
        m_config.local_frame = config["local_frame"].as<std::string>();
        m_config.update_hz = config["update_hz"].as<double>();

        m_localizer_config.rough_scan_resolution = config["rough_scan_resolution"].as<double>();
        m_localizer_config.rough_map_resolution = config["rough_map_resolution"].as<double>();
        m_localizer_config.rough_max_iteration = config["rough_max_iteration"].as<int>();
        m_localizer_config.rough_score_thresh = config["rough_score_thresh"].as<double>();

        m_localizer_config.refine_scan_resolution = config["refine_scan_resolution"].as<double>();
        m_localizer_config.refine_map_resolution = config["refine_map_resolution"].as<double>();
        m_localizer_config.refine_max_iteration = config["refine_max_iteration"].as<int>();
        m_localizer_config.refine_score_thresh = config["refine_score_thresh"].as<double>();
        
        // ICP 结果过滤参数 (可选, 有默认值)
        if (config["max_offset_jump_t"]) {
            m_config.max_offset_jump_t = config["max_offset_jump_t"].as<double>();
        }
        if (config["max_offset_jump_yaw"]) {
            m_config.max_offset_jump_yaw = config["max_offset_jump_yaw"].as<double>();
        }
        RCLCPP_INFO(this->get_logger(), 
            "[CONFIG] ICP offset filter: max_jump_t=%.3fm, max_jump_yaw=%.1fdeg",
            m_config.max_offset_jump_t, m_config.max_offset_jump_yaw);
    }
    
    // Lightweight timer callback - only handles TF broadcasting
    void tfTimerCB()
    {
        if (!m_state.message_received)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                "Waiting for message from fastlio2 (body_cloud + lio_odom)...");
            return;
        }
        
        // Always broadcast TF at high rate
        sendBroadCastTF(m_state.last_message_time);
    }
    
    // ICP processing loop runs in separate thread
    void icpProcessLoop()
    {
        while (m_running) {
            // Wait for signal or timeout
            {
                std::unique_lock<std::mutex> lock(m_cv_mutex);
                auto wait_time = std::chrono::duration<double>(1.0 / m_config.update_hz);
                m_cv.wait_for(lock, wait_time, [this]() {
                    return !m_running || m_state.new_data_available;
                });
            }
            
            if (!m_running) break;
            
            // Process ICP alignment
            processICP();
        }
    }
    
    void processICP()
    {
        if (!m_state.message_received) return;
        
        m_diag.icp_total_count++;
        auto icp_start_time = std::chrono::steady_clock::now();
        
        M4F initial_guess = M4F::Identity();
        if (m_state.service_received)
        {
            std::lock_guard<std::mutex> lock(m_state.service_mutex);
            initial_guess = m_state.initial_guess;
            RCLCPP_INFO(this->get_logger(), "Using user-provided initial guess from relocalize service");
        }
        else
        {
            std::lock_guard<std::mutex> lock(m_state.message_mutex);
            initial_guess.block<3, 3>(0, 0) = (m_state.last_offset_r * m_state.last_r).cast<float>();
            initial_guess.block<3, 1>(0, 3) = (m_state.last_offset_r * m_state.last_t + m_state.last_offset_t).cast<float>();
        }

        if (!initial_guess.allFinite())
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Initial guess contains NaN/Inf, skipping ICP this cycle");
            return;
        }

        M3D current_local_r;
        V3D current_local_t;
        builtin_interfaces::msg::Time current_time;
        size_t cloud_size = 0;
        {
            std::lock_guard<std::mutex> lock(m_state.message_mutex);
            current_local_r = m_state.last_r;
            current_local_t = m_state.last_t;
            current_time = m_state.last_message_time;
            m_localizer->setInput(m_state.last_cloud);
            cloud_size = m_state.last_cloud->size();
            m_state.new_data_available = false;
        }
        
        // 跟踪输入点云质量
        if (cloud_size < m_diag.min_cloud_size) m_diag.min_cloud_size = cloud_size;
        if (cloud_size > m_diag.max_cloud_size) m_diag.max_cloud_size = cloud_size;
        
        // 跟踪里程计行驶距离
        if (!m_diag.first_odom_recorded) {
            m_diag.first_odom_t = current_local_t;
            auto safeYaw0 = [](const M3D& R) -> double { return std::atan2(R(1,0), R(0,0)); };
            m_diag.first_odom_yaw = safeYaw0(current_local_r);
            m_diag.prev_odom_t = current_local_t;
            m_diag.first_odom_recorded = true;
        } else {
            double step = (current_local_t - m_diag.prev_odom_t).head<2>().norm();
            m_diag.odom_total_distance += step;
            m_diag.prev_odom_t = current_local_t;
        }

        // 记录 ICP 前的 initial guess 位置（用于检测 ICP 修正量）
        V3D initial_guess_t(initial_guess(0,3), initial_guess(1,3), initial_guess(2,3));

        bool result = m_localizer->align(initial_guess);
        
        auto icp_end_time = std::chrono::steady_clock::now();
        double icp_duration_ms = std::chrono::duration_cast<std::chrono::microseconds>(
            icp_end_time - icp_start_time).count() / 1000.0;
        
        // 读取 ICP fitness scores
        double rough_score = m_localizer->lastRoughFitness();
        double refine_score = m_localizer->lastRefineFitness();
        
        RCLCPP_INFO(this->get_logger(), 
            "[ICP] result=%s cloud=%zu rough_score=%.4f refine_score=%.4f time=%.1fms", 
            result ? "SUCCESS" : "FAILED", cloud_size, rough_score, refine_score, icp_duration_ms);
        
        if (result)
        {
            m_diag.icp_success_count++;
            m_diag.icp_consecutive_fails = 0;
            m_diag.icp_last_rough_score = rough_score;
            m_diag.icp_last_refine_score = refine_score;
            m_diag.icp_score_sum += refine_score;
            if (refine_score > m_diag.icp_score_max) m_diag.icp_score_max = refine_score;
            if (refine_score < m_diag.icp_score_min) m_diag.icp_score_min = refine_score;
            
            // 计算 ICP 对 initial guess 的修正量
            V3D icp_result_t(initial_guess(0,3), initial_guess(1,3), initial_guess(2,3));
            double icp_correction = (icp_result_t - initial_guess_t).head<2>().norm();
            
            // 计算距上次成功 ICP 的时间间隔
            double gap_since_last = std::chrono::duration_cast<std::chrono::milliseconds>(
                icp_end_time - m_diag.last_icp_success_time).count() / 1000.0;
            m_diag.last_icp_success_time = icp_end_time;
            
            M3D map_body_r = initial_guess.block<3, 3>(0, 0).cast<double>();
            V3D map_body_t = initial_guess.block<3, 1>(0, 3).cast<double>();
            
            auto safeYaw = [](const M3D& R) -> double {
                return std::atan2(R(1, 0), R(0, 0));
            };
            auto safePitch = [](const M3D& R) -> double {
                return std::asin(-std::clamp(R(2, 0), -1.0, 1.0));
            };
            auto safeRoll = [](const M3D& R) -> double {
                return std::atan2(R(2, 1), R(2, 2));
            };
            
            double map_body_yaw = safeYaw(map_body_r);
            double map_body_pitch = safePitch(map_body_r);
            double map_body_roll = safeRoll(map_body_r);
            double local_body_yaw = safeYaw(current_local_r);
            
            std::lock_guard<std::mutex> lock(m_state.message_mutex);
            M3D new_offset_r = map_body_r * current_local_r.transpose();
            V3D new_offset_t = -map_body_r * current_local_r.transpose() * current_local_t + map_body_t;
            
            double offset_yaw = safeYaw(new_offset_r);
            
            // 计算 offset 变化量
            V3D delta_offset_t = new_offset_t - m_state.last_offset_t;
            double old_offset_yaw = safeYaw(m_state.last_offset_r);
            double delta_offset_yaw = offset_yaw - old_offset_yaw;
            while (delta_offset_yaw > M_PI) delta_offset_yaw -= 2.0 * M_PI;
            while (delta_offset_yaw < -M_PI) delta_offset_yaw += 2.0 * M_PI;
            
            double delta_offset_t_norm = delta_offset_t.head<2>().norm();
            double delta_offset_yaw_deg = std::fabs(delta_offset_yaw) * 180.0 / M_PI;
            
            // 更新诊断统计
            m_diag.cumulative_offset_drift += delta_offset_t_norm;
            if (delta_offset_t_norm > m_diag.max_delta_offset_t) m_diag.max_delta_offset_t = delta_offset_t_norm;
            if (delta_offset_yaw_deg > m_diag.max_delta_offset_yaw) m_diag.max_delta_offset_yaw = delta_offset_yaw_deg;
            
            // ===== 关键诊断日志 =====
            // 检测大的 offset 跳变 (可能是 ICP 错误收敛)
            if (delta_offset_t_norm > 0.05 || delta_offset_yaw_deg > 1.0) {
                RCLCPP_WARN(this->get_logger(), 
                    "[DRIFT_ALERT] Large offset change! dt_2d=%.4fm dyaw=%.3fdeg "
                    "(refine_score=%.4f, icp_correction=%.4fm, gap=%.1fs)",
                    delta_offset_t_norm, delta_offset_yaw_deg,
                    refine_score, icp_correction, gap_since_last);
            }
            
            // ===== ICP 结果过滤: 拒绝 offset 跳变过大的结果 =====
            // 跳过检查的条件:
            //   1. 首次获得有效 offset (has_valid_offset == false)
            //   2. 刚通过 relocalize 服务设置了初始位姿
            // 否则, 如果 offset 变化超过阈值, 拒绝此次 ICP 结果
            bool skip_rejection_check = !m_state.has_valid_offset || m_state.service_received;
            bool offset_rejected = false;
            
            if (!skip_rejection_check && 
                (delta_offset_t_norm > m_config.max_offset_jump_t || 
                 delta_offset_yaw_deg > m_config.max_offset_jump_yaw))
            {
                offset_rejected = true;
                m_state.icp_reject_count++;
                m_state.icp_reject_consecutive++;
                
                RCLCPP_WARN(this->get_logger(), 
                    "[ICP_REJECT] Offset jump too large! dt_2d=%.4fm (max=%.3f) dyaw=%.3fdeg (max=%.1f) "
                    "refine=%.4f | REJECTED (total=%lu consecutive=%lu)",
                    delta_offset_t_norm, m_config.max_offset_jump_t,
                    delta_offset_yaw_deg, m_config.max_offset_jump_yaw,
                    refine_score,
                    m_state.icp_reject_count, m_state.icp_reject_consecutive);
                
                // 如果连续拒绝过多次 (>10), 说明可能是真正的位置变化 (如手动搬运机器人)
                // 此时接受 ICP 结果, 重置连续计数
                if (m_state.icp_reject_consecutive > 10) {
                    offset_rejected = false;
                    RCLCPP_WARN(this->get_logger(), 
                        "[ICP_REJECT] Too many consecutive rejections (%lu), accepting result as position may have truly changed",
                        m_state.icp_reject_consecutive);
                    m_state.icp_reject_consecutive = 0;
                }
            } else {
                m_state.icp_reject_consecutive = 0;  // 正常结果, 重置连续拒绝计数
            }
            
            // 标准诊断日志
            RCLCPP_INFO(this->get_logger(), 
                "[DRIFT_DEBUG] ICP(map->body): t=(%.3f,%.3f,%.3f) rpy=(%.1f,%.1f,%.1f)deg",
                map_body_t.x(), map_body_t.y(), map_body_t.z(),
                map_body_roll * 180.0 / M_PI, map_body_pitch * 180.0 / M_PI, map_body_yaw * 180.0 / M_PI);
            RCLCPP_INFO(this->get_logger(), 
                "[DRIFT_DEBUG] LIO(local->body): t=(%.3f,%.3f,%.3f) yaw=%.1fdeg",
                current_local_t.x(), current_local_t.y(), current_local_t.z(),
                local_body_yaw * 180.0 / M_PI);
            RCLCPP_INFO(this->get_logger(), 
                "[DRIFT_DEBUG] Offset(map->local): t=(%.3f,%.3f,%.3f) yaw=%.1fdeg | "
                "delta: dt_2d=%.4fm dyaw=%.3fdeg | correction=%.4fm | %s",
                new_offset_t.x(), new_offset_t.y(), new_offset_t.z(),
                offset_yaw * 180.0 / M_PI,
                delta_offset_t_norm, delta_offset_yaw_deg, icp_correction,
                offset_rejected ? "REJECTED" : "ACCEPTED");
            
            if (!offset_rejected) {
                m_state.last_offset_r = new_offset_r;
                m_state.last_offset_t = new_offset_t;
                m_state.offset_ref_local_r = current_local_r;
                m_state.offset_ref_local_t = current_local_t;
                m_state.has_valid_offset = true;
            }
            
            if (!m_state.localize_success && m_state.service_received)
            {
                std::lock_guard<std::mutex> slock(m_state.service_mutex);
                m_state.localize_success = true;
                m_state.service_received = false;
            }
        }
        else
        {
            // ICP 失败统计
            m_diag.icp_fail_count++;
            m_diag.icp_consecutive_fails++;
            if (m_diag.icp_consecutive_fails > m_diag.icp_max_consecutive_fails) {
                m_diag.icp_max_consecutive_fails = m_diag.icp_consecutive_fails;
            }
            
            double gap = std::chrono::duration_cast<std::chrono::milliseconds>(
                icp_end_time - m_diag.last_icp_success_time).count() / 1000.0;
            if (gap > m_diag.max_icp_gap_seconds) m_diag.max_icp_gap_seconds = gap;
            
            RCLCPP_WARN(this->get_logger(), 
                "[ICP_FAIL] consecutive=%lu gap_since_success=%.1fs "
                "rough_score=%.4f refine_score=%.4f cloud=%zu",
                m_diag.icp_consecutive_fails, gap,
                rough_score, refine_score, cloud_size);
        }
        
        // 每30秒输出综合诊断报告
        auto now = std::chrono::steady_clock::now();
        double since_last_diag = std::chrono::duration_cast<std::chrono::seconds>(
            now - m_diag.last_diag_print_time).count();
        if (since_last_diag >= 30.0) {
            m_diag.last_diag_print_time = now;
            double uptime = std::chrono::duration_cast<std::chrono::seconds>(
                now - m_diag.start_time).count();
            double success_rate = m_diag.icp_total_count > 0 ? 
                (double)m_diag.icp_success_count / m_diag.icp_total_count * 100.0 : 0.0;
            double avg_score = m_diag.icp_success_count > 0 ? 
                m_diag.icp_score_sum / m_diag.icp_success_count : 0.0;
            double drift_from_origin = (m_diag.prev_odom_t - m_diag.first_odom_t).head<2>().norm();
            
            RCLCPP_INFO(this->get_logger(),
                "╔══════════════════════════════════════════════════════════════╗");
            RCLCPP_INFO(this->get_logger(),
                "║ [LOCALIZER DIAG] uptime=%.0fs  odom_distance=%.1fm          ║",
                uptime, m_diag.odom_total_distance);
            RCLCPP_INFO(this->get_logger(),
                "║ ICP: total=%lu success=%lu fail=%lu rate=%.1f%%              ║",
                m_diag.icp_total_count, m_diag.icp_success_count, 
                m_diag.icp_fail_count, success_rate);
            RCLCPP_INFO(this->get_logger(),
                "║ ICP score: avg=%.4f min=%.4f max=%.4f last=%.4f     ║",
                avg_score, 
                m_diag.icp_score_min < 1e9 ? m_diag.icp_score_min : 0.0,
                m_diag.icp_score_max, m_diag.icp_last_refine_score);
            RCLCPP_INFO(this->get_logger(),
                "║ Offset drift: cumulative=%.4fm max_single=%.4fm             ║",
                m_diag.cumulative_offset_drift, m_diag.max_delta_offset_t);
            RCLCPP_INFO(this->get_logger(),
                "║ Offset yaw: max_single=%.3fdeg                              ║",
                m_diag.max_delta_offset_yaw);
            RCLCPP_INFO(this->get_logger(),
                "║ Fails: max_consecutive=%lu max_gap=%.1fs                    ║",
                m_diag.icp_max_consecutive_fails, m_diag.max_icp_gap_seconds);
            RCLCPP_INFO(this->get_logger(),
                "║ Cloud: min=%zu max=%zu  LIO_dist_from_origin=%.2fm          ║",
                m_diag.min_cloud_size < SIZE_MAX ? m_diag.min_cloud_size : (size_t)0,
                m_diag.max_cloud_size, drift_from_origin);
            RCLCPP_INFO(this->get_logger(),
                "╚══════════════════════════════════════════════════════════════╝");
        }
        
        publishMapCloud(current_time);
    }

    void syncCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg, const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg)
    {
        const rclcpp::Time cloud_time(cloud_msg->header.stamp);
        const rclcpp::Time odom_time(odom_msg->header.stamp);
        const double time_diff = std::fabs((cloud_time - odom_time).seconds());
        if (time_diff > 0.1)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "cloud/odom time diff too large: %.3f s (cloud=%.9f, odom=%.9f)",
                time_diff, cloud_time.seconds(), odom_time.seconds());
        }

        const auto &p = odom_msg->pose.pose.position;
        const auto &q = odom_msg->pose.pose.orientation;
        const bool finite_pose =
            std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
            std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w);
        const double abs_limit = 1e6;  // meters, sanity limit
        if (!finite_pose || std::fabs(p.x) > abs_limit || std::fabs(p.y) > abs_limit || std::fabs(p.z) > abs_limit)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Received invalid odometry (non-finite or too large), dropping this sync pair");
            return;
        }

        std::lock_guard<std::mutex> lock(m_state.message_mutex);

        pcl::fromROSMsg(*cloud_msg, *m_state.last_cloud);

        m_state.last_r = Eigen::Quaterniond(odom_msg->pose.pose.orientation.w,
                                            odom_msg->pose.pose.orientation.x,
                                            odom_msg->pose.pose.orientation.y,
                                            odom_msg->pose.pose.orientation.z)
                             .toRotationMatrix();
        m_state.last_t = V3D(odom_msg->pose.pose.position.x,
                             odom_msg->pose.pose.position.y,
                             odom_msg->pose.pose.position.z);
        m_state.last_message_time = cloud_msg->header.stamp;
        m_state.new_data_available = true;
        
        // 诊断日志：每5秒输出一次 FAST-LIO2 的原始里程计
        static auto last_odom_log_time = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_odom_log_time).count() >= 5) {
            // 使用 atan2 提取 yaw，避免 eulerAngles 奇异性
            double yaw = std::atan2(m_state.last_r(1, 0), m_state.last_r(0, 0));
            double pitch = std::asin(-std::clamp(m_state.last_r(2, 0), -1.0, 1.0));
            double roll = std::atan2(m_state.last_r(2, 1), m_state.last_r(2, 2));
            RCLCPP_INFO(this->get_logger(), 
                "[SYNC_DEBUG] LIO odom (local->body): t=(%.3f,%.3f,%.3f) rpy=(%.1f,%.1f,%.1f)deg",
                m_state.last_t.x(), m_state.last_t.y(), m_state.last_t.z(),
                roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);
            last_odom_log_time = now;
        }
        
        if (!m_state.message_received)
        {
            m_state.message_received = true;
            m_config.local_frame = odom_msg->header.frame_id;
        }
        
        // Notify ICP thread
        m_cv.notify_one();
    }

    void sendBroadCastTF(builtin_interfaces::msg::Time &time)
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.frame_id = m_config.map_frame;
        transformStamped.child_frame_id = m_config.local_frame;
        transformStamped.header.stamp = time;
        Eigen::Quaterniond q(m_state.last_offset_r);
        V3D t = m_state.last_offset_t;
        transformStamped.transform.translation.x = t.x();
        transformStamped.transform.translation.y = t.y();
        transformStamped.transform.translation.z = t.z();
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        m_tf_broadcaster->sendTransform(transformStamped);
        
        // 每5秒输出一次 TF 发布的诊断信息
        static auto last_tf_log_time = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_tf_log_time).count() >= 5) {
            // 计算当前完整的 map → body 变换
            M3D current_map_body_r;
            V3D current_map_body_t;
            {
                std::lock_guard<std::mutex> lock(m_state.message_mutex);
                current_map_body_r = m_state.last_offset_r * m_state.last_r;
                current_map_body_t = m_state.last_offset_r * m_state.last_t + m_state.last_offset_t;
            }
            double yaw = std::atan2(current_map_body_r(1, 0), current_map_body_r(0, 0));
            
            // 计算 local 增量（相对于 offset 计算时的参考位姿）
            M3D delta_local_r;
            V3D delta_local_t;
            {
                std::lock_guard<std::mutex> lock(m_state.message_mutex);
                delta_local_r = m_state.offset_ref_local_r.transpose() * m_state.last_r;
                delta_local_t = m_state.last_t - m_state.offset_ref_local_t;
            }
            double delta_dist = delta_local_t.norm();
            double delta_yaw = std::atan2(delta_local_r(1, 0), delta_local_r(0, 0));
            
            RCLCPP_INFO(this->get_logger(), 
                "[TF_BROADCAST] map->body: t=(%.3f,%.3f,%.3f) yaw=%.1f deg | local_delta: dist=%.3f yaw=%.1f deg",
                current_map_body_t.x(), current_map_body_t.y(), current_map_body_t.z(),
                yaw * 180.0 / M_PI,
                delta_dist, delta_yaw * 180.0 / M_PI);
            last_tf_log_time = now;
        }
    }

    void relocCB(const std::shared_ptr<interface::srv::Relocalize::Request> request, std::shared_ptr<interface::srv::Relocalize::Response> response)
    {
        std::string pcd_path = request->pcd_path;
        float x = request->x;
        float y = request->y;
        float z = request->z;
        float yaw = request->yaw;
        float roll = request->roll;
        float pitch = request->pitch;

        RCLCPP_INFO(this->get_logger(), "Relocalize request: pcd=%s, x=%.3f, y=%.3f, z=%.3f, yaw=%.3f, roll=%.3f, pitch=%.3f",
            pcd_path.c_str(), x, y, z, yaw, roll, pitch);

        if (!std::filesystem::exists(pcd_path))
        {
            response->success = false;
            response->message = "pcd file not found";
            return;
        }

        Eigen::AngleAxisd yaw_angle = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd roll_angle = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitch_angle = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());
        bool load_flag = m_localizer->loadMap(pcd_path);
        RCLCPP_INFO(this->get_logger(), "Map load result: %s", load_flag ? "SUCCESS" : "FAILED");
        if (!load_flag)
        {
            response->success = false;
            response->message = "load map failed";
            return;
        }
        {
            std::lock_guard<std::mutex> lock(m_state.message_mutex);
            m_state.initial_guess.setIdentity();
            m_state.initial_guess.block<3, 3>(0, 0) = (yaw_angle * roll_angle * pitch_angle).toRotationMatrix().cast<float>();
            m_state.initial_guess.block<3, 1>(0, 3) = V3F(x, y, z);
            m_state.service_received = true;
            m_state.localize_success = false;
            RCLCPP_INFO(this->get_logger(), "Initial guess set: service_received=true, localize_success=false");
        }

        response->success = true;
        response->message = "relocalize service call accepted, waiting for ICP alignment in timerCB";
        return;
    }

    void relocCheckCB(const std::shared_ptr<interface::srv::IsValid::Request> request, std::shared_ptr<interface::srv::IsValid::Response> response)
    {
        std::lock_guard<std::mutex> slock(m_state.service_mutex);
        if (request->code == 1)
            response->valid = true;
        else
            response->valid = m_state.localize_success;
        return;
    }
    void publishMapCloud(builtin_interfaces::msg::Time &time)
    {
        if (m_map_cloud_pub->get_subscription_count() < 1)
            return;
        CloudType::Ptr map_cloud = m_localizer->refineMap();
        if (map_cloud->size() < 1)
            return;
        sensor_msgs::msg::PointCloud2 map_cloud_msg;
        pcl::toROSMsg(*map_cloud, map_cloud_msg);
        map_cloud_msg.header.frame_id = m_config.map_frame;
        map_cloud_msg.header.stamp = time;
        m_map_cloud_pub->publish(map_cloud_msg);
    }

private:
    NodeConfig m_config;
    NodeState m_state;

    ICPConfig m_localizer_config;
    std::shared_ptr<ICPLocalizer> m_localizer;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> m_cloud_sub;
    message_filters::Subscriber<nav_msgs::msg::Odometry> m_odom_sub;
    rclcpp::TimerBase::SharedPtr m_tf_timer;
    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>> m_sync;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
    rclcpp::Service<interface::srv::Relocalize>::SharedPtr m_reloc_srv;
    rclcpp::Service<interface::srv::IsValid>::SharedPtr m_reloc_check_srv;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_map_cloud_pub;
    
    // Thread management for async ICP processing
    std::thread m_process_thread;
    std::atomic<bool> m_running;
    std::mutex m_cv_mutex;
    std::condition_variable m_cv;
    
    // 诊断统计
    DiagnosticStats m_diag;
};

// Register as composed node
RCLCPP_COMPONENTS_REGISTER_NODE(LocalizerNode)

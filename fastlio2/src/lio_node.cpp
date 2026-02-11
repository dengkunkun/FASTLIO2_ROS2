#include <mutex>
#include <vector>
#include <queue>
#include <memory>
#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <algorithm>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

#include "utils.h"
#include "map_builder/commons.h"
#include "map_builder/map_builder.h"

#include <pcl_conversions/pcl_conversions.h>
#include "tf2_ros/transform_broadcaster.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <yaml-cpp/yaml.h>

// For component registration
#include <rclcpp_components/register_node_macro.hpp>

using namespace std::chrono_literals;
struct NodeConfig
{
    std::string imu_topic = "/livox/imu";
    std::string lidar_topic = "/livox/lidar";
    std::string body_frame = "body";
    std::string world_frame = "lidar";
    bool print_time_cost = false;
};
struct StateData
{
    bool lidar_pushed = false;
    std::mutex imu_mutex;
    std::mutex lidar_mutex;
    double last_lidar_time = -1.0;
    double last_imu_time = -1.0;
    std::deque<IMUData> imu_buffer;
    std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>> lidar_buffer;
    nav_msgs::msg::Path path;
};

class LIONode : public rclcpp::Node
{
public:
    explicit LIONode(const rclcpp::NodeOptions & options)
    : Node("lio_node", rclcpp::NodeOptions(options).enable_logger_service(true))
    , m_running(true)
    {
        RCLCPP_INFO(this->get_logger(), "LIO Node Started (Composed)");
        loadParameters();

        m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(m_node_config.imu_topic, 10, std::bind(&LIONode::imuCB, this, std::placeholders::_1));
        m_lidar_sub = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(m_node_config.lidar_topic, 10, std::bind(&LIONode::lidarCB, this, std::placeholders::_1));

        m_body_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("body_cloud", 10000);
        m_world_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("world_cloud", 10000);
        m_path_pub = this->create_publisher<nav_msgs::msg::Path>("lio_path", 10000);
        m_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("lio_odom", 10000);
        m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        m_state_data.path.poses.clear();
        m_state_data.path.header.frame_id = m_node_config.world_frame;

        m_kf = std::make_shared<IESKF>();
        m_builder = std::make_shared<MapBuilder>(m_builder_config, m_kf);
        
        if (m_builder_config.freeze_map) {
            RCLCPP_INFO(this->get_logger(), "[FREEZE_MAP] Map frozen mode ENABLED - ikd-Tree will NOT add new scan points");
        }
        if (m_builder_config.localization_mode) {
            RCLCPP_INFO(this->get_logger(), 
                "[LOCALIZATION_MODE] Enabled - ikd-Tree will auto-freeze after %d frames if not already frozen",
                m_builder_config.localization_freeze_delay_frames);
        }

        // Start processing thread for heavy computation (avoiding callback blocking)
        m_process_thread = std::thread(&LIONode::processLoop, this);
    }
    
    ~LIONode()
    {
        m_running = false;
        m_cv.notify_all();
        if (m_process_thread.joinable()) {
            m_process_thread.join();
        }
        RCLCPP_INFO(this->get_logger(), "LIO Node Shutdown");
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

        m_node_config.imu_topic = config["imu_topic"].as<std::string>();
        m_node_config.lidar_topic = config["lidar_topic"].as<std::string>();
        m_node_config.body_frame = config["body_frame"].as<std::string>();
        m_node_config.world_frame = config["world_frame"].as<std::string>();
        m_node_config.print_time_cost = config["print_time_cost"].as<bool>();

        m_builder_config.lidar_filter_num = config["lidar_filter_num"].as<int>();
        m_builder_config.lidar_min_range = config["lidar_min_range"].as<double>();
        m_builder_config.lidar_max_range = config["lidar_max_range"].as<double>();
        m_builder_config.scan_resolution = config["scan_resolution"].as<double>();
        m_builder_config.map_resolution = config["map_resolution"].as<double>();
        m_builder_config.cube_len = config["cube_len"].as<double>();
        m_builder_config.det_range = config["det_range"].as<double>();
        m_builder_config.move_thresh = config["move_thresh"].as<double>();
        m_builder_config.na = config["na"].as<double>();
        m_builder_config.ng = config["ng"].as<double>();
        m_builder_config.nba = config["nba"].as<double>();
        m_builder_config.nbg = config["nbg"].as<double>();

        m_builder_config.imu_init_num = config["imu_init_num"].as<int>();
        m_builder_config.near_search_num = config["near_search_num"].as<int>();
        m_builder_config.ieskf_max_iter = config["ieskf_max_iter"].as<int>();
        m_builder_config.gravity_align = config["gravity_align"].as<bool>();
        m_builder_config.esti_il = config["esti_il"].as<bool>();
        if (config["freeze_map"]) {
            m_builder_config.freeze_map = config["freeze_map"].as<bool>();
        }
        if (config["localization_mode"]) {
            m_builder_config.localization_mode = config["localization_mode"].as<bool>();
        }
        if (config["localization_freeze_delay_frames"]) {
            m_builder_config.localization_freeze_delay_frames = config["localization_freeze_delay_frames"].as<int>();
        }
        std::vector<double> t_il_vec = config["t_il"].as<std::vector<double>>();
        std::vector<double> r_il_vec = config["r_il"].as<std::vector<double>>();
        m_builder_config.t_il << t_il_vec[0], t_il_vec[1], t_il_vec[2];
        m_builder_config.r_il << r_il_vec[0], r_il_vec[1], r_il_vec[2], r_il_vec[3], r_il_vec[4], r_il_vec[5], r_il_vec[6], r_il_vec[7], r_il_vec[8];
        m_builder_config.lidar_cov_inv = config["lidar_cov_inv"].as<double>();
    }

    void imuCB(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(m_state_data.imu_mutex);
        double timestamp = Utils::getSec(msg->header);
        if (timestamp < m_state_data.last_imu_time)
        {
            RCLCPP_WARN(this->get_logger(), "IMU Message is out of order");
            std::deque<IMUData>().swap(m_state_data.imu_buffer);
        }
        m_state_data.imu_buffer.emplace_back(V3D(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z) * 10.0,
                                             V3D(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
                                             timestamp);
        m_state_data.last_imu_time = timestamp;
    }
    void lidarCB(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
    {
        CloudType::Ptr cloud = Utils::livox2PCL(msg, m_builder_config.lidar_filter_num, m_builder_config.lidar_min_range, m_builder_config.lidar_max_range);
        std::lock_guard<std::mutex> lock(m_state_data.lidar_mutex);
        double timestamp = Utils::getSec(msg->header);
        if (timestamp < m_state_data.last_lidar_time)
        {
            RCLCPP_WARN(this->get_logger(), "Lidar Message is out of order");
            std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>>().swap(m_state_data.lidar_buffer);
        }
        m_state_data.lidar_buffer.emplace_back(timestamp, cloud);
        m_state_data.last_lidar_time = timestamp;
        
        // Notify processing thread that new data is available
        m_cv.notify_one();
    }

    bool syncPackage()
    {
        std::lock_guard<std::mutex> imu_lock(m_state_data.imu_mutex);
        std::lock_guard<std::mutex> lidar_lock(m_state_data.lidar_mutex);
        if (m_state_data.imu_buffer.empty() || m_state_data.lidar_buffer.empty())
            return false;
        if (!m_state_data.lidar_pushed)
        {
            m_package.cloud = m_state_data.lidar_buffer.front().second;
            if (m_package.cloud->empty())
            {
                RCLCPP_WARN(this->get_logger(), "Lidar cloud empty, drop frame");
                m_state_data.lidar_buffer.pop_front();
                m_state_data.lidar_pushed = false;
                return false;
            }
            std::sort(m_package.cloud->points.begin(), m_package.cloud->points.end(), [](PointType &p1, PointType &p2)
                      { return p1.curvature < p2.curvature; });
            m_package.cloud_start_time = m_state_data.lidar_buffer.front().first;
            m_package.cloud_end_time = m_package.cloud_start_time + m_package.cloud->points.back().curvature / 1000.0;
            m_state_data.lidar_pushed = true;
        }
        if (m_state_data.last_imu_time < m_package.cloud_end_time)
            return false;

        Vec<IMUData>().swap(m_package.imus);
        while (!m_state_data.imu_buffer.empty() && m_state_data.imu_buffer.front().time < m_package.cloud_end_time)
        {
            m_package.imus.emplace_back(m_state_data.imu_buffer.front());
            m_state_data.imu_buffer.pop_front();
        }
        m_state_data.lidar_buffer.pop_front();
        m_state_data.lidar_pushed = false;
        return true;
    }

    void publishCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub, CloudType::Ptr cloud, std::string frame_id, const double &time)
    {
        if (pub->get_subscription_count() <= 0)
            return;
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_msg.header.frame_id = frame_id;
        cloud_msg.header.stamp = Utils::getTime(time);
        pub->publish(cloud_msg);
    }

    void publishOdometry(rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub, std::string frame_id, std::string child_frame, const double &time)
    {
        if (odom_pub->get_subscription_count() <= 0)
            return;
        nav_msgs::msg::Odometry odom;
        odom.header.frame_id = frame_id;
        odom.header.stamp = Utils::getTime(time);
        odom.child_frame_id = child_frame;
        odom.pose.pose.position.x = m_kf->x().t_wi.x();
        odom.pose.pose.position.y = m_kf->x().t_wi.y();
        odom.pose.pose.position.z = m_kf->x().t_wi.z();
        Eigen::Quaterniond q(m_kf->x().r_wi);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        V3D vel = m_kf->x().r_wi.transpose() * m_kf->x().v;
        odom.twist.twist.linear.x = vel.x();
        odom.twist.twist.linear.y = vel.y();
        odom.twist.twist.linear.z = vel.z();
        odom_pub->publish(odom);
    }

    void publishPath(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub, std::string frame_id, const double &time)
    {
        if (path_pub->get_subscription_count() <= 0)
            return;
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = frame_id;
        pose.header.stamp = Utils::getTime(time);
        pose.pose.position.x = m_kf->x().t_wi.x();
        pose.pose.position.y = m_kf->x().t_wi.y();
        pose.pose.position.z = m_kf->x().t_wi.z();
        Eigen::Quaterniond q(m_kf->x().r_wi);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
        m_state_data.path.poses.push_back(pose);
        path_pub->publish(m_state_data.path);
    }

    void broadCastTF(std::shared_ptr<tf2_ros::TransformBroadcaster> broad_caster, std::string frame_id, std::string child_frame, const double &time)
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.frame_id = frame_id;
        transformStamped.child_frame_id = child_frame;
        transformStamped.header.stamp = Utils::getTime(time);
        Eigen::Quaterniond q(m_kf->x().r_wi);
        V3D t = m_kf->x().t_wi;
        transformStamped.transform.translation.x = t.x();
        transformStamped.transform.translation.y = t.y();
        transformStamped.transform.translation.z = t.z();
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        broad_caster->sendTransform(transformStamped);
    }

    // Processing loop runs in separate thread to avoid blocking ROS callbacks
    void processLoop()
    {
        while (m_running) {
            // Wait for new data or shutdown signal
            {
                std::unique_lock<std::mutex> lock(m_cv_mutex);
                m_cv.wait_for(lock, 20ms, [this]() {
                    if (!m_running) {
                        return true;
                    }
                    std::lock_guard<std::mutex> lidar_lock(m_state_data.lidar_mutex);
                    return !m_state_data.lidar_buffer.empty();
                });
            }
            
            if (!m_running) break;
            
            // Process available data
            processOnce();
        }
    }

    void processOnce()
    {
        std::lock_guard<std::mutex> builder_lock(m_builder_mutex);
        if (!syncPackage())
            return;
        auto t1 = std::chrono::high_resolution_clock::now();
        m_builder->process(m_package);
        auto t2 = std::chrono::high_resolution_clock::now();

        if (m_node_config.print_time_cost)
        {
            auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
            RCLCPP_WARN(this->get_logger(), "Time cost: %.2f ms", time_used);
        }

        if (m_builder->status() != BuilderStatus::MAPPING)
            return;

        // ============================================================
        // 定位模式安全网：自动冻结 ikd-Tree
        // 在 IESKF 初始化完成后的 N 帧自动冻结，防止因外部脚本
        // 未调用 reset_mapping 而导致地图退化
        // ============================================================
        if (m_builder_config.localization_mode && !m_builder->isFreezeMap()) {
            m_localization_mode_frame_count++;
            if (m_localization_mode_frame_count >= m_builder_config.localization_freeze_delay_frames) {
                m_builder->setFreezeMap(true);
                RCLCPP_WARN(this->get_logger(),
                    "[FREEZE_MAP] Auto-freeze triggered by localization_mode after %d frames. "
                    "ikd-Tree will NOT add new scan points.",
                    m_localization_mode_frame_count);
            }
        }

        broadCastTF(m_tf_broadcaster, m_node_config.world_frame, m_node_config.body_frame, m_package.cloud_end_time);

        publishOdometry(m_odom_pub, m_node_config.world_frame, m_node_config.body_frame, m_package.cloud_end_time);

        CloudType::Ptr body_cloud = m_builder->lidar_processor()->transformCloud(m_package.cloud, m_kf->x().r_il, m_kf->x().t_il);

        publishCloud(m_body_cloud_pub, body_cloud, m_node_config.body_frame, m_package.cloud_end_time);

        CloudType::Ptr world_cloud = m_builder->lidar_processor()->transformCloud(m_package.cloud, m_builder->lidar_processor()->r_wl(), m_builder->lidar_processor()->t_wl());

        publishCloud(m_world_cloud_pub, world_cloud, m_node_config.world_frame, m_package.cloud_end_time);

        publishPath(m_path_pub, m_node_config.world_frame, m_package.cloud_end_time);
        
        // ============================================================
        // FAST-LIO2 状态诊断日志 (每10秒输出一次)
        // ============================================================
        m_lio_frame_count++;
        static auto last_lio_diag_time = std::chrono::steady_clock::now();
        auto now_diag = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now_diag - last_lio_diag_time).count() >= 10) {
            last_lio_diag_time = now_diag;
            
            const auto& state = m_kf->x();
            const auto& P = m_kf->P();
            
            // IMU 偏置
            RCLCPP_INFO(this->get_logger(),
                "[LIO_DIAG] IMU bias: bg=(%.6f,%.6f,%.6f) ba=(%.6f,%.6f,%.6f)",
                state.bg.x(), state.bg.y(), state.bg.z(),
                state.ba.x(), state.ba.y(), state.ba.z());
            
            // 偏置幅值 (大偏置可能说明 IMU 估计有问题)
            double bg_norm = state.bg.norm();
            double ba_norm = state.ba.norm();
            if (bg_norm > 0.01 || ba_norm > 0.5) {
                RCLCPP_WARN(this->get_logger(),
                    "[LIO_DIAG] IMU bias WARNING! |bg|=%.6f rad/s |ba|=%.4f m/s^2 "
                    "(large bias may cause drift)",
                    bg_norm, ba_norm);
            }
            
            // 重力方向 (应该接近 [0,0,-9.81])
            RCLCPP_INFO(this->get_logger(),
                "[LIO_DIAG] gravity=(%.4f,%.4f,%.4f) |g|=%.4f",
                state.g.x(), state.g.y(), state.g.z(), state.g.norm());
            
            // 位姿和速度
            double yaw = std::atan2(state.r_wi(1,0), state.r_wi(0,0));
            double pitch = std::asin(-std::clamp(state.r_wi(2,0), -1.0, 1.0));
            double roll = std::atan2(state.r_wi(2,1), state.r_wi(2,2));
            RCLCPP_INFO(this->get_logger(),
                "[LIO_DIAG] pose: t=(%.3f,%.3f,%.3f) rpy=(%.1f,%.1f,%.1f)deg v=(%.3f,%.3f,%.3f)",
                state.t_wi.x(), state.t_wi.y(), state.t_wi.z(),
                roll*180.0/M_PI, pitch*180.0/M_PI, yaw*180.0/M_PI,
                state.v.x(), state.v.y(), state.v.z());
            
            // 协方差对角线 (越大表示越不确定)
            // P 矩阵: [rot(3), pos(3), rot_il(3), pos_il(3), vel(3), bg(3), ba(3)] = 21维
            // 位置对角线: P(3,3), P(4,4), P(5,5)
            // 旋转对角线: P(0,0), P(1,1), P(2,2)
            // 偏置: bg P(15..17), ba P(18..20)
            RCLCPP_INFO(this->get_logger(),
                "[LIO_DIAG] P_diag: rot=(%.2e,%.2e,%.2e) pos=(%.2e,%.2e,%.2e) bg=(%.2e,%.2e,%.2e) ba=(%.2e,%.2e,%.2e)",
                P(0,0), P(1,1), P(2,2),
                P(3,3), P(4,4), P(5,5),
                P(15,15), P(16,16), P(17,17),
                P(18,18), P(19,19), P(20,20));
            
            // 帧率统计
            RCLCPP_INFO(this->get_logger(),
                "[LIO_DIAG] frames=%lu cloud_points=%zu freeze_map=%s ikdtree_size=%d", 
                m_lio_frame_count, m_package.cloud->size(),
                m_builder->isFreezeMap() ? "ON" : "OFF",
                m_builder->getIkdTreeSize());
        }
    }

    // Keep timerCB for backward compatibility (not used with thread model)
    void timerCB()
    {
        processOnce();
    }

private:
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr m_lidar_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_body_cloud_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_world_cloud_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_path_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom_pub;

    rclcpp::TimerBase::SharedPtr m_timer;
    StateData m_state_data;
    SyncPackage m_package;
    NodeConfig m_node_config;
    Config m_builder_config;
    std::shared_ptr<IESKF> m_kf;
    std::shared_ptr<MapBuilder> m_builder;
    std::mutex m_builder_mutex;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
    
    // Thread management for async processing
    std::thread m_process_thread;
    std::atomic<bool> m_running;
    std::mutex m_cv_mutex;
    std::condition_variable m_cv;
    
    // 诊断计数器
    uint64_t m_lio_frame_count = 0;
    int m_localization_mode_frame_count = 0;  // 定位模式延迟冻结帧计数
};

// Register as composed node
RCLCPP_COMPONENTS_REGISTER_NODE(LIONode)
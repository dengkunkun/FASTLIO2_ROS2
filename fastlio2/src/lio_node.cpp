#include <mutex>
#include <vector>
#include <queue>
#include <memory>
#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <filesystem>
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

// ResetMapping service
#include "interface/srv/reset_mapping.hpp"

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
    std::mutex reset_mutex;  // 重置操作互斥锁
    double last_lidar_time = -1.0;
    double last_imu_time = -1.0;
    std::deque<IMUData> imu_buffer;
    std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>> lidar_buffer;
    nav_msgs::msg::Path path;
    bool reset_pending = false;  // 是否有待处理的重置请求
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

        // ResetMapping 服务
        m_reset_srv = this->create_service<interface::srv::ResetMapping>(
            "reset_mapping",
            std::bind(&LIONode::resetMappingCB, this, std::placeholders::_1, std::placeholders::_2));

        m_state_data.path.poses.clear();
        m_state_data.path.header.frame_id = m_node_config.world_frame;

        m_kf = std::make_shared<IESKF>();
        m_builder = std::make_shared<MapBuilder>(m_builder_config, m_kf);
        
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
        if (m_state_data.imu_buffer.empty() || m_state_data.lidar_buffer.empty())
            return false;
        if (!m_state_data.lidar_pushed)
        {
            m_package.cloud = m_state_data.lidar_buffer.front().second;
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
                    return !m_running || !m_state_data.lidar_buffer.empty();
                });
            }
            
            if (!m_running) break;
            
            // 检查是否有待处理的重置请求
            {
                std::lock_guard<std::mutex> lock(m_state_data.reset_mutex);
                if (m_state_data.reset_pending)
                {
                    // 重置进行中，跳过本次处理
                    continue;
                }
            }
            
            // Process available data
            processOnce();
        }
    }

    void processOnce()
    {
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

        broadCastTF(m_tf_broadcaster, m_node_config.world_frame, m_node_config.body_frame, m_package.cloud_end_time);

        publishOdometry(m_odom_pub, m_node_config.world_frame, m_node_config.body_frame, m_package.cloud_end_time);

        CloudType::Ptr body_cloud = m_builder->lidar_processor()->transformCloud(m_package.cloud, m_kf->x().r_il, m_kf->x().t_il);

        publishCloud(m_body_cloud_pub, body_cloud, m_node_config.body_frame, m_package.cloud_end_time);

        CloudType::Ptr world_cloud = m_builder->lidar_processor()->transformCloud(m_package.cloud, m_builder->lidar_processor()->r_wl(), m_builder->lidar_processor()->t_wl());

        publishCloud(m_world_cloud_pub, world_cloud, m_node_config.world_frame, m_package.cloud_end_time);

        publishPath(m_path_pub, m_node_config.world_frame, m_package.cloud_end_time);
    }

    // Keep timerCB for backward compatibility (not used with thread model)
    void timerCB()
    {
        processOnce();
    }

    /**
     * @brief ResetMapping 服务回调
     * @details 处理重置建图请求，支持：
     *   1. 固定点重建：指定初始位姿，从零开始建图
     *   2. 已有地图继续建图：加载 PCD 作为初始地图
     */
    void resetMappingCB(
        const std::shared_ptr<interface::srv::ResetMapping::Request> request,
        std::shared_ptr<interface::srv::ResetMapping::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "ResetMapping request received");
        RCLCPP_INFO(this->get_logger(), "  Pose: x=%.3f, y=%.3f, z=%.3f, roll=%.3f, pitch=%.3f, yaw=%.3f",
            request->x, request->y, request->z, request->roll, request->pitch, request->yaw);
        RCLCPP_INFO(this->get_logger(), "  Options: load_map=%d, reuse_bias=%d, clear_path=%d",
            request->load_map, request->reuse_bias, request->clear_path);

        // 检查地图文件是否存在（如果需要加载）
        if (request->load_map && !request->map_pcd_path.empty())
        {
            if (!std::filesystem::exists(request->map_pcd_path))
            {
                response->success = false;
                response->message = "Map PCD file not found: " + request->map_pcd_path;
                RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
                return;
            }
            RCLCPP_INFO(this->get_logger(), "  Map path: %s", request->map_pcd_path.c_str());
        }

        // 设置重置标志，暂停处理循环
        {
            std::lock_guard<std::mutex> lock(m_state_data.reset_mutex);
            m_state_data.reset_pending = true;
        }

        // 等待处理循环暂停（最多等待 500ms）
        std::this_thread::sleep_for(100ms);

        // 清空数据缓冲区
        {
            std::lock_guard<std::mutex> imu_lock(m_state_data.imu_mutex);
            std::lock_guard<std::mutex> lidar_lock(m_state_data.lidar_mutex);
            m_state_data.imu_buffer.clear();
            m_state_data.lidar_buffer.clear();
            m_state_data.lidar_pushed = false;
            m_state_data.last_imu_time = -1.0;
            m_state_data.last_lidar_time = -1.0;
        }

        // 构建重置请求
        ResetRequest reset_req;

        // 计算旋转矩阵 (ZYX 欧拉角: yaw-pitch-roll)
        Eigen::AngleAxisd yaw_angle(request->yaw, Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd pitch_angle(request->pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd roll_angle(request->roll, Eigen::Vector3d::UnitX());
        reset_req.r_wi = (yaw_angle * pitch_angle * roll_angle).toRotationMatrix();
        reset_req.t_wi = V3D(request->x, request->y, request->z);

        reset_req.load_map = request->load_map;
        reset_req.map_path = request->map_pcd_path;
        reset_req.voxel_size = 0.0;  // 使用地图原始分辨率
        reset_req.reuse_bias = request->reuse_bias;

        // 执行重置
        bool success = m_builder->reset(reset_req);

        // 清除轨迹（如果需要）
        if (success && request->clear_path)
        {
            m_state_data.path.poses.clear();
            RCLCPP_INFO(this->get_logger(), "Path history cleared");
        }

        // 恢复处理循环
        {
            std::lock_guard<std::mutex> lock(m_state_data.reset_mutex);
            m_state_data.reset_pending = false;
        }

        // 设置响应
        if (success)
        {
            response->success = true;
            response->message = "Reset mapping successful";
            RCLCPP_INFO(this->get_logger(), "ResetMapping completed successfully");
        }
        else
        {
            response->success = false;
            response->message = "Reset mapping failed";
            RCLCPP_ERROR(this->get_logger(), "ResetMapping failed");
        }
    }

private:
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr m_lidar_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_body_cloud_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_world_cloud_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_path_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom_pub;

    rclcpp::Service<interface::srv::ResetMapping>::SharedPtr m_reset_srv;

    rclcpp::TimerBase::SharedPtr m_timer;
    StateData m_state_data;
    SyncPackage m_package;
    NodeConfig m_node_config;
    Config m_builder_config;
    std::shared_ptr<IESKF> m_kf;
    std::shared_ptr<MapBuilder> m_builder;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
    
    // Thread management for async processing
    std::thread m_process_thread;
    std::atomic<bool> m_running;
    std::mutex m_cv_mutex;
    std::condition_variable m_cv;
};

// Register as composed node
RCLCPP_COMPONENTS_REGISTER_NODE(LIONode)
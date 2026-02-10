#include "map_builder.h"
MapBuilder::MapBuilder(Config &config, std::shared_ptr<IESKF> kf) : m_config(config), m_kf(kf)
{
    m_imu_processor = std::make_shared<IMUProcessor>(config, kf);
    m_lidar_processor = std::make_shared<LidarProcessor>(config, kf);
    m_status = BuilderStatus::IMU_INIT;
}

void MapBuilder::process(SyncPackage &package)
{
    if (m_status == BuilderStatus::IMU_INIT)
    {
        if (m_imu_processor->initialize(package))
            m_status = BuilderStatus::MAP_INIT;
        return;
    }

    m_imu_processor->undistort(package);

    if (m_status == BuilderStatus::MAP_INIT)
    {
        CloudType::Ptr cloud_world = LidarProcessor::transformCloud(package.cloud, m_lidar_processor->r_wl(), m_lidar_processor->t_wl());
        m_lidar_processor->initCloudMap(cloud_world->points);
        m_status = BuilderStatus::MAPPING;
        return;
    }
    
    m_lidar_processor->process(package);
}

bool MapBuilder::reset(const ResetRequest &req)
{
    std::cout << "[MapBuilder] Resetting with pose: t=(" << req.t_wi.x() << ", " << req.t_wi.y() << ", " << req.t_wi.z() << ")" << std::endl;

    // 1. 重置 IMU 处理器
    m_imu_processor->reset(req.reuse_bias);

    // 2. 重置 Lidar 处理器
    m_lidar_processor->reset();

    // 3. 设置 IESKF 初始状态
    m_kf->x().r_wi = req.r_wi;
    m_kf->x().t_wi = req.t_wi;
    m_kf->x().v = V3D::Zero();

    // 重置协方差矩阵
    m_kf->P().setIdentity();
    m_kf->P().block<3, 3>(6, 6) = M3D::Identity() * 0.00001;
    m_kf->P().block<3, 3>(9, 9) = M3D::Identity() * 0.00001;
    m_kf->P().block<3, 3>(15, 15) = M3D::Identity() * 0.0001;
    m_kf->P().block<3, 3>(18, 18) = M3D::Identity() * 0.0001;

    // 4. 处理地图加载
    if (req.load_map && !req.map_path.empty())
    {
        bool load_success = m_lidar_processor->loadMapFromPCD(
            req.map_path, req.voxel_size, req.has_map_to_world_tf, req.r_w_m, req.t_w_m);
        if (!load_success)
        {
            std::cerr << "[MapBuilder] Failed to load map from: " << req.map_path << std::endl;
            m_status = BuilderStatus::IMU_INIT;
            return false;
        }
        // 已加载地图，直接进入 MAPPING 状态
        m_status = BuilderStatus::MAPPING;
        std::cout << "[MapBuilder] Loaded existing map, entering MAPPING state" << std::endl;
    }
    else
    {
        // 不加载地图，需要从首帧点云初始化
        m_status = BuilderStatus::MAP_INIT;
        std::cout << "[MapBuilder] No map loaded, entering MAP_INIT state" << std::endl;
    }

    return true;
}

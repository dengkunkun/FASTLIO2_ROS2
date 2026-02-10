#pragma once
#include "imu_processor.h"
#include "lidar_processor.h"

enum BuilderStatus
{
    IMU_INIT,
    MAP_INIT,
    MAPPING
};

/**
 * @brief 重置建图请求参数
 */
struct ResetRequest
{
    // 初始位姿
    M3D r_wi = M3D::Identity();  ///< 初始旋转 (world -> imu)
    V3D t_wi = V3D::Zero();      ///< 初始平移 (world -> imu)

    // 地图加载选项
    bool load_map = false;       ///< 是否加载已有地图
    std::string map_path;        ///< PCD 地图路径
    double voxel_size = 0.0;     ///< 地图降采样体素大小（0表示不降采样）
    // 当 map_path 指向的 PCD 位于 map_frame 时，需提供 T_world_map 用于变换到 world_frame。
    bool has_map_to_world_tf = false;
    M3D r_w_m = M3D::Identity(); ///< R_world_map
    V3D t_w_m = V3D::Zero();     ///< t_world_map

    // 其他选项
    bool reuse_bias = false;     ///< 是否复用上一次的 IMU 偏置估计
};

class MapBuilder
{
public:
    MapBuilder(Config &config, std::shared_ptr<IESKF> kf);

    void process(SyncPackage &package);
    BuilderStatus status() { return m_status; }    
    std::shared_ptr<LidarProcessor> lidar_processor(){return m_lidar_processor;}
    std::shared_ptr<IMUProcessor> imu_processor(){return m_imu_processor;}

    /// 设置是否冻结地图（定位模式时不往ikd-Tree添加新点）
    void setFreezeMap(bool freeze) { m_lidar_processor->setFreezeMap(freeze); }
    bool isFreezeMap() const { return m_lidar_processor->isFreezeMap(); }

    /// 获取 ikd-Tree 有效点数量（诊断用）
    int getIkdTreeSize() const { return m_lidar_processor->getIkdTreeSize(); }

    /**
     * @brief 重置建图状态，支持在已有地图上重新建图
     * @param req 重置请求参数
     * @return 是否重置成功
     * @details 流程：
     *   1. 重置 IMU 处理器（可选保留偏置）
     *   2. 重置 Lidar 处理器的 KD-Tree 和 LocalMap
     *   3. 设置初始位姿到 IESKF 状态
     *   4. 可选：从 PCD 加载已有地图到 KD-Tree
     *   5. 根据是否加载地图决定进入 MAP_INIT 或 MAPPING 状态
     */
    bool reset(const ResetRequest &req);

    /**
     * @brief 定位模式专用：仅重载 ikd-Tree 并冻结，不触碰 IESKF/IMU 状态
     * @param pcd_path PCD 地图文件路径
     * @param has_map_to_world_tf 是否提供 map->world TF
     * @param r_w_m world<-map 旋转
     * @param t_w_m world<-map 平移
     * @return 是否成功
     * @details 仅执行：清空ikd-Tree → 加载PCD → 冻结地图。
     *          保持当前 IESKF 位姿、IMU 偏置、所有滤波器状态不变。
     */
    bool loadAndFreezeMap(const std::string &pcd_path,
                         bool has_map_to_world_tf = false,
                         const M3D &r_w_m = M3D::Identity(),
                         const V3D &t_w_m = V3D::Zero());

private:
    Config m_config;
    BuilderStatus m_status;
    bool m_has_prior_map_loaded = false;
    std::shared_ptr<IESKF> m_kf;
    std::shared_ptr<IMUProcessor> m_imu_processor;
    std::shared_ptr<LidarProcessor> m_lidar_processor;
};

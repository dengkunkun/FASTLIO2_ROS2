#pragma once
#include "commons.h"
#include "ieskf.h"
#include "ikd_Tree.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

struct LocalMap
{
    bool initialed = false;
    BoxPointType local_map_corner;
    Vec<BoxPointType> cub_to_rm;
};

class LidarProcessor
{
public:
    LidarProcessor(Config &config, std::shared_ptr<IESKF> kf);

    void trimCloudMap();

    void incrCloudMap();

    void initCloudMap(PointVec &point_vec);

    void process(SyncPackage &package);

    void updateLossFunc(State &state, SharedState &share_data);

    static CloudType::Ptr transformCloud(CloudType::Ptr inp, const M3D &r, const V3D &t);
    M3D r_wl() { return m_kf->x().r_wi * m_kf->x().r_il; }
    V3D t_wl() { return m_kf->x().t_wi + m_kf->x().r_wi * m_kf->x().t_il; }

    /**
     * @brief 重置 LidarProcessor 状态
     * @details 清空 KD-Tree 和 LocalMap，恢复到初始状态
     */
    void reset();

    /**
     * @brief 从 PCD 文件加载地图作为初始地图
     * @param pcd_path PCD 文件路径
     * @param voxel_size 体素滤波大小，0 表示不滤波
     * @return 是否加载成功
     * @details 加载已有地图到 KD-Tree，支持从已有地图继续建图
     */
    bool loadMapFromPCD(const std::string &pcd_path, double voxel_size = 0.0);

private:
    Config m_config;
    LocalMap m_local_map;
    std::shared_ptr<IESKF> m_kf;
    std::shared_ptr<KD_TREE<PointType>> m_ikdtree;
    CloudType::Ptr m_cloud_lidar;
    CloudType::Ptr m_cloud_down_lidar;
    CloudType::Ptr m_cloud_down_world;
    std::vector<bool> m_point_selected_flag;
    CloudType::Ptr m_norm_vec;
    CloudType::Ptr m_effect_cloud_lidar;
    CloudType::Ptr m_effect_norm_vec;
    std::vector<PointVec> m_nearest_points;
    pcl::VoxelGrid<PointType> m_scan_filter;
    size_t m_buffer_size = 0;  // Dynamic buffer size for point cloud processing
};
